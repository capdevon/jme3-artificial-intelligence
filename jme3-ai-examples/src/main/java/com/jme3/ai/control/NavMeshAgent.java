package com.jme3.ai.control;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.ai.navmesh.NavMesh;
import com.jme3.ai.navmesh.NavMeshPathfinder;
import com.jme3.ai.navmesh.Path.Waypoint;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;

/**
 * No Thread
 * @author capdevon
 */
public class NavMeshAgent extends AbstractControl {

    private static final Logger logger = Logger.getLogger(NavMeshAgent.class.getName());

    private BetterCharacterControl bcc;
    private final Quaternion lookRotation = new Quaternion();
    private final Vector3f walkDirection = new Vector3f();
    private final Vector3f viewDirection = new Vector3f();
    private final Vector3f position2D = new Vector3f();
    private final Vector3f waypoint2D = new Vector3f();
    
    private float radius = 1f;
    private NavMesh navMesh;
    private NavMeshPathfinder nav;
    private PathViewer pathViewer;
    
    private boolean hasPath;
    private boolean pathChanged;
    private boolean stopped = false;
    
    // Stop within this distance from the target position.
    private float stoppingDistance = .25f;
    // Maximum movement speed when following a path.
    private float speed = 4;
    // Maximum turning speed in (deg/s) while following a path.
    private float angularSpeed = 6;
    // Should the agent update the transform orientation?
    private boolean updateRotation = true;

    /**
     * Instantiate a NavMeshAgent.
     * @param mesh
     */
    public NavMeshAgent(Mesh mesh) {
        this.navMesh = new NavMesh(mesh);
        this.nav = new NavMeshPathfinder(navMesh);
        this.nav.setEntityRadius(radius);
    }

    @Override
    public void setSpatial(Spatial sp) {
        super.setSpatial(sp);
        if (spatial != null) {
            this.pathViewer = spatial.getControl(PathViewer.class);
            this.bcc = spatial.getControl(BetterCharacterControl.class);
            Objects.requireNonNull(bcc, "BetterCharacterControl not found: " + spatial);
        }
    }

    @Override
    protected void controlUpdate(float tpf) {

        if (pathChanged) {
            drawPath();
            pathChanged = false;
        }

        if (stopped) {
            bcc.setWalkDirection(Vector3f.ZERO);
            return;
        }

        /**
         * getNextWayPoint will return always the same waypoint until we manually
         * advance to the next
         */
        Waypoint wayPoint = nav.getNextWaypoint();

        if (wayPoint != null) {
            // Gets the movement direction
            position2D.set(spatial.getWorldTranslation()).setY(0);
            waypoint2D.set(wayPoint.getPosition()).setY(0);

            // If they are more than one world distance unit they are not at the goal and
            // keep finding
            float remainingDistance = position2D.distance(waypoint2D);

            // Move the spatial to location while its not there
            if (remainingDistance > stoppingDistance) {
                Vector3f dir = waypoint2D.subtract(position2D, walkDirection).normalizeLocal();
                moveTo(dir, tpf);

            } // If at the final waypoint set at goal to true
            else if (nav.isAtGoalWaypoint()) {
                resetPath();

            } // If less than one from current waypoint and not the goal, go to next waypoint
            else {
                nav.goToNextWaypoint();
            }
        }
    }

    private void moveTo(Vector3f dir, float tpf) {
        if (updateRotation && dir.lengthSquared() > 0) {
            lookRotation.lookAt(dir, Vector3f.UNIT_Y);
            smoothDamp(spatial.getWorldRotation(), lookRotation, angularSpeed * tpf, viewDirection);
            bcc.setViewDirection(viewDirection);
        }
        bcc.setWalkDirection(dir.multLocal(speed));
    }
    
    /**
     * Spherically interpolates between quaternions a and b by ratio t. The
     * parameter t is clamped to the range [0, 1].
     */
    private Vector3f smoothDamp(Quaternion from, Quaternion to, float smoothTime, Vector3f store) {
        float changeAmount = FastMath.clamp(smoothTime, 0, 1);
        from.slerp(to, changeAmount);
        return from.mult(Vector3f.UNIT_Z, store);
    }
    
    /**
     * Set the destination of the agent in world-space units.
     * @param targetPos
     */
    public void setDestination(Vector3f targetPos) {
        nav.clearPath();
        nav.setPosition(spatial.getWorldTranslation());
        nav.warpInside(targetPos);
        
        hasPath = nav.computePath(targetPos);
        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Path found: {0}", hasPath);
        }

        if (hasPath) {
            // display motion path
            pathChanged = true;
        } else {
            clearPath();
            bcc.setWalkDirection(Vector3f.ZERO);
        }
    }

    /**
     * Clears the current path.
     */
    public void resetPath() {
        clearPath();
        nav.clearPath();
        bcc.setWalkDirection(Vector3f.ZERO);
        hasPath = false;
    }

    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }
    
    /**
     * Displays a motion path showing each waypoint. 
     * Stays in scene until another path is set.
     */
    private void drawPath() {
        if (pathViewer != null) {
            pathViewer.drawPath(nav.getPath());
        }
    }

    private void clearPath() {
        if (pathViewer != null) {
            pathViewer.clearPath();
        }
    }

    /**
     * @return The distance between the agent's position and the destination on
     * the current path. (Read Only)
     */
//    public float remainingDistance() {
//        float pathLength = 0f;
//        ArrayList<Waypoint> corners = nav.getPath().getWaypoints();
//        for (int j = 0; j < corners.size(); j++) {
//            Vector3f a = (j == 0) ? spatial.getWorldTranslation() : corners.get(j - 1).getPosition();
//            Vector3f b = corners.get(j).getPosition();
//            pathLength += a.distance(b);
//        }
//        return pathLength;
//    }
    
    /**
     * @return Corner points of the path. (Read Only)
     */
    public List<Vector3f> getCorners() {
        List<Vector3f> results = new ArrayList<>();
        getCornersNonAlloc(results);
        return results;
    }
    
    /**
     * Calculate the corners for the path.
     * 
     * @param results List to store path corners.
     * @return The number of corners along the path - including start and end points.
     */
    public int getCornersNonAlloc(List<Vector3f> results) {
        results.clear();
        for (Waypoint waypoint : nav.getPath().getWaypoints()) {
            Vector3f v = waypoint.getPosition().clone();
            results.add(v);
        }
        return results.size();
    }
    
    public float getSpeed() {
        return speed;
    }

    /**
     * Set the maximum movement speed when following a path.
     * @param speed
     */
    public void setSpeed(float speed) {
        this.speed = speed;
    }

    public float getAngularSpeed() {
        return angularSpeed;
    }

    /**
     * Maximum turning speed in (deg/s) while following a path.
     * @param angularSpeed
     */
    public void setAngularSpeed(float angularSpeed) {
        this.angularSpeed = angularSpeed;
    }

    public float getRadius() {
        return radius;
    }

    /**
     * Set the avoidance radius for the agent.
     * @param radius
     */
    public void setRadius(float radius) {
        this.radius = radius;
        this.nav.setEntityRadius(radius);
    }

    public float getStoppingDistance() {
        return stoppingDistance;
    }

    /**
     * Stop within this distance from the target position.
     * @param stoppingDistance
     */
    public void setStoppingDistance(float stoppingDistance) {
        this.stoppingDistance = stoppingDistance;
    }

    public boolean isStopped() {
        return stopped;
    }

    /**
     * This property holds the stop or resume condition of the NavMesh agent.
     * @param stopped
     */
    public void setStopped(boolean stopped) {
        this.stopped = stopped;
    }
    
    public boolean isUpdateRotation() {
        return updateRotation;
    }

    /**
     * Should the agent update the transform orientation?
     * @param updateRotation
     */
    public void setUpdateRotation(boolean updateRotation) {
        this.updateRotation = updateRotation;
    }

    public boolean hasPath() {
        return hasPath;
    }

}
