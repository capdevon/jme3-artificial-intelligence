package com.jme3.ai.control;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.ai.navmesh.NavMesh;
import com.jme3.ai.navmesh.NavMeshPathfinder;
import com.jme3.ai.navmesh.Path;
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
 *
 * @author capdevon
 */
public class NavMeshAgentMT extends AbstractControl {

    private static final Logger logger = Logger.getLogger(NavMeshAgentMT.class.getName());

    private PathViewer pathViewer;
    private final NavMesh navMesh;
    private final NavMeshPathfinder nav;
    private final ScheduledExecutorService executor;
    
    private BetterCharacterControl bcc;
    private Vector3f targetPos;
    private final Vector3f position2D = new Vector3f();
    private final Vector3f waypoint2D = new Vector3f();
    private final Vector3f viewDirection = new Vector3f(0, 0, 1);
    private final Quaternion lookRotation = new Quaternion();
    private float radius = 1f;

    private boolean hasPath;
    // Is a path in the process of being computed but not yet ready? (Read Only)
    private boolean pathPending;
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
    public NavMeshAgentMT(Mesh mesh) {
        this(mesh, null);
    }

    /**
     * Instantiate a NavMeshAgent.
     * @param mesh
     * @param pathViewer
     */
    public NavMeshAgentMT(Mesh mesh, PathViewer pathViewer) {
        this.executor = Executors.newScheduledThreadPool(1);
        this.navMesh = new NavMesh(mesh);
        this.nav = new NavMeshPathfinder(navMesh);
        this.nav.setEntityRadius(radius);
        this.pathViewer = pathViewer;
    }

    @Override
    public void setSpatial(Spatial sp) {
        super.setSpatial(sp);

        if (spatial != null) {
            this.bcc = spatial.getControl(BetterCharacterControl.class);
            Objects.requireNonNull(bcc, "BetterCharacterControl not found: " + spatial);
            startPathfinder();

        } else {
            stopPathfinder();
        }
    }

    @Override
    protected void controlUpdate(float tpf) {
        if (pathPending) {
            return;
        }

        if (pathChanged) {
            drawPath();
            pathChanged = false;
        }

        if (stopped) {
            bcc.setWalkDirection(Vector3f.ZERO);
            return;
        }

        /**
         * getNextWayPoint will return always the same waypoint until we
         * manually advance to the next
         */
        Path.Waypoint wayPoint = nav.getNextWaypoint();

        if (wayPoint != null) {

            position2D.set(spatial.getWorldTranslation());
            position2D.y = 0;

            waypoint2D.set(wayPoint.getPosition());
            waypoint2D.y = 0;

            float remainingDistance = position2D.distance(waypoint2D);

            // Move the spatial to location while its not there
            if (remainingDistance > stoppingDistance) {
                Vector3f dir = waypoint2D.subtract(position2D).normalizeLocal();
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

    private void startPathfinder() {
        executor.scheduleWithFixedDelay(new Runnable() {

            @Override
            public void run() {
                if (targetPos != null) {

                    nav.clearPath();
                    pathPending = true;

                    nav.setPosition(spatial.getWorldTranslation());
                    nav.warpInside(targetPos);

                    hasPath = nav.computePath(targetPos);
                    if (logger.isLoggable(Level.FINE)) {
                        logger.log(Level.FINE, "Path found: {0}", hasPath);
                    }

                    if (hasPath) {
                        // display motion path
                        pathChanged = true;
                    }

                    targetPos = null;
                    pathPending = false;
                }
            }
        }, 0, 500, TimeUnit.MILLISECONDS);
    }

    private void stopPathfinder() {
        // Disable new tasks from being submitted
        executor.shutdown();
        try {
            if (!executor.awaitTermination(6, TimeUnit.SECONDS)) {
                logger.log(Level.SEVERE, "Pool did not terminate {0}", executor);
                executor.shutdownNow();
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
            Thread.currentThread().interrupt();
        }
        logger.log(Level.INFO, "shutdown {0}", executor);
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

    /**
     * @return The distance between the agent's position and the destination on
     * the current path. (Read Only)
     */
    public float remainingDistance() {
        float pathLength = 0f;
        ArrayList<Waypoint> corners = nav.getPath().getWaypoints();
        for (int j = 0; j < corners.size(); j++) {
            Vector3f a = (j == 0) ? spatial.getWorldTranslation() : corners.get(j - 1).getPosition();
            Vector3f b = corners.get(j).getPosition();
            pathLength += a.distance(b);
        }
        return pathLength;
    }
    
    /**
     * @return Corner points of the path. (Read Only)
     */
    public List<Vector3f> getCorners() {
        List<Vector3f> result = new ArrayList<>();
        for (Waypoint waypoint : nav.getPath().getWaypoints()) {
            Vector3f v = waypoint.getPosition().clone();
            result.add(v);
        }
        return result;
    }

    /**
     * Displays a motion path showing each waypoint. Stays in scene until
     * another path is set.
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

    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
        if (pathViewer != null) {
            pathViewer.show(rm, vp);
        }
    }

    /**
     * Set the destination of the agent in world-space units.
     * @param targetPos
     */
    public void setDestination(Vector3f targetPos) {
        this.targetPos = targetPos;
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

    public boolean hasPath() {
        return hasPath;
    }

    public boolean pathPending() {
        return pathPending;
    }

}
