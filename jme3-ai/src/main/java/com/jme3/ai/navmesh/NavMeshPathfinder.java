package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import com.jme3.ai.navmesh.Line2D.LineIntersect;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

@Deprecated
public class NavMeshPathfinder {
    
    private static final Logger logger = Logger.getLogger(NavMeshPathfinder.class.getName());

    private NavMesh navMesh;
    private Path path = new Path();
    private float entityRadius;
    private Vector2f currentPos = new Vector2f();
    private Vector3f currentPos3d = new Vector3f();
    private Cell currentCell;
    private Cell goalCell;
    private Waypoint nextWaypoint;
    /**
     * path finding data...
     */
    private volatile int sessionID = 0;
    private volatile NavHeap heap = new NavHeap();

    /**
     * Instantiate a <code>NavMeshPathfinder</code>
     * @param navMesh
     */
    public NavMeshPathfinder(NavMesh navMesh) {
        this.navMesh = navMesh;
    }

    public Vector3f getPosition() {
        return currentPos3d;
    }

    public void setPosition(Vector3f position) {
        this.currentPos3d.set(position);
        this.currentPos.set(currentPos3d.x, currentPos3d.z);
    }

    public float getEntityRadius() {
        return entityRadius;
    }

    public void setEntityRadius(float entityRadius) {
        this.entityRadius = entityRadius;
    }

    /**
     * Warp into the scene on the navmesh, finding the nearest cell.
     * The currentCell and position3d are updated and this new
     * position is returned. This updates the state of the path finder!
     * 
     * @return the new position in the nearest cell
     */
    public Vector3f warp(Vector3f newPos) {
        Vector3f newPos2d = new Vector3f(newPos.x, newPos.y, newPos.z);
        currentCell = navMesh.findClosestCell(newPos2d);
        currentPos3d.set(navMesh.snapPointToCell(currentCell, newPos2d));
        currentPos3d.setY(newPos.getY());
        currentPos.set(currentPos3d.getX(), currentPos3d.getZ());
        return currentPos3d;
    }

    /**
     * Get the nearest cell to the supplied position
     * and place the returned position in that cell.
     * 
     * @param position to place in the nearest cell
     * @return the position in the cell
     */
    public Vector3f warpInside(Vector3f position) {
        Vector3f newPos2d = new Vector3f(position.x, position.y, position.z);
        Cell cell = navMesh.findClosestCell(newPos2d);
        position.set(navMesh.snapPointToCell(cell, newPos2d));
        return position;
    }

    /**
     * Test if the position is inside a cell of the navmesh.
     * 
     * @return false if it falls outside a cell, not in the navmesh.
     */
    public boolean isInsideNavMesh(Vector3f position) {
        Cell cell = navMesh.findClosestCell(position);
        return cell != null;
    }
    
    /**
     * Generate a new Path from the currentPos3d to the supplied goal 
     * position.
     * setPosition() must be called first for this to work. If the 
     * point is not in the mesh, false is returned. You should use
     * warp() in that case to place the point in the mesh.
     * 
     * @return fail if no path found or start location outside of a cell
     */
    public boolean computePath(Vector3f targetPos) {
        return computePath(targetPos, null);
    }

    /**
     * Generate a new Path from the currentPos3d to the supplied goal 
     * position.
     * setPosition() must be called first for this to work. If the 
     * point is not in the mesh, false is returned. You should use
     * warp() in that case to place the point in the mesh.
     * 
     * @return fail if no path found or start location outside of a cell
     */
    public boolean computePath(Vector3f targetPos, DebugInfo debugInfo) {
        // get the cell that this point is in
        Vector3f m_spos = new Vector3f(currentPos3d.x, currentPos3d.y, currentPos3d.z);
        currentCell = navMesh.findClosestCell(m_spos);
        if (currentCell == null) {
            return false;
        }

        Vector3f m_epos = new Vector3f(targetPos.x, targetPos.y, targetPos.z);
        goalCell = navMesh.findClosestCell(m_epos);
        boolean result = buildNavigationPath(path, currentCell, currentPos3d, goalCell, m_epos, entityRadius, debugInfo);
        if (!result) {
            goalCell = null;
            return false;
        }
        
        nextWaypoint = path.getFirst();
        return true;
    }

    public void clearPath() {
        path.clear();
        goalCell = null;
        nextWaypoint = null;
    }

    public boolean isAtGoalWaypoint() {
        return nextWaypoint == path.getLast();
    }

    public Waypoint getNextWaypoint() {
        return nextWaypoint;
    }

    public void goToNextWaypoint() {
        int from = path.getWaypoints().indexOf(nextWaypoint);
        nextWaypoint = path.getWaypoints().get(from + 1);
        currentCell = nextWaypoint.getCell();
    }

    public Path getPath() {
        return path;
    }

    /**
     * Build a navigation path using the provided points and the A* method
     */
    private boolean buildNavigationPath(Path navPath,
            Cell startCell, Vector3f startPos,
            Cell endCell, Vector3f endPos,
            float entityRadius, DebugInfo debugInfo) {

        // Increment our path finding session ID
        // This Identifies each pathfinding session
        // so we do not need to clear out old data
        // in the cells from previous sessions.
        sessionID++;

        // load our data into the Heap object
        // to prepare it for use.
        heap.initialize(sessionID, startPos);

        // We are doing a reverse search, from EndCell to StartCell.
        // Push our EndCell onto the Heap at the first cell to be processed
        endCell.queryForPath(heap, null, 0.0f);

        // process the heap until empty, or a path is found
        boolean foundPath = false;
        while (heap.isNotEmpty() && !foundPath) {

            // pop the top cell (the open cell with the lowest cost) off the
            // Heap
            NavNode currentNode = heap.getTop();

            // if this cell is our StartCell, we are done
            if (currentNode.cell.equals(startCell)) {
                foundPath = true;
            } else {
                // Process the Cell, Adding it's neighbors to the Heap as needed
                currentNode.cell.processCell(heap);
            }
        }

        Vector2f intersectionPoint = new Vector2f();

        // if we found a path, build a waypoint list
        // out of the cells on the path
        if (!foundPath) {
            return false;
        }

        // Setup the Path object, clearing out any old data
        navPath.initialize(navMesh, startPos, startCell, endPos, endCell);

        Vector3f lastWayPoint = startPos;

        // Step through each cell linked by our A* algorithm
        // from StartCell to EndCell
        Cell currCell = startCell;
        while (currCell != null && currCell != endCell) {

            if (debugInfo != null) {
                debugInfo.addPlannedCell(currCell);
            }

            // add the link point of the cell as a way point (the exit
            // wall's center)
            int linkWall = currCell.getArrivalWall();
            Vector3f newWayPoint = currCell.getWallMidpoint(linkWall).clone();

            Line2D wall = currCell.getWall(linkWall);
            float length = wall.length();
            float distBlend = entityRadius / length;

            Line2D lineToGoal = new Line2D(
                    new Vector2f(lastWayPoint.x, lastWayPoint.z),
                    new Vector2f(endPos.x, endPos.z));
            
            LineIntersect result = lineToGoal.intersect(wall, intersectionPoint);
            switch (result) {
                case SegmentsIntersect:
                    float d1 = wall.getPointA().distance(intersectionPoint);
                    float d2 = wall.getPointB().distance(intersectionPoint);
                    if (d1 > entityRadius && d2 > entityRadius) {
                        // we can fit through the wall if we go
                        // directly to the goal.
                        newWayPoint = new Vector3f(intersectionPoint.x, 0, intersectionPoint.y);
                    } else {
                        // cannot fit directly.
                        // try to find point where we can
                        if (d1 < d2) {
                            intersectionPoint.interpolateLocal(wall.getPointA(), wall.getPointB(), distBlend);
                            newWayPoint = new Vector3f(intersectionPoint.x, 0, intersectionPoint.y);
                        } else {
                            intersectionPoint.interpolateLocal(wall.getPointB(), wall.getPointA(), distBlend);
                            newWayPoint = new Vector3f(intersectionPoint.x, 0, intersectionPoint.y);
                        }
                    }
                    currCell.computeHeightOnCell(newWayPoint);
                    break;
                case LinesIntersect:
                case ABisectsB:
                case BBisectsA:
                    Vector2f lastPt2d = new Vector2f(lastWayPoint.x, lastWayPoint.z);
                    Vector2f endPos2d = new Vector2f(endPos.x, endPos.z);

                    Vector2f normalEnd = endPos2d.subtract(lastPt2d).normalizeLocal();
                    Vector2f normalA = wall.getPointA().subtract(lastPt2d).normalizeLocal();
                    Vector2f normalB = wall.getPointB().subtract(lastPt2d).normalizeLocal();
                    if (normalA.dot(normalEnd) < normalB.dot(normalEnd)) {
                        // choose point b
                        intersectionPoint.interpolateLocal(wall.getPointB(), wall.getPointA(), distBlend);
                        newWayPoint = new Vector3f(intersectionPoint.x, 0, intersectionPoint.y);
                    } else {
                        // choose point a
                        intersectionPoint.interpolateLocal(wall.getPointA(), wall.getPointB(), distBlend);
                        newWayPoint = new Vector3f(intersectionPoint.x, 0, intersectionPoint.y);
                    }
                    currCell.computeHeightOnCell(newWayPoint);

                    break;
                case CoLinear:
                case Parallel:
                    logger.fine("## colinear or parallel");
                    break;
            }

            if (debugInfo != null) {
                debugInfo.addPreOptWaypoints(newWayPoint.clone());
            }
            //newWayPoint = snapPointToCell(currentCell, newWayPoint);
            lastWayPoint = newWayPoint.clone();

            navPath.addWaypoint(newWayPoint, currCell);

            // get the next cell
            currCell = currCell.getLink(linkWall);
        }

        // cap the end of the path.
        navPath.finishPath();

        // remove optimization so it can be done as the actor moves
        // further: optimize the path
        List<Waypoint> newPath = new ArrayList<>();
        Waypoint curWayPoint = navPath.getFirst();
        newPath.add(curWayPoint);
        while (curWayPoint != navPath.getLast()) {
            curWayPoint = navPath.getFurthestVisibleWayPoint(curWayPoint);
            newPath.add(curWayPoint);
        }

        navPath.initialize(navMesh, startPos, startCell, endPos, endCell);
        for (Waypoint newWayPoint : newPath) {
            navPath.addWaypoint(newWayPoint.getPosition(), newWayPoint.getCell());
        }
        navPath.finishPath();

        return true;
    }
    
}
