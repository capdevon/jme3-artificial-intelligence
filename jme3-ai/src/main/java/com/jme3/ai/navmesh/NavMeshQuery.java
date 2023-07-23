package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.ai.navmesh.Line2D.LineIntersect;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

/**
 * 
 * @author capdevon
 */
public class NavMeshQuery {
    
    private NavMesh navMesh;
    private NavMeshPath path = new NavMeshPath();
    private StraightPathOptions straightPathOptions = StraightPathOptions.AreaCrossings;
    private float entityRadius = 1f;
    
    /**
     * path finding data...
     */
    private volatile int sessionID = 0;
    private volatile Heap heap = new Heap();
    
    public NavMeshQuery(NavMesh navMesh) {
        this.navMesh = navMesh;
    }

    public boolean computePath(Vector3f startPos, Vector3f targetPos) {
        return computePath(startPos, targetPos, null);
    }

    public boolean computePath(Vector3f startPos, Vector3f endPos, DebugInfo debugInfo) {
        
        Vector3f m_spos = new Vector3f(startPos.x, startPos.y, startPos.z);
        Cell startCell = navMesh.findClosestCell(m_spos);

        Vector3f m_epos = new Vector3f(endPos.x, endPos.y, endPos.z);
        Cell endCell = navMesh.findClosestCell(m_epos);
        navMesh.snapPointToCell(endCell, m_epos);
        
        boolean result = buildNavigationPath(path, startCell, startPos, endCell, m_epos, debugInfo);
        if (!result) {
            return false;
        }
        
        return true;
    }

    public NavMeshPath getPath() {
        return path;
    }
    
    public float getEntityRadius() {
        return entityRadius;
    }

    public void setEntityRadius(float entityRadius) {
        this.entityRadius = entityRadius;
    }
    
    public StraightPathOptions getStraightPathOptions() {
        return straightPathOptions;
    }

    public void setStraightPathOptions(StraightPathOptions straightPathOptions) {
        this.straightPathOptions = straightPathOptions;
    }

    /**
     * Build a navigation path using the provided points and the A* method
     */
    private boolean buildNavigationPath(NavMeshPath navPath,
            Cell startCell, Vector3f startPos,
            Cell endCell, Vector3f endPos, DebugInfo debugInfo) {

        boolean foundPath = processHeap(startCell, startPos, endCell, endPos);

        // if we found a path, build a waypoint list
        // out of the cells on the path
        if (!foundPath) {
            return false;
        }

        // Setup the Path object, clearing out any old data
        navPath.initialize(startPos, startCell, endPos, endCell);

        Vector3f lastWayPoint = startPos;
        Vector2f intersectionPoint = new Vector2f();

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
        
        if (straightPathOptions == StraightPathOptions.AreaCrossings) {
            // further: optimize the path
            List<Waypoint> newPath = new ArrayList<>();
            Waypoint curWayPoint = navPath.getFirst();
            newPath.add(curWayPoint);
            while (curWayPoint != navPath.getLast()) {
                curWayPoint = getFurthestVisibleWayPoint(navPath, curWayPoint);
                newPath.add(curWayPoint);
            }

            navPath.initialize(startPos, startCell, endPos, endCell);
            for (Waypoint newWayPoint : newPath) {
                navPath.addWaypoint(newWayPoint.getPosition(), newWayPoint.getCell());
            }
            navPath.finishPath();
        }

        return true;
    }
    
    private boolean processHeap(Cell startCell, Vector3f startPos,
            Cell endCell, Vector3f endPos) {

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
            // pop the top cell (the open cell with the lowest cost) off the Heap
            Node currentNode = heap.getTop();
            // if this cell is our StartCell, we are done
            if (currentNode.cell.equals(startCell)) {
                foundPath = true;
            } else {
                // Process the Cell, Adding it's neighbors to the Heap as needed
                currentNode.cell.processCell(heap);
            }
        }

        return foundPath;
    }
    
    private Waypoint getFurthestVisibleWayPoint(NavMeshPath navPath, Waypoint vantagePoint) {
        return getFurthestVisibleWayPoint(navPath, vantagePoint, null);
    }
    
    /**
     * Find the farthest visible waypoint from the VantagePoint provided. This
     * is used to smooth out irregular paths.
     * 
     * @param vantagePoint
     * @return
     */
    private Waypoint getFurthestVisibleWayPoint(NavMeshPath navPath, Waypoint vantagePoint, DebugInfo debugInfo) {
        // see if we are already talking about the last waypoint
        if (vantagePoint == navPath.getLast()) {
            return vantagePoint;
        }

        int i = navPath.getWaypoints().indexOf(vantagePoint);
        if (i < 0) {
            // The given waypoint does not belong to this path.
            return vantagePoint;
        }

        Waypoint testPoint = navPath.getWaypoints().get(++i);
        if (testPoint == navPath.getLast()) {
            return testPoint;
        }
        
        if (debugInfo != null)
            debugInfo.setFarthestTestedWaypoint(testPoint);
        
        Waypoint visibleWaypoint = testPoint;
        while (testPoint != navPath.getLast()) {
            if (!navMesh.isInLineOfSight(vantagePoint.cell, vantagePoint.position,
                    testPoint.position, debugInfo)) {
                if (debugInfo != null)
                    debugInfo.setFailedVisibleWaypoint(testPoint);
                return visibleWaypoint;
            }
            visibleWaypoint = testPoint;
            testPoint = navPath.getWaypoints().get(++i);
            if (debugInfo != null)
                debugInfo.setFarthestTestedWaypoint(testPoint);
        }
        // if it is the last point, and not visible, return the previous point
        if (testPoint == navPath.getLast()) {
            if (!navMesh.isInLineOfSight(vantagePoint.cell, vantagePoint.position,
                    testPoint.position, debugInfo))
                return visibleWaypoint;
        }
        return testPoint;
    }
    
    /**
     * Do not use!
     * 
     * @param vantagePoint
     * @return
     */
    private Waypoint getFurthestVisibleWayPointOptimized(NavMeshPath navPath, Waypoint vantagePoint) {
        // see if we are already talking about the last waypoint
        if (vantagePoint == navPath.getLast()) {
            return vantagePoint;
        }

        int i = navPath.getWaypoints().indexOf(vantagePoint);
        int startI = i;
        if (i < 0) {
            // The given waypoint does not belong to this path.
            return vantagePoint;
        }

        Waypoint testPoint = navPath.getWaypoints().get(++i);
        if (testPoint == navPath.getLast()) {
            System.out.println(" WAY IND was last");
            return testPoint;
        }

        Waypoint visibleWaypoint = testPoint;
        int c = 0;
        while (testPoint != navPath.getLast()) {
            if (!isInLineOfSight(vantagePoint.position, testPoint.getCell(), testPoint.position)) {
                if (c > 1)
                    System.out.println(" WAY IND jump was:" + (i - 1 - startI) + ", new idx= " + (i - 1));
                else if (c == 0)
                    System.out.println(" WAY IND jump was 0!");
                return visibleWaypoint;
            }
            visibleWaypoint = testPoint;
            testPoint = navPath.getWaypoints().get(++i);
            c++;
        }
        return testPoint;
    }
    
    private boolean isInLineOfSight(Vector3f position, Cell nextCell, Vector3f nextPosition) {
        return lineIntersectsTriangle(position, nextPosition, nextCell.getTriangle());
    }
    
    private boolean lineIntersectsTriangle(Vector3f position, Vector3f nextPosition, Vector3f[] cell) {
        
        if (isLeft(position, nextPosition, cell[0]) ^ isLeft(position, nextPosition, cell[1]))
            return true;
        if (isLeft(position, nextPosition, cell[1]) ^ isLeft(position, nextPosition, cell[2]))
            return true;
        if (isLeft(position, nextPosition, cell[2]) ^ isLeft(position, nextPosition, cell[0]))
            return true;
        
        return false;
    }
    
    /**
     * Check if C is left of the line AB
     */
    private boolean isLeft(Vector3f a, Vector3f b, Vector3f c) {
        return ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)) > 0;
    }

}
