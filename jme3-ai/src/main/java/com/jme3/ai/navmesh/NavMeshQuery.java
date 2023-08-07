package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.ai.navmesh.Cell.ClassifyResult;
import com.jme3.ai.navmesh.Line2D.LineIntersect;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

/**
 * 
 * @author capdevon
 */
public class NavMeshQuery {
    
    private NavMesh navMesh;
    private StraightPathOptions straightPathOptions = StraightPathOptions.AreaCrossings;
    private float entityRadius = 1f;
    
    /**
     * path finding data...
     */
    private volatile int sessionID = 0;
    private volatile Heap heap = new Heap();
    
    /**
     * Instantiate a <code>NavMeshQuery</code>
     * @param navMesh
     */
    public NavMeshQuery(NavMesh navMesh) {
        this.navMesh = navMesh;
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
     * Calculate a path between two points and store the resulting path.
     * 
     * @param startPosition The initial position of the path requested.
     * @param endPosition   The final position of the path requested.
     * @param path          The resulting path.
     * @return True if a complete path is found. False otherwise.
     */
    public boolean calculatePath(Vector3f startPos, Vector3f targetPos, NavMeshPath path) {
        return calculatePath(startPos, targetPos, path, null);
    }

    /**
     * Calculate a path between two points and store the resulting path.
     * 
     * @param startPosition The initial position of the path requested.
     * @param endPosition   The final position of the path requested.
     * @param path          The resulting path.
     * @param debugInfo     The resulting debugInfo.
     * @return True if a complete path is found. False otherwise.
     */
    public boolean calculatePath(Vector3f startPos, Vector3f endPos, NavMeshPath path, DebugInfo debugInfo) {
        
        Vector3f spos = new Vector3f(startPos);
        Cell startCell = navMesh.findClosestCell(spos);

        Vector3f epos = new Vector3f(endPos);
        Cell endCell = navMesh.findClosestCell(epos);
        navMesh.snapPointToCell(endCell, epos);
        
        return buildNavigationPath(path, startCell, spos, endCell, epos, debugInfo);
    }

    /**
     * Build a navigation path using the provided points and the A* method
     */
    private boolean buildNavigationPath(NavMeshPath navPath,
            Cell startCell, Vector3f startPos,
            Cell endCell, Vector3f endPos, DebugInfo debugInfo) {
        
        // clear the navigation path
        navPath.clear();
        
        if (debugInfo != null) {
            debugInfo.reset();
            debugInfo.setStartPos(startPos);
            debugInfo.setStartCell(startCell);
            debugInfo.setEndPos(endPos);
            debugInfo.setEndCell(endCell);
        }

        boolean foundPath = processHeap(startCell, startPos, endCell, endPos);
        if (!foundPath) {
            navPath.setStatus(NavMeshPathStatus.PathInvalid);
            return false;
        }
        
        // if we found a path, build a waypoint list
        // out of the cells on the path
        computePath(startCell, startPos, endCell, endPos, navPath, debugInfo);
        
        if (straightPathOptions == StraightPathOptions.AreaCrossings) {
            optimizePath(startCell, startPos, endCell, endPos, navPath, debugInfo);
        }
        
        navPath.setStatus(NavMeshPathStatus.PathComplete);
        return true;
    }

    private void optimizePath(Cell startCell, Vector3f startPos, Cell endCell, Vector3f endPos, 
            NavMeshPath navPath, DebugInfo debugInfo) {

        // further: optimize the path
        List<Waypoint> newPath = new ArrayList<>();
        Waypoint curWayPoint = navPath.getFirst();
        newPath.add(curWayPoint);

        while (curWayPoint != navPath.getLast()) {
            curWayPoint = getFarthestVisibleWayPoint(navPath, curWayPoint, debugInfo);
            newPath.add(curWayPoint);
        }

        // Setup the Path object, clearing out any old data
        navPath.startPath(startPos, startCell);

        for (Waypoint newWayPoint : newPath) {
            navPath.addWaypoint(newWayPoint.getPosition(), newWayPoint.getCell());
        }
        navPath.endPath(endPos, endCell);
    }

    private void computePath(Cell startCell, Vector3f startPos, Cell endCell, Vector3f endPos, 
            NavMeshPath navPath, DebugInfo debugInfo) {
        
        // Setup the Path object, clearing out any old data
        navPath.startPath(startPos, startCell);

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
            //newWayPoint = snapPointToCell(currCell, newWayPoint);
            lastWayPoint = newWayPoint.clone();

            navPath.addWaypoint(newWayPoint, currCell);

            // get the next cell
            currCell = currCell.getLink(linkWall);
        }

        // cap the end of the path.
        navPath.endPath(endPos, endCell);
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
    
    /**
     * Find the farthest visible waypoint from the VantagePoint provided. This
     * is used to smooth out irregular paths.
     */
    private Waypoint getFarthestVisibleWayPoint(NavMeshPath navPath, Waypoint vantagePoint, DebugInfo debugInfo) {
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
            if (!isInLineOfSight(vantagePoint.cell, vantagePoint.position,
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
            if (!isInLineOfSight(vantagePoint.cell, vantagePoint.position,
                    testPoint.position, debugInfo))
                return visibleWaypoint;
        }
        return testPoint;
    }
    
    /**
     * Test to see if two points on the mesh can view each other
     */
    private boolean isInLineOfSight(Cell startCell, Vector3f startPos, Vector3f endPos, DebugInfo debugInfo) {

        Vector2f pointA = new Vector2f(startPos.x, startPos.z);
        Vector2f pointB = new Vector2f(endPos.x, endPos.z);
        Line2D motionPath = new Line2D(pointA, pointB);

        Cell testCell = startCell;
        ClassifyResult pathResult = testCell.classifyPathToCell(motionPath);
        ClassifyResult prevResult = pathResult;

        while (pathResult.result == Cell.PathResult.ExitingCell) {
            if (pathResult.cell == null) {
                // hit a wall, so the point is not visible
                if (debugInfo != null) {
                    debugInfo.setFailedCell(prevResult.cell);
                }
                return false;
            }
            if (debugInfo != null) {
                debugInfo.addPassedCell(prevResult.cell);
            }
            prevResult = pathResult;
            pathResult = pathResult.cell.classifyPathToCell(motionPath);
        }
        
        if (debugInfo != null) {
            debugInfo.setEndCell(prevResult.cell);
        }
        
        // This is messing up the result, I think because of shared borders
        return (pathResult.result == Cell.PathResult.EndingCell 
                || pathResult.result == Cell.PathResult.ExitingCell);
    }
    
    /**
     * Do not use!
     */
    private Waypoint getFarthestVisibleWayPointOptimized(NavMeshPath navPath, Waypoint vantagePoint) {
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
    
    private boolean lineIntersectsTriangle(Vector3f position, Vector3f nextPosition, Vector3f[] nextCell) {
        
        if (isLeft(position, nextPosition, nextCell[0]) ^ isLeft(position, nextPosition, nextCell[1]))
            return true;
        if (isLeft(position, nextPosition, nextCell[1]) ^ isLeft(position, nextPosition, nextCell[2]))
            return true;
        if (isLeft(position, nextPosition, nextCell[2]) ^ isLeft(position, nextPosition, nextCell[0]))
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
