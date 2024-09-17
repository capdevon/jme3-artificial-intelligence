package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.ai.navmesh.Cell.ClassifyResult;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

/**
 * NavigationPath is a collection of waypoints that define a movement path for
 * an Actor. This object is owned by an Actor and filled by
 * NavigationMesh::BuildNavigationPath().
 * 
 * Portions Copyright (C) Greg Snook, 2000
 * 
 * @author TR
 */
@Deprecated
public class Path {

    private NavMesh navMesh;
    private Waypoint start;
    private Waypoint end;
    private List<Waypoint> waypointList = new ArrayList<>();

    /**
     * Sets up a new path from StartPoint to EndPoint. It adds the StartPoint as
     * the first waypoint in the list and waits for further calls to AddWayPoint
     * and EndPath to complete the list
     * 
     * @param navMesh
     * @param startPoint
     * @param startCell
     * @param endPoint
     * @param endCell
     */
    public void initialize(NavMesh navMesh,
                           Vector3f startPoint, Cell startCell,
                           Vector3f endPoint, Cell endCell) {

        this.navMesh = navMesh;
        this.start = new Waypoint(startPoint, startCell);
        this.end = new Waypoint(endPoint, endCell);

        // setup the waypoint list with our start and end points
        waypointList.clear();
        waypointList.add(start);
    }

    /**
     * Caps the end of the waypoint list by adding our final destination point.
     */
    void finishPath() {
        // cap the waypoint path with the last endpoint
        waypointList.add(end);
    }
    
    public void clear() {
        waypointList.clear();
    }

    /**
     * Adds a new waypoint to the end of the list
     * 
     * @param point
     * @param cell
     */
    public void addWaypoint(Vector3f point, Cell cell) {
        Waypoint newPoint = new Waypoint(point, cell);
        waypointList.add(newPoint);
    }

    public Waypoint getStart() {
        return start;
    }

    public Waypoint getEnd() {
        return end;
    }

    public Waypoint getFirst() {
        return waypointList.get(0);
    }

    public Waypoint getLast() {
        return waypointList.get(waypointList.size() - 1);
    }

    public List<Waypoint> getWaypoints() {
        return waypointList;
    }
        
    protected Waypoint getFurthestVisibleWayPoint(Waypoint vantagePoint) {
        return getFurthestVisibleWayPoint(vantagePoint, null);
    }
    
    /**
     * Find the farthest visible waypoint from the VantagePoint provided. This
     * is used to smooth out irregular paths.
     * 
     * @param vantagePoint
     * @param debugInfo
     * @return
     */
    protected Waypoint getFurthestVisibleWayPoint(Waypoint vantagePoint, DebugInfo debugInfo) {
        // see if we are already talking about the last waypoint
        if (vantagePoint == getLast()) {
            return vantagePoint;
        }

        int i = waypointList.indexOf(vantagePoint);
        if (i < 0) {
            // The given waypoint does not belong to this path.
            return vantagePoint;
        }

        Waypoint testPoint = waypointList.get(++i);
        if (testPoint == getLast()) {
            return testPoint;
        }
        
        if (debugInfo != null)
            debugInfo.setFarthestTestedWaypoint(testPoint);
        
        Waypoint visibleWaypoint = testPoint;
        while (testPoint != getLast()) {
            if (!isInLineOfSight(vantagePoint.cell, vantagePoint.position,
                    testPoint.position, debugInfo)) {
                if (debugInfo != null)
                    debugInfo.setFailedVisibleWaypoint(testPoint);
                return visibleWaypoint;
            }
            visibleWaypoint = testPoint;
            testPoint = waypointList.get(++i);
            if (debugInfo != null)
                debugInfo.setFarthestTestedWaypoint(testPoint);
        }
        // if it is the last point, and not visible, return the previous point
        if (testPoint == getLast()) {
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
        Line2D motionPath = new Line2D(new Vector2f(startPos.x, startPos.z), new Vector2f(endPos.x, endPos.z));

        Cell testCell = startCell;
        ClassifyResult result = testCell.classifyPathToCell(motionPath);
        ClassifyResult prevResult = result;

        while (result.result == Cell.PathResult.ExitingCell) {
            if (result.cell == null) {
                // hit a wall, so the point is not visible
                if (debugInfo != null) {
                    debugInfo.setFailedCell(prevResult.cell);
                }
                return false;
            }
            if (debugInfo != null) {
                debugInfo.addPassedCell(prevResult.cell);
            }
            prevResult = result;
            result = result.cell.classifyPathToCell(motionPath);
        }
        if (debugInfo != null) {
            debugInfo.setEndCell(prevResult.cell);
        }
        // This is messing up the result, I think because of shared borders
        return (result.result == Cell.PathResult.EndingCell || result.result == Cell.PathResult.ExitingCell);
    }
    
    /**
     * Do not use!
     * 
     * @param vantagePoint
     * @return
     */
    protected Waypoint getFurthestVisibleWayPointOptimized(Waypoint vantagePoint) {
        // see if we are already talking about the last waypoint
        if (vantagePoint == getLast()) {
            return vantagePoint;
        }

        int i = waypointList.indexOf(vantagePoint);
        int startI = i;
        if (i < 0) {
            // The given waypoint does not belong to this path.
            return vantagePoint;
        }

        Waypoint testPoint = waypointList.get(++i);
        if (testPoint == getLast()) {
            System.out.println(" WAY IND was last");
            return testPoint;
        }

        Waypoint visibleWaypoint = testPoint;
        int c = 0;
        while (testPoint != getLast()) {
            if (!isInLineOfSight(vantagePoint.position, testPoint.getCell(), testPoint.position)) {
                if (c > 1)
                    System.out.println(" WAY IND jump was:" + (i - 1 - startI) + ", new idx= " + (i - 1));
                else if (c == 0)
                    System.out.println(" WAY IND jump was 0!");
                return visibleWaypoint;
            }
            visibleWaypoint = testPoint;
            testPoint = waypointList.get(++i);
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
