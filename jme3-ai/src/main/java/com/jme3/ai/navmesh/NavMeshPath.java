package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Vector3f;

/**
 * 
 * @author capdevon
 */
public class NavMeshPath {
    
    private Waypoint nextWaypoint;
    private final Waypoint start = new Waypoint();
    private final Waypoint end = new Waypoint();
    private final List<Waypoint> waypointList = new ArrayList<>();

    /**
     * Sets up a new path from StartPoint to EndPoint. It adds the StartPoint as
     * the first waypoint in the list and waits for further calls to AddWayPoint
     * and EndPath to complete the list
     * 
     * @param startPoint
     * @param startCell
     * @param endPoint
     * @param endCell
     */
    void initialize(Vector3f startPoint, Cell startCell, Vector3f endPoint, Cell endCell) {

        waypointList.clear();

        start.setPosition(startPoint);
        start.setCell(startCell);
        
        // setup the waypoint list with our start and end points
        waypointList.add(start);
        
        end.setPosition(endPoint);
        end.setCell(endCell);
    }
    
    /**
     * Caps the end of the waypoint list by adding our final destination point.
     */
    void finishPath() {
        // cap the waypoint path with the last endpoint
        waypointList.add(end);
        nextWaypoint = getFirst();
    }

    /**
     * Adds a new waypoint to the end of the list
     */
    public void addWaypoint(Vector3f point, Cell cell) {
        Waypoint newPoint = new Waypoint();
        newPoint.setPosition(point);
        newPoint.setCell(cell);
        waypointList.add(newPoint);
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
    
    public void clear() {
        waypointList.clear();
        nextWaypoint = null;
    }

    public boolean isAtGoalWaypoint() {
        return nextWaypoint == getLast();
    }

    public Waypoint getNextWaypoint() {
        return nextWaypoint;
    }

    public void goToNextWaypoint() {
        int from = waypointList.indexOf(nextWaypoint);
        nextWaypoint = waypointList.get(from + 1);
    }

}
