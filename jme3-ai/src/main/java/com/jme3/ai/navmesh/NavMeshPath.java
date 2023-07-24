package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Vector3f;

/**
 * A path as calculated by the navigation system.
 * 
 * @author capdevon
 */
public class NavMeshPath {
    
    // Status of the path.
    private NavMeshPathStatus status;
    // Corner points of the path.
    private final List<Waypoint> waypointList = new ArrayList<>();
    private Waypoint nextWaypoint;

    /**
     * Sets up a new path from StartPoint to EndPoint. It adds the StartPoint as the
     * first waypoint in the list and waits for further calls to AddWayPoint and
     * EndPath to complete the list
     * 
     * @param startPoint
     * @param startCell
     */
    protected void startPath(Vector3f startPoint, Cell startCell) {
        Waypoint start = new Waypoint(startPoint, startCell);
        // setup the waypoint list with our start and end points
        waypointList.clear();
        waypointList.add(start);
    }

    /**
     * Caps the end of the waypoint list by adding our final destination point.
     * 
     * @param endPoint
     * @param endCell
     */
    protected void endPath(Vector3f endPoint, Cell endCell) {
        Waypoint end = new Waypoint(endPoint, endCell);
        // cap the waypoint path with the last endpoint
        waypointList.add(end);
        nextWaypoint = getFirst();
    }

    protected Waypoint getFirst() {
        return waypointList.get(0);
    }

    protected Waypoint getLast() {
        return waypointList.get(waypointList.size() - 1);
    }
    
    /**
     * Adds a new waypoint to the end of the list
     */
    public void addWaypoint(Vector3f point, Cell cell) {
        Waypoint waypoint = new Waypoint(point, cell);
        waypointList.add(waypoint);
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
    
    /**
     * @return Status of the path. (Read Only)
     */
    public NavMeshPathStatus getStatus() {
        return status;
    }
    
    protected void setStatus(NavMeshPathStatus status) {
        this.status = status;
    }

}
