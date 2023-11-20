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
    void startPath(Vector3f startPoint, Cell startCell) {
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
    void endPath(Vector3f endPoint, Cell endCell) {
        Waypoint end = new Waypoint(endPoint, endCell);
        // cap the waypoint path with the last endpoint
        waypointList.add(end);
        nextWaypoint = getFirst();
    }
    
    /**
     * Adds a new waypoint to the end of the list
     */
    void addWaypoint(Vector3f point, Cell cell) {
        Waypoint waypoint = new Waypoint(point, cell);
        waypointList.add(waypoint);
    }

    protected Waypoint getFirst() {
        return waypointList.get(0);
    }

    protected Waypoint getLast() {
        return waypointList.get(waypointList.size() - 1);
    }

    public List<Waypoint> getWaypoints() {
        return waypointList;
    }
    
    public void clear() {
        status = NavMeshPathStatus.PathInvalid;
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
        int index = waypointList.indexOf(nextWaypoint);
        nextWaypoint = waypointList.get(index + 1);
        waypointList.remove(index);
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
