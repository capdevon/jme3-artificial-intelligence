package com.jme3.ai.navmesh;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Vector3f;

/**
 * Debug information from a pathfinding search.
 * 
 * @author sploreg
 */
public class DebugInfo {
    
    private Vector3f startPos;
    private Vector3f endPos;
    private Cell startCell;
    private Cell endCell;
    
    private List<Vector3f> preOptWaypoints = new ArrayList<>();
    private List<Cell> plannedCells = new ArrayList<>();
    
    private Waypoint failedVisibleWaypoint;
    private Waypoint farthestTestedWaypoint;
    private Cell failedCell;
    private List<Cell> passedCells = new ArrayList<>();

    public void reset() {
        startPos = null;
        endPos = null;
        startCell = null;
        endCell = null;
        preOptWaypoints.clear();
        plannedCells.clear();
        passedCells.clear();
        failedVisibleWaypoint = null;
        farthestTestedWaypoint = null;
        failedCell = null;
    }
    
    public Cell getStartCell() {
        return startCell;
    }

    protected void setStartCell(Cell startCell) {
        this.startCell = startCell;
    }

    public Cell getEndCell() {
        return endCell;
    }

    protected void setEndCell(Cell endCell) {
        this.endCell = endCell;
    }

    public Vector3f getEndPos() {
        return endPos;
    }

    protected void setEndPos(Vector3f endPos) {
        this.endPos = endPos;
    }

    public Vector3f getStartPos() {
        return startPos;
    }

    protected void setStartPos(Vector3f startPos) {
        this.startPos = startPos;
    }

    public Waypoint getFailedVisibleWaypoint() {
        return failedVisibleWaypoint;
    }
    
    protected void setFailedVisibleWaypoint(Waypoint testPoint) {
        this.failedVisibleWaypoint = testPoint;
    }

    public Waypoint getFarthestTestedWaypoint() {
        return farthestTestedWaypoint;
    }
    
    protected void setFarthestTestedWaypoint(Waypoint farthest) {
        this.farthestTestedWaypoint = farthest;
    }
    
    public Cell getFailedCell() {
        return failedCell;
    }
    
    protected void setFailedCell(Cell failed) {
        this.failedCell = failed;
    }

    protected void addPassedCell(Cell passed) {
        this.passedCells.add(passed);
    }

    public List<Cell> getPassedCells() {
        return passedCells;
    }

    protected void addPlannedCell(Cell cell) {
        plannedCells.add(cell);
    }

    public List<Cell> getPlannedCells() {
        return plannedCells;
    }

    protected void addPreOptWaypoints(Vector3f wp) {
        preOptWaypoints.add(wp);
    }

    public List<Vector3f> getPreOptWaypoints() {
        return preOptWaypoints;
    }

    @Override
    public String toString() {
        return "DebugInfo [preOptWaypoints=" + preOptWaypoints 
                + ", plannedCells=" + plannedCells 
                + ", startPos=" + startPos 
                + ", endPos=" + endPos 
                + ", startCell=" + startCell 
                + ", endCell=" + endCell 
                + ", failedVisibleWaypoint=" + failedVisibleWaypoint 
                + ", farthestTestedWaypoint=" + farthestTestedWaypoint 
                + ", failedCell=" + failedCell 
                + ", passedCells=" + passedCells 
                + "]";
    }

}
