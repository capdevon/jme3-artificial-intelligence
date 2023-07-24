package com.jme3.ai.navmesh;

import com.jme3.math.Vector3f;

public class Waypoint {

    // 3D position of waypoint
    protected final Vector3f position;
    // The cell which owns the waypoint
    protected final Cell cell;
    
    /**
     * 
     * @param position
     * @param cell
     */
    public Waypoint(Vector3f position, Cell cell) {
        this.position = position;
        this.cell = cell;
    }

    public Cell getCell() {
        return cell;
    }

    public Vector3f getPosition() {
        return position;
    }

    @Override
    public String toString() {
        return "Waypoint [position=" + position + ", cell=" + cell + "]";
    }
    
}