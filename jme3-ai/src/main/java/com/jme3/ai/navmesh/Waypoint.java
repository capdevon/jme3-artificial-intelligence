package com.jme3.ai.navmesh;

import com.jme3.math.Vector3f;

public class Waypoint {

    // 3D position of waypoint
    protected Vector3f position;
    // The cell which owns the waypoint
    protected Cell cell;

    public Cell getCell() {
        return cell;
    }

    public void setCell(Cell cell) {
        this.cell = cell;
    }

    public Vector3f getPosition() {
        return position;
    }

    public void setPosition(Vector3f position) {
        this.position = position;
    }

    @Override
    public String toString() {
        return "Waypoint[position=" + position.x + ", " + position.z + " cell:" + cell + "]";
    }
    
}