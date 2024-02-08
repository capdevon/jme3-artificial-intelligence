package com.jme3.ai.navmesh;

/**
 * A NavigationNode represents an entry in the NavigationHeap. It provides some
 * simple operators to classify it against other NavigationNodes when the heap
 * is sorted.
 * 
 * Portions Copyright (C) Greg Snook, 2000
 * 
 * @author TR
 * 
 */
class NavNode implements Comparable<NavNode> {

    /**
     * pointer to the cell in question
     */
    protected Cell cell = null;
    /**
     * (g + h) in A* represents the cost of traveling through this cell
     */
    protected float cost = 0.0f;

    NavNode(Cell cell, float cost) {
        this.cell = cell;
        this.cost = cost;
    }

    /**
     * To compare two nodes, we compare the cost or 'f' value, which is the sum of
     * the g and h values defined by A*.
     *
     * @param o the Object to be compared.
     * @return a negative integer, zero, or a positive integer as this object is
     *         less than, equal to, or greater than the specified object.
     */
    @Override
    public int compareTo(NavNode o) {
        if (this.cost < (o.cost)) {
            return -1;
        } else if ((this.cost > (o.cost))) {
            return 1;
        } else {
            return 0;
        }
    }
}
