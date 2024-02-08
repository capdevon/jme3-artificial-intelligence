package com.jme3.ai.navmesh;

import com.jme3.math.Vector3f;
import com.jme3.ai.navmesh.util.MinHeap;

/**
 * A NavigationHeap is a priority-ordered list facilitated by the STL heap
 * functions. This class is also used to hold the current path finding session
 * ID and the desired goal point for NavigationCells to query.
 * 
 * Thanks to Amit J. Patel for detailing the use of STL heaps in this way. 
 * It's much faster than a linked list or multimap approach.
 * 
 * Portions Copyright (C) Greg Snook, 2000
 * 
 * @author TR
 */
class NavHeap {

    private final MinHeap<NavNode> nodes = new MinHeap<>();
    private int sessionID;
    private Vector3f goal;

    protected int getSessionID() {
        return sessionID;
    }

    protected Vector3f getGoal() {
        return goal;
    }

    protected void initialize(int sessionID, Vector3f goal) {
        this.goal = goal;
        this.sessionID = sessionID;
        nodes.clear();
    }

    protected void addCell(Cell pCell) {
        NavNode newNode = new NavNode(pCell, pCell.getTotalCost());
        nodes.add(newNode);
    }

    /**
     * Adjust a cell in the heap to reflect it's updated cost value. NOTE: Cells
     * may only sort up in the heap.
     */
    protected void adjustCell(Cell pCell) {
        NavNode node = findNodeIterator(pCell);

        if (node != nodes.lastElement()) {
            // update the node data
            node.cell = pCell;
            node.cost = pCell.getTotalCost();

            nodes.sort();
        }
    }

    /**
     * @return true if the heap is not empty
     */
    protected boolean isNotEmpty() {
        return !nodes.isEmpty();
    }

    /**
     * Pop the top off the heap and remove the best value for processing.
     */
    protected NavNode getTop() {
        return nodes.deleteMin();
    }

    /**
     * Search the container for a given cell. May be slow, so don't do this
     * unless necessary.
     */
    protected NavNode findNodeIterator(Cell pCell) {
        for (NavNode node : nodes) {
            if (node.cell.equals(pCell)) {
                return node;
            }
        }
        return nodes.lastElement();
    }
}
