package com.jme3.ai.navmesh;

import com.jme3.math.Vector3f;

/**
 * Contains data describing a triangulation of a navmesh.
 * 
 * @author capdevon
 */
public class NavMeshTriangulation {

    // Triangle indices for the navmesh triangulation.
    private final int[] indices;
    // Vertices for the navmesh triangulation.
    private final Vector3f[] vertices;

    /**
     * 
     * @param indices
     * @param vertices
     */
    public NavMeshTriangulation(int[] indices, Vector3f[] vertices) {
        this.indices = indices;
        this.vertices = vertices;
    }

    public int[] getIndices() {
        return indices;
    }

    public Vector3f[] getVertices() {
        return vertices;
    }

}
