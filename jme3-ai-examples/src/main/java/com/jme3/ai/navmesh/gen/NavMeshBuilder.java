package com.jme3.ai.navmesh.gen;

import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.critterai.nmgen.IntermediateData;
import org.critterai.nmgen.NavmeshGenerator;
import org.critterai.nmgen.TriangleMesh;

import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;

/**
 * https://github.com/jMonkeyEngine/sdk/blob/master/jme3-navmesh-gen/src/com/jme3/gde/nmgen/NavMeshGenerator.java
 * Generates the navigation mesh using the org.critterai.nmgen.NavmeshGenerator
 * class.
 */
public class NavMeshBuilder {

    private static final Logger logger = Logger.getLogger(NavMeshBuilder.class.getName());

    private org.critterai.nmgen.NavmeshGenerator nmgen;
    private IntermediateData intermediateData;
    private int timeout = 60000;
    
    /**
     * The data object to use for storing data related to building the navigation mesh.
     * @param intermediateData
     */
    public void setIntermediateData(IntermediateData intermediateData) {
        this.intermediateData = intermediateData;
    }
    
    /**
     * Takes a normal mesh and optimizes it using CritterAi NavMeshGenerator.
     *
     * @param mesh - The mesh to be optimized for pathfinding.
     * @param settings
     * @return An optimized Triangle mesh to be used for pathfinding.
     */
    public Mesh buildNavMesh(Mesh mesh, NavMeshBuildSettings settings) {
        nmgen = new NavmeshGenerator(
                settings.cellSize, 
                settings.cellHeight, 
                settings.minTraversableHeight,
                settings.maxTraversableStep, 
                settings.maxTraversableSlope,
                settings.clipLedges, 
                settings.traversableAreaBorderSize,
                settings.smoothingThreshold, 
                settings.useConservativeExpansion,
                settings.minUnconnectedRegionSize, 
                settings.mergeRegionSize,
                settings.maxEdgeLength, 
                settings.edgeMaxDeviation, 
                settings.maxVertsPerPoly,
                settings.contourSampleDistance, 
                settings.contourMaxDeviation);

        FloatBuffer pb = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        IndexBuffer ib = mesh.getIndexBuffer();
        // copy positions to float array
        float[] positions = new float[pb.capacity()];
        pb.clear();
        pb.get(positions);
        // generate int array of indices
        int[] indices = new int[ib.size()];
        for (int i = 0; i < indices.length; i++) {
            indices[i] = ib.get(i);
        }

        TriangleMesh triMesh = buildNavMeshData(positions, indices, intermediateData);
        if (triMesh == null) {
            logger.log(Level.WARNING, "TriangleMesh is null.");
            return null;
        }

        int[] indices2 = triMesh.indices;
        float[] positions2 = triMesh.vertices;

        Mesh mesh2 = new Mesh();
        mesh2.setBuffer(VertexBuffer.Type.Position, 3, positions2);
        mesh2.setBuffer(VertexBuffer.Type.Index, 3, indices2);
        mesh2.updateBound();
        mesh2.updateCounts();

        return mesh2;
    }

    private TriangleMesh buildNavMeshData(float[] positions, int[] indices, IntermediateData intermediateData) {
        MeshBuildRunnable runnable = new MeshBuildRunnable(positions, indices, intermediateData);
        try {
            execute(runnable, timeout);
        } catch (TimeoutException ex) {
            logger.log(Level.SEVERE, "NavMesh Generation timed out.", ex);
        }
        
        return runnable.getTriMesh();
    }
    
    private void execute(Runnable task, long timeout) throws TimeoutException {
        Thread t = new Thread(task, "Timeout guard");
        t.setDaemon(true);
        execute(t, timeout);
    }

    private void execute(Thread task, long timeout) throws TimeoutException {
        task.start();
        try {
            task.join(timeout);
        } catch (InterruptedException ex) {
            logger.log(Level.SEVERE, "NavMesh Generation interrupted.", ex);
        }
        if (task.isAlive()) {
            task.interrupt();
            throw new TimeoutException();
        }
    }

    /**
     * @return the time in milliseconds before the generation process fails
     */
    public int getTimeout() {
        return timeout;
    }

    /**
     * @param timeout length of time in milliseconds before the generation process ends
     */
    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }

    /**
     * the runnable for the build process
     */
    private class MeshBuildRunnable implements Runnable {

        private final float[] positions;
        private final int[] indices;
        private final IntermediateData intermediateData;
        private TriangleMesh triMesh;

        public MeshBuildRunnable(float[] positions, int[] indices, IntermediateData intermediateData) {
            this.positions = positions;
            this.indices = indices;
            this.intermediateData = intermediateData;
        }

        @Override
        public void run() {
            triMesh = nmgen.build(positions, indices, intermediateData);
        }

        public TriangleMesh getTriMesh() {
            return triMesh;
        }
    }

    private class TimeoutException extends Exception {

        private static final long serialVersionUID = 1L;

        public TimeoutException() {}
    }
}
