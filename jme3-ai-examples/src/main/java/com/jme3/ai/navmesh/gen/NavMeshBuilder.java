package com.jme3.ai.navmesh.gen;

import java.nio.FloatBuffer;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.critterai.nmgen.IntermediateData;
import org.critterai.nmgen.NavmeshGenerator;
import org.critterai.nmgen.TriangleMesh;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.terrain.Terrain;

import jme3tools.optimize.GeometryBatchFactory;

/**
 * Generates the navigation mesh using the
 * {@link org.critterai.nmgen.NavmeshGenerator} class.
 */
public class NavMeshBuilder {

    private static final Logger logger = Logger.getLogger(NavMeshBuilder.class.getName());

    private NavmeshGenerator nmgen;
    private IntermediateData intermediateData;
    private int timeout = 60000;
    
    /**
     * The data object to use for storing data related to building the 
     * navigation mesh.
     * 
     * @param {@link org.critterai.nmgen.IntermediateData}
     */
    public void setIntermediateData(IntermediateData intermediateData) {
        this.intermediateData = intermediateData;
    }
    
    /**
     * Sets the timeout duration for the navigation mesh generation process.
     * 
     * @param timeout the length of time in milliseconds before the generation process ends.
     *                If the process exceeds this duration, it will be terminated.
     */
    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }
    
    /**
     * Takes a list of geometries and optimizes them using
     * {@link org.critterai.nmgen.NavmeshGenerator}
     * 
     * @param sources
     * @param settings
     * @return An optimized Triangle mesh to be used for pathfinding.
     */
    public Mesh buildNavMesh(List<Geometry> sources, NavMeshBuildSettings settings) {
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
        
        Mesh mesh = new Mesh();
        GeometryBatchFactory.mergeGeometries(sources, mesh);

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

        TriangleMesh triMesh = generateNavMesh(positions, indices, intermediateData);
        if (triMesh == null) {
            logger.log(Level.WARNING, "TriangleMesh is null.");
            return null;
        }

        Mesh navMesh = new Mesh();
        navMesh.setBuffer(VertexBuffer.Type.Position, 3, triMesh.vertices);
        navMesh.setBuffer(VertexBuffer.Type.Index, 3, triMesh.indices);
        navMesh.updateBound();
        navMesh.updateCounts();

        return navMesh;
    }
    
    /**
     * Takes a Terrain, which can be composed of numerous meshes, and converts them
     * into a single mesh.
     *
     * @param terrain the terrain to be converted
     * @return a single mesh consisting of all meshes of a Terrain
     */
    static Mesh terrain2mesh(Terrain terrain) {
        float[] heightMap = terrain.getHeightMap();
        int length = heightMap.length;
        int size = (int) FastMath.sqrt(heightMap.length);
        float[] vertices = new float[length * 3];
        int[] indices = new int[(size - 1) * (size - 1) * 6];

        Vector3f scale = ((Node) terrain).getWorldScale().clone();
        Vector3f trans = ((Node) terrain).getWorldTranslation().clone();
        trans.x -= terrain.getTerrainSize() / 2f;
        trans.z -= terrain.getTerrainSize() / 2f;
        float offsetX = trans.x * scale.x;
        float offsetZ = trans.z * scale.z;

        // do vertices
        int i = 0;
        for (int z = 0; z < size; z++) {
            for (int x = 0; x < size; x++) {
                vertices[i++] = x + offsetX;
                vertices[i++] = heightMap[z * size + x] * scale.y;
                vertices[i++] = z + offsetZ;
            }
        }

        // do indexes
        i = 0;
        for (int z = 0; z < size - 1; z++) {
            for (int x = 0; x < size - 1; x++) {
                // triangle 1
                indices[i++] = z * size + x;
                indices[i++] = (z + 1) * size + x;
                indices[i++] = (z + 1) * size + x + 1;
                // triangle 2
                indices[i++] = z * size + x;
                indices[i++] = (z + 1) * size + x + 1;
                indices[i++] = z * size + x + 1;
            }
        }

        Mesh mesh2 = new Mesh();
        mesh2.setBuffer(Type.Position, 3, vertices);
        mesh2.setBuffer(Type.Index, 3, indices);
        mesh2.updateBound();
        mesh2.updateCounts();

        return mesh2;
    }

    /**
     * Generates a navigation mesh (TriangleMesh) using the provided vertex
     * positions, mesh indices, and intermediate data.
     */
    private TriangleMesh generateNavMesh(float[] positions, int[] indices, IntermediateData navMeshData) {
        NavMeshBuildRunnable meshBuilder = new NavMeshBuildRunnable(positions, indices, navMeshData);
        try {
            execute(meshBuilder, timeout);
        } catch (NavMeshTimeoutException ex) {
            logger.log(Level.SEVERE, "NavMesh generation timed out.", ex);
        }
        
        return meshBuilder.getTriangleMesh();
    }
    
    /**
     * Executes a given task with a specified timeout.
     */
    private void execute(Runnable task, long timeout) throws NavMeshTimeoutException {
        Thread t = new Thread(task, "TimeoutGuard");
        t.setDaemon(true);
        execute(t, timeout);
    }

    private void execute(Thread task, long timeout) throws NavMeshTimeoutException {
        task.start();
        try {
            task.join(timeout);
            if (task.isAlive()) {
                task.interrupt();
                throw new NavMeshTimeoutException("Task timed out after " + timeout + " milliseconds.");
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            logger.log(Level.SEVERE, "Task execution interrupted.", ex);
            throw new NavMeshTimeoutException("Task was interrupted.");
        }
    }

    /**
     * Runnable for the navigation mesh build process.
     */
    private class NavMeshBuildRunnable implements Runnable {

        private final float[] positions;
        private final int[] indices;
        private final IntermediateData navMeshData;
        private TriangleMesh triangleMesh;

        public NavMeshBuildRunnable(float[] positions, int[] indices, IntermediateData navMeshData) {
            this.positions = positions;
            this.indices = indices;
            this.navMeshData = navMeshData;
        }

        @Override
        public void run() {
            triangleMesh = nmgen.build(positions, indices, navMeshData);
        }

        public TriangleMesh getTriangleMesh() {
            return triangleMesh;
        }
    }

    private class NavMeshTimeoutException extends RuntimeException {
        private static final long serialVersionUID = 1L;

        public NavMeshTimeoutException(String message) {
            super(message);
        }
    }
}
