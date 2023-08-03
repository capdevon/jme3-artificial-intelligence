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

        TriangleMesh triMesh = buildNavMeshData(positions, indices, intermediateData);
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
    public static Mesh terrain2mesh(Terrain terrain) {
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
