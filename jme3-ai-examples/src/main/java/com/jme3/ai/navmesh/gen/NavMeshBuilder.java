package com.jme3.ai.navmesh.gen;

import java.nio.FloatBuffer;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.critterai.nmgen.IntermediateData;
import org.critterai.nmgen.NavmeshGenerator;
import org.critterai.nmgen.TriangleMesh;

import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;

import jme3tools.optimize.GeometryBatchFactory;

/**
 * Generates the navigation mesh using the
 * {@link org.critterai.nmgen.NavmeshGenerator} class.
 */
public class NavMeshBuilder {

    private static final Logger logger = Logger.getLogger(NavMeshBuilder.class.getName());

    private final ExecutorService executor;
    private NavmeshGenerator nmgen;
    private IntermediateData intermediateData;
    private long timeout = 60;
    private TimeUnit timeUnit = TimeUnit.SECONDS;
    
    public NavMeshBuilder() {
        this.executor = Executors.newSingleThreadExecutor();
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

        TriangleMesh triMesh = generateNavMesh(positions, indices);
        if (triMesh != null) {
            logger.log(Level.INFO, "NavMesh generation completed successfully.");
            Mesh navMesh = new Mesh();
            navMesh.setBuffer(VertexBuffer.Type.Position, 3, triMesh.vertices);
            navMesh.setBuffer(VertexBuffer.Type.Index, 3, triMesh.indices);
            navMesh.updateBound();
            navMesh.updateCounts();

            return navMesh;
        }

        logger.log(Level.WARNING, "NavMesh generation failed.");
        return null;
    }
    
    /**
     * Generates a navigation mesh (TriangleMesh) using the provided vertex
     * positions, mesh indices, and intermediate data.
     *
     * @param positions an array of vertex positions
     * @param indices   an array of mesh indices
     * @return the generated TriangleMesh, or null if the generation fails or times out
     */
    private TriangleMesh generateNavMesh(float[] positions, int[] indices) {
        logger.log(Level.INFO, "Starting NavMesh generation task.");
        Future<TriangleMesh> future = executor.submit(() -> nmgen.build(positions, indices, intermediateData));
        
        try {
            return future.get(timeout, timeUnit);
            
        } catch (TimeoutException ex) {
            logger.log(Level.SEVERE, "Task timed out.", ex);
            future.cancel(true);
            
        } catch (InterruptedException | ExecutionException ex) {
            logger.log(Level.SEVERE, "Task execution interrupted.", ex);
        }
        
        return null;
    }
    
    public void shutdown() {
        logger.log(Level.INFO, "Shutting down executor.");
        executor.shutdown();
        try {
            if (!executor.awaitTermination(60, TimeUnit.SECONDS)) {
                logger.log(Level.WARNING, "Executor did not terminate in the specified time. Forcing shutdown.");
                executor.shutdownNow();
            }
        } catch (InterruptedException ex) {
            logger.log(Level.SEVERE, "Shutdown interrupted. Forcing shutdown now.", ex);
            executor.shutdownNow();
        }
    }
    
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
     * Sets the timeout duration for the task execution.
     *
     * @param timeout  the maximum time to wait for the task to complete
     * @param timeUnit the time unit of the timeout parameter
     */
    public void setTimeout(long timeout, TimeUnit timeUnit) {
        this.timeout = timeout;
        this.timeUnit = timeUnit;
    }

}
