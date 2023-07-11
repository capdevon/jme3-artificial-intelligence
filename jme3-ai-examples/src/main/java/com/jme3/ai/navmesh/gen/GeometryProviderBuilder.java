package com.jme3.ai.navmesh.gen;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.terrain.Terrain;

import jme3tools.optimize.GeometryBatchFactory;

/**
 *
 * @author capdevon
 */
public class GeometryProviderBuilder {

    public static final String NAVMESH_IGNORE = "ignoreFromBuild";
    private static final Predicate<Spatial> DefaultFilter = sp -> sp.getUserData(NAVMESH_IGNORE) == null;

    private List<Geometry> geometryList;
    private Mesh mesh;

    /**
     * Provides this Node to the Builder and performs a search through the
     * SceneGraph to gather all Geometries This uses the default filter: If
     * userData "ignoreFromBuild" is set, ignore this spatial
     *
     * @param node The Node to use
     */
    public GeometryProviderBuilder(Node node) {
        this(node, DefaultFilter);
    }

    /**
     * Provides this Node to the Builder and performs a search through the
     * SceneGraph to gather all Geometries
     *
     * @param node The Node to use
     * @param filter A Filter which defines when a Spatial should be gathered
     */
    public GeometryProviderBuilder(Node node, Predicate<Spatial> filter) {
        geometryList = findGeometries(node, new ArrayList<>(), filter);
    }
    
    public Mesh build() {
        if (mesh == null) {
            mesh = new Mesh();
            GeometryBatchFactory.mergeGeometries(geometryList, mesh);
        }
        return mesh;
    }
    
    /**
     * Gathers all geometries in supplied node into supplied List. Uses
     * NavMeshGenerator to merge found Terrain meshes into one geometry prior to
     * adding. Scales and sets translation of merged geometry.
     *
     * @param node
     * @param results
     * @param filter
     * @return
     */
    private List<Geometry> findGeometries(Node node, List<Geometry> results, Predicate<Spatial> filter) {
        for (Spatial spatial : node.getChildren()) {
            if (!filter.test(spatial)) {
                continue;
            }

            if (spatial instanceof Geometry) {
                results.add((Geometry) spatial);

            } else if (spatial instanceof Terrain) {
                Mesh merged = NavMeshBuilder.terrain2mesh((Terrain) spatial);
                Geometry geom = new Geometry("mergedTerrain");
                geom.setMesh(merged);
                results.add(geom);

            } else if (spatial instanceof Node) {
                findGeometries((Node) spatial, results, filter);
            }
        }
        return results;
    }
    
}
