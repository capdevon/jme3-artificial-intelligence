package com.jme3.ai.navmesh.gen;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.terrain.Terrain;

/**
 *
 * @author capdevon
 */
public class GeometryProviderBuilder {

    private static final Predicate<Spatial> DefaultFilter = 
            sp -> sp.getUserData(NavMeshUserData.JME_NAVMESH_IGNORE) == null;
    
    private GeometryProviderBuilder() {}

    /**
     * Performs a search in the SceneGraph to collect all geometries of the supplied
     * node. It uses the default filter: If userData "ignoreFromBuild" is set, it
     * ignores this space.
     * 
     * @param node
     * @return
     */
    public static List<Geometry> collectSources(Node node) {
        return collectSources(node, new ArrayList<>(), DefaultFilter);
    }

    /**
     * Performs a search in the SceneGraph to collect all geometries of the supplied
     * node.
     * 
     * @param node
     * @param filter
     * @return
     */
    public static List<Geometry> collectSources(Node node, Predicate<Spatial> filter) {
        return collectSources(node, new ArrayList<>(), filter);
    }
    
    /**
     * Gathers all geometries in supplied node into supplied List.
     * 
     * Uses {@link com.jme3.ai.navmesh.gen.TerrainMeshConverter} to merge found
     * Terrain meshes into one geometry prior to adding. Scales and sets translation
     * of merged geometry.
     */
    private static List<Geometry> collectSources(Node node, List<Geometry> results, Predicate<Spatial> filter) {
        for (Spatial spatial : node.getChildren()) {
            if (!filter.test(spatial)) {
                continue;
            }

            if (spatial instanceof Geometry) {
                results.add((Geometry) spatial);

            } else if (spatial instanceof Terrain) {
                Mesh merged = TerrainMeshConverter.convert((Terrain) spatial);
                Geometry geom = new Geometry("mergedTerrain");
                geom.setMesh(merged);
                results.add(geom);

            } else if (spatial instanceof Node) {
                collectSources((Node) spatial, results, filter);
            }
        }
        return results;
    }
    
}
