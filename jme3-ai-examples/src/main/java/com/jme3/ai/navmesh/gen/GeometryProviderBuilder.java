package com.jme3.ai.navmesh.gen;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer.Type;
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
                Mesh merged = terrain2mesh((Terrain) spatial);
                Geometry geom = new Geometry("mergedTerrain");
                geom.setMesh(merged);
                results.add(geom);

            } else if (spatial instanceof Node) {
                findGeometries((Node) spatial, results, filter);
            }
        }
        return results;
    }
    
    private Mesh terrain2mesh(Terrain terrain) {
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

}
