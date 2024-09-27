package com.jme3.ai.navmesh.gen;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.terrain.Terrain;

public class TerrainMeshConverter {
    
    /**
     * Takes a Terrain, which can be composed of numerous meshes, and converts them
     * into a single mesh.
     *
     * @param terrain the terrain to be converted
     * @return a single mesh consisting of all meshes of a Terrain
     */
    static Mesh convert(Terrain terrain) {
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
