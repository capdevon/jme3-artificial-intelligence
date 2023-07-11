package com.jme3.ai.test.terrain;

import java.util.ArrayList;
import java.util.List;

import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.instancing.InstancedNode;
import com.jme3.scene.shape.Cylinder;
import com.jme3.terrain.geomipmap.TerrainPatch;
import com.jme3.terrain.geomipmap.TerrainQuad;

/**
 * 
 * @author capdevon
 */
public class TreeGenerator {
    
    protected AssetManager assetManager;
    private Spatial[] trees = null;
    
    /**
     * 
     * @param assetManager
     */
    public TreeGenerator(AssetManager assetManager) {
        this.assetManager = assetManager;
        initModels();
    }
    
    private void initModels() {
        trees = new Spatial[8];
        for (int i = 0; i < trees.length; i++) {
            float weight = (1.0f / trees.length) * i;
            ColorRGBA color = new ColorRGBA().interpolateLocal(ColorRGBA.Brown, ColorRGBA.Green, weight);
            Spatial model = createCylinder("Cylynder", color, 1f, 20f);
            trees[i] = model;
        }
    }
    
    public Node generateTrees(TerrainQuad terrain) {
        InstancedNode instancedNode = new InstancedNode("Trees-" + terrain.getName());

        List<TerrainPatch> patches = new ArrayList<>();
        terrain.getAllTerrainPatches(patches);
        for (TerrainPatch patch : patches) {

            //just a simple planting algorithm
            Vector3f center = patch.getWorldBound().getCenter();
            float radius = 16f;
            int elements = 8;
            
            for (int i = 0; i < elements; i++) {
                float angle = FastMath.TWO_PI * i / elements;
                float x = FastMath.cos(angle) * radius;
                float z = FastMath.sin(angle) * radius;
                Vector3f v = center.add(x, 0, z);

                float y = terrain.getHeight(new Vector2f(v.x, v.z));
                Vector3f location = new Vector3f(v.x, y, v.z);

                spawnObject(location, instancedNode);
            }
        }
        
        instancedNode.instance();
        return instancedNode;
    }
    
    private void spawnObject(Vector3f position, Node parent) {
        boolean cloneMaterial = false;
        int index = FastMath.nextRandomInt(0, trees.length - 1);
        Spatial tree = trees[index].clone(cloneMaterial);
        BoundingBox bbox = (BoundingBox) tree.getWorldBound();
        tree.setLocalTranslation(position.addLocal(0, bbox.getYExtent(), 0));
        parent.attachChild(tree);

        CollisionShape collShape = new BoxCollisionShape(1f, bbox.getYExtent(), 1f);
        tree.addControl(new RigidBodyControl(collShape, 0f));
    }
    
    private Geometry createCylinder(String name, ColorRGBA color, float radius, float height) {
        Cylinder mesh = new Cylinder(2, 8, radius, height, true);
        Geometry geo = new Geometry(name, mesh);
        Material mat = createPBRLighting(color);
        mat.setBoolean("UseInstancing", true);
        geo.setMaterial(mat);
        geo.rotate(FastMath.HALF_PI, 0, 0);
        geo.setShadowMode(ShadowMode.Cast);
        return geo;
    }
    
    private Material createPBRLighting(ColorRGBA color) {
        Material mat = new Material(assetManager, "Common/MatDefs/Light/PBRLighting.j3md");
        mat.setName("PBRLighting");
        mat.setColor("BaseColor", color);
        mat.setFloat("Metallic", 0);
        mat.setFloat("Roughness", 0.4f);
        return mat;
    }

}
