package com.examples.states;

import com.examples.terrain.FractalHeightMap;
import com.examples.terrain.TreeGenerator;
import com.jme3.app.Application;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.Trigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FXAAFilter;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.shadow.DirectionalLightShadowFilter;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;

/**
 *
 * @author capdevon
 */
public class TerrainState extends MyBaseState implements ActionListener {

    private Node worldNode = new Node("World");
    private TerrainQuad terrain;
    private Material matRock;
    private Material matWire;
    private boolean wireframe = false;

    @Override
    protected void initialize(Application appl) {
        refreshCacheFields();
        setupMatWire();
        setupMatTerrain();
        setupTerrain();
        initLights();
        setupKeys();
    }

    @Override
    protected void cleanup(Application app) {
    }

    @Override
    protected void onEnable() {
    }

    @Override
    protected void onDisable() {
    }

    private void initLights() {
        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        DirectionalLight light = new DirectionalLight();
        light.setDirection(new Vector3f(-0.5f, -1f, -0.5f).normalizeLocal());
        light.setName("sun");
        rootNode.addLight(light);

        AmbientLight ambient = new AmbientLight();
        ambient.setColor(new ColorRGBA(0.25f, 0.25f, 0.25f, 1));
        ambient.setName("ambient");
        rootNode.addLight(ambient);

        DirectionalLightShadowFilter shadowFilter = new DirectionalLightShadowFilter(assetManager, 4096, 2);
        shadowFilter.setLight(light);
        shadowFilter.setShadowIntensity(0.4f);
        shadowFilter.setShadowZExtend(256);

        FXAAFilter fxaa = new FXAAFilter();

        FilterPostProcessor fpp = new FilterPostProcessor(assetManager);
        fpp.addFilter(shadowFilter);
        fpp.addFilter(fxaa);
        viewPort.addProcessor(fpp);
    }
    
    private void setupTerrain() {

        int patchSize = 65;
        int tileSize = 128;
        int terrainSize = (tileSize * 2) + 1;
        float heightScale = 256f;
        float worldScale = 1;
        float worldHeight = 1;

        FractalHeightMap map = new FractalHeightMap(heightScale, terrainSize);
        float[] heightmap = map.getHeightMap();
        
        terrain = new TerrainQuad("MyTerrain", patchSize, terrainSize, heightmap);
        terrain.setMaterial(matRock);
        terrain.setLocalScale(worldScale, worldHeight, worldScale);
        terrain.setShadowMode(RenderQueue.ShadowMode.Receive);
        worldNode.attachChild(terrain);
        
        TreeGenerator gen = new TreeGenerator(assetManager);
        Node trees = gen.generateTrees(terrain);
        worldNode.attachChild(trees);
        
        rootNode.attachChild(worldNode);
//        worldNode.setLocalTranslation(0, -128, 0);
    }

    private void setupMatTerrain() {
        // TERRAIN TEXTURE material
        matRock = new Material(assetManager, "Common/MatDefs/Terrain/Terrain.j3md");
        matRock.setBoolean("useTriPlanarMapping", false);

        // ALPHA map (for splat textures)
        matRock.setTexture("Alpha", assetManager.loadTexture("Textures/Terrain/splat/alphamap.png"));
        
        setTerrainTexture("Textures/Terrain/splat/grass.jpg", "Tex1", 64);
        setTerrainTexture("Textures/Terrain/splat/dirt.jpg", "Tex2", 16);
        setTerrainTexture("Textures/Terrain/splat/road.jpg", "Tex3", 128);
    }

    private void setTerrainTexture(String texture, String name, float scale) {
        Texture tex = assetManager.loadTexture(texture);
        tex.setWrap(WrapMode.Repeat);
        matRock.setTexture(name, tex);
        matRock.setFloat(name + "Scale", scale);
    }

    private void setupMatWire() {
        // WIREFRAME material (used to debug the terrain, only useful for this test case)
        matWire = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        matWire.getAdditionalRenderState().setWireframe(true);
        matWire.setColor("Color", ColorRGBA.Green);
    }

    private void setupKeys() {
        addMapping("ToggleWireframe", new KeyTrigger(KeyInput.KEY_T));
    }

    private void addMapping(String mappingName, Trigger... triggers) {
        inputManager.addMapping(mappingName, triggers);
        inputManager.addListener(this, mappingName);
    }

    @Override
    public void onAction(String name, boolean pressed, float tpf) {
        if (name.equals("ToggleWireframe") && !pressed) {
            wireframe = !wireframe;
            terrain.setMaterial(wireframe ? matWire : matRock);
        }
    }

}
