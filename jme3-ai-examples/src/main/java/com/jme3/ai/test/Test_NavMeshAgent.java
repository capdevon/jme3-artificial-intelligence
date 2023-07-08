package com.jme3.ai.test;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.lang3.builder.ReflectionToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;

import com.jme3.ai.control.NavMeshAgentMT;
import com.jme3.ai.control.PathViewer;
import com.jme3.ai.navmesh.gen.GeometryProviderBuilder;
import com.jme3.ai.navmesh.gen.NavMeshBuildSettings;
import com.jme3.ai.navmesh.gen.NavMeshBuilder;
import com.jme3.ai.navmesh.gen.NavMeshDebugRenderer;
import com.jme3.ai.navmesh.gen.NavMeshExporter;
import com.jme3.ai.test.terrain.FractalHeightMap;
import com.jme3.ai.test.terrain.TreeGenerator;
import com.jme3.ai.test.util.MainCamera;
import com.jme3.ai.test.util.TogglePhysicsDebugState;
import com.jme3.anim.util.AnimMigrationUtils;
import com.jme3.app.DebugKeysAppState;
import com.jme3.app.SimpleApplication;
import com.jme3.app.StatsAppState;
import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.control.BetterCharacterControl;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.input.CameraInput;
import com.jme3.input.ChaseCamera;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.input.controls.Trigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FXAAFilter;
import com.jme3.renderer.Caps;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.SceneGraphVisitor;
import com.jme3.scene.Spatial;
import com.jme3.shadow.DirectionalLightShadowFilter;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainLodControl;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.geomipmap.lodcalc.DistanceLodCalculator;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;

/**
 * @author capdevon
 */
public class Test_NavMeshAgent extends SimpleApplication implements ActionListener {

    /**
     * @param args
     */
    public static void main(String[] args) {
        Test_NavMeshAgent app = new Test_NavMeshAgent();

        AppSettings settings = new AppSettings(true);
        settings.setTitle("Test_NavMeshAgent");
        settings.setResolution(1280, 720);
        settings.setBitsPerPixel(32);
        settings.setSamples(4);

        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);
        app.start();
    }

    public Test_NavMeshAgent() {
        super(new StatsAppState(),
                new DebugKeysAppState());
    }

    private BulletAppState physics;
    private Node worldNode = new Node("World");
    private TerrainQuad terrain;
    private Material matRock;
    private Material matWire;
    private boolean wireframe = false;
    
    private Node player;
    private Mesh navMesh;
    private NavMeshAgentMT agent;
    private NavMeshDebugRenderer navMeshRenderer;
    private boolean showNavMesh = true;
    private boolean bakeNavMesh = true;

    @Override
    public void simpleInitApp() {
        
        initPhysics();
        createWireMaterial();
        createTerrainMaterial();
        generateTerrain();
        placeTrees();
        generateNavMesh();
        initPlayer();
        initLights();
        initKeys();
    }

    private void initPhysics() {
        physics = new BulletAppState();
        physics.setDebugEnabled(false);
        stateManager.attach(physics);
        stateManager.attach(new TogglePhysicsDebugState());
    }

    private void initLights() {
        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        AmbientLight ambient = new AmbientLight(ColorRGBA.White.mult(0.6f));
        rootNode.addLight(ambient);

        DirectionalLight light = new DirectionalLight();
        light.setDirection(new Vector3f(-0.1f, -0.7f, -1.0f).normalizeLocal());
        rootNode.addLight(light);

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

    private void initPlayer() {

        player = new Node("Player");
        Node model = (Node) assetManager.loadModel("Models/Jaime/Jaime.j3o");
        model.setName("jaime");
        AnimMigrationUtils.migrate(model);
        player.attachChild(model);

        BetterCharacterControl bcc = new BetterCharacterControl(0.3f, 1.5f, 1f);
        player.addControl(bcc);
        physics.getPhysicsSpace().add(player);
        rootNode.attachChild(player);
        
        PathViewer pathViewer = new PathViewer(assetManager);
        agent = new NavMeshAgentMT(navMesh, pathViewer);
        agent.setSpeed(5f);
        player.addControl(agent);

        player.addControl(new Animator());
        player.addControl(new CharacterAgentControl());

        setupChaseCamera(model);
        bcc.warp(new Vector3f(0, 156, 0));
    }

    private void setupChaseCamera(Spatial target) {
        ChaseCamera chaseCamera = new ChaseCamera(cam, target, inputManager);
        chaseCamera.setMinDistance(5);
        chaseCamera.setMaxDistance(150);
        chaseCamera.setDefaultDistance(25);
        chaseCamera.setSmoothMotion(false);
        chaseCamera.setInvertVerticalAxis(true);
        inputManager.deleteTrigger(CameraInput.CHASECAM_TOGGLEROTATE, new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
    }

    /**
     * creates the NavMesh
     */
    public void generateNavMesh() {

        navMeshRenderer = new NavMeshDebugRenderer(assetManager);

        if (bakeNavMesh) {
            NavMeshBuildSettings nmSettings = new NavMeshBuildSettings();
            nmSettings.setCellSize(.5f);
            nmSettings.setCellHeight(.8f);
            System.out.println(ReflectionToStringBuilder.toString(nmSettings, ToStringStyle.MULTI_LINE_STYLE));

            GeometryProviderBuilder provider = new GeometryProviderBuilder(worldNode);
            NavMeshBuilder navMeshBuilder = new NavMeshBuilder();
            navMeshBuilder.setTimeout(40000);

            System.out.println("Generating new navmesh...");
            navMesh = navMeshBuilder.buildNavMesh(provider.build(), nmSettings);
            
            Path dir = Paths.get("src/main/resources", "Scenes", "NavMesh");
            File file = new File(dir.toFile(), "NavMesh.j3o");
            NavMeshExporter exporter = new NavMeshExporter(assetManager);
            exporter.save(navMesh, file);

        } else {
            System.out.println("Loading navmesh...");
            Geometry geom = (Geometry) assetManager.loadModel("Scenes/NavMesh/NavMesh.j3o");
            navMesh = geom.getMesh();
        }

        if (navMesh != null) {
            navMeshRenderer.drawNavMesh(navMesh);
        } else {
            throw new RuntimeException("NavMesh generation failed!");
        }
    }

    private void generateTerrain() {
        int patchSize = 65;
        int tileSize = 128;
        int terrainSize = (tileSize * 2) + 1;
        float heightScale = 256f;
        float worldScale = 1;
        float worldHeight = 1;

        FractalHeightMap map = new FractalHeightMap(heightScale, terrainSize);
        float[] heightmap = map.getHeightMap();

        terrain = new TerrainQuad("MyTerrain", patchSize, terrainSize, heightmap);
        TerrainLodControl control = new TerrainLodControl(terrain, cam);
        control.setLodCalculator(new DistanceLodCalculator(patchSize, 2.7f)); // patch size, and a multiplier
        terrain.addControl(control);
        terrain.setMaterial(matRock);
        terrain.setModelBound(new BoundingBox());
        terrain.updateModelBound();
        terrain.setLocalScale(worldScale, worldHeight, worldScale);
        //terrain.setLocalTranslation(0, -128, 0);
        terrain.setShadowMode(ShadowMode.Receive);
        worldNode.attachChild(terrain);

        CollisionShape collShape = new HeightfieldCollisionShape(terrain, terrain.getLocalScale());
        RigidBodyControl rbc = new RigidBodyControl(collShape, PhysicsBody.massForStatic);
        terrain.addControl(rbc);
        physics.getPhysicsSpace().add(rbc);

        rootNode.attachChild(worldNode);
    }
    
    private void placeTrees() {
        if (!renderer.getCaps().contains(Caps.MeshInstancing)) {
            System.out.println("MeshInstancing not supported!");
            return;
        }
        
        TreeGenerator generator = new TreeGenerator(assetManager);
        Node trees = generator.generateTrees(terrain);
        worldNode.attachChild(trees);
        
        CompoundCollisionShape ccs = new CompoundCollisionShape();

        trees.depthFirstTraversal(new SceneGraphVisitor() {
            @Override
            public void visit(Spatial sp) {
                RigidBodyControl rb = sp.getControl(RigidBodyControl.class);
                if (rb != null) {
                    CollisionShape shape = rb.getCollisionShape();
                    //float yExtent = ((BoundingBox) sp.getWorldBound()).getYExtent();
                    Transform transform = sp.getWorldTransform().clone();
                    transform.getRotation().multLocal(new Quaternion().fromAngles(FastMath.HALF_PI, 0, 0));
                    //transform.getTranslation().y += yExtent;
                    ccs.addChildShape(shape, transform);
                }
            }
        });

        RigidBodyControl rbc = new RigidBodyControl(ccs, PhysicsBody.massForStatic);
        trees.addControl(rbc);
        physics.getPhysicsSpace().add(rbc);
    }

    private void createTerrainMaterial() {
        float grassScale = 64;
        float dirtScale = 16;
        float rockScale = 128;

        // TERRAIN TEXTURE material
        matRock = new Material(assetManager, "Common/MatDefs/Terrain/HeightBasedTerrain.j3md");
        setTerrainTexture("Textures/Terrain/splat/grass.jpg", "region1", new Vector3f(15, 200, grassScale));
        setTerrainTexture("Textures/Terrain/splat/dirt.jpg", "region2", new Vector3f(0, 20, dirtScale));
        setTerrainTexture("Textures/Terrain/Rock2/rock.jpg", "region3", new Vector3f(198, 260, rockScale));
        setTerrainTexture("Textures/Terrain/Rock2/rock.jpg", "region4", new Vector3f(198, 260, rockScale));

        Texture rock = assetManager.loadTexture("Textures/Terrain/Rock2/rock.jpg");
        rock.setWrap(WrapMode.Repeat);
        matRock.setTexture("slopeColorMap", rock);
        matRock.setFloat("slopeTileFactor", 32);

        matRock.setFloat("terrainSize", 513);
    }

    private void setTerrainTexture(String texture, String name, Vector3f scale) {
        Texture tex = assetManager.loadTexture(texture);
        tex.setWrap(WrapMode.Repeat);
        matRock.setTexture(name + "ColorMap", tex);
        matRock.setVector3(name, scale);
    }

    private void createWireMaterial() {
        // WIREFRAME material (used to debug the terrain, only useful for this test case)
        matWire = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        matWire.getAdditionalRenderState().setWireframe(true);
        matWire.setColor("Color", ColorRGBA.Green);
    }

    private void initKeys() {
        addMapping("toggleWireframe", new KeyTrigger(KeyInput.KEY_T));
        addMapping("stopAgent", new KeyTrigger(KeyInput.KEY_ESCAPE));
        addMapping("LMB", new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
    }

    private void addMapping(String mappingName, Trigger... triggers) {
        inputManager.addMapping(mappingName, triggers);
        inputManager.addListener(this, mappingName);
    }

    @Override
    public void onAction(String name, boolean isPressed, float tpf) {
        if (name.equals("toggleWireframe") && !isPressed) {
            wireframe = !wireframe;
            terrain.setMaterial(wireframe ? matWire : matRock);

        } else if (name.equals("stopAgent") && isPressed) {
            agent.resetPath();

        } else if (name.equals("LMB") && isPressed) {
            onMouseClicked();
        }
    }

    private void onMouseClicked() {
        Ray ray = MainCamera.screenPointToRay(cam, inputManager.getCursorPosition());
        CollisionResults results = new CollisionResults();
        terrain.collideWith(ray, results);

        if (results.size() > 0) {
            CollisionResult closest = results.getClosestCollision();
            Geometry geom = closest.getGeometry();

            if (geom.getName().startsWith("MyTerrain")) {
                Vector3f targetPoint = closest.getContactPoint().clone();
                agent.setDestination(targetPoint);
            }
        } else {
            System.out.println("Nothing...");
        }
    }

    @Override
    public void simpleUpdate(float tpf) {
    }

    @Override
    public void stop() {
        super.stop();
        player.removeControl(NavMeshAgentMT.class);
    }

    @Override
    public void simpleRender(RenderManager rm) {
        if (showNavMesh) {
            navMeshRenderer.show(rm, viewPort);
        }
    }
    
}
