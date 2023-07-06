package com.jme3.ai.test.states;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.apache.commons.lang3.builder.ReflectionToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;
import org.critterai.nmgen.IntermediateData;

import com.jme3.ai.navmesh.gen.GeometryProviderBuilder;
import com.jme3.ai.navmesh.gen.NavMeshBuildSettings;
import com.jme3.ai.navmesh.gen.NavMeshBuilder;
import com.jme3.ai.navmesh.gen.NavMeshDebugRenderer;
import com.jme3.ai.navmesh.gen.NavMeshExporter;
import com.jme3.app.Application;
import com.jme3.math.Vector2f;
import com.jme3.renderer.RenderManager;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.simsilica.lemur.Button;
import com.simsilica.lemur.Container;
import com.simsilica.lemur.GuiGlobals;
import com.simsilica.lemur.RollupPanel;
import com.simsilica.lemur.props.PropertyPanel;
import com.simsilica.lemur.style.BaseStyles;

/**
 *
 * @author capdevon
 */
public class NavMeshEditorState extends MyBaseState {

    private static final Logger logger = Logger.getLogger(NavMeshEditorState.class.getName());

    private Vector2f screenSize;
    private Container container;
    private NavMeshDebugRenderer navMeshRenderer;
    private boolean navMeshDebugEnabled = true;
    private boolean autoSave = true;

    @Override
    protected void initialize(Application app) {
        
        refreshCacheFields();

        screenSize = getScreenSize();
        navMeshRenderer = new NavMeshDebugRenderer(assetManager);

        // initialize lemur
        GuiGlobals.initialize(app);
        BaseStyles.loadGlassStyle();
        GuiGlobals.getInstance().getStyles().setDefaultStyle("glass");
        initComponents();
    }

    public boolean isNavMeshDebugEnabled() {
        return navMeshDebugEnabled;
    }

    public void setNavMeshDebugEnabled(boolean navMeshDebugEnabled) {
        this.navMeshDebugEnabled = navMeshDebugEnabled;
    }

    public boolean isAutoSave() {
        return autoSave;
    }

    public void setAutoSave(boolean autoSave) {
        this.autoSave = autoSave;
    }

    private Container initComponents() {

        NavMeshBuildSettings nmSettings = new NavMeshBuildSettings();
        nmSettings.setCellSize(.5f);
        nmSettings.setCellHeight(.8f);
//        try {
//            String dirName = System.getProperty("user.dir") + "/src/main/resources/Scenes";
//            String fileName = "navmesh.properties";
//            nmSettings = NavMeshProperties.fromFile(new File(dirName, fileName));
//
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }

        container = new Container();
        container.setLocalTranslation(10, screenSize.y - 10, 1);

        PropertyPanel propertyPanel = new PropertyPanel("glass");
        propertyPanel.addFloatProperty("CellSize", nmSettings, "cellSize", 0.01f, 10, 0.1f);
        propertyPanel.addFloatProperty("CellHeight", nmSettings, "cellHeight", 0.01f, 10, 0.1f);
        propertyPanel.addFloatProperty("MinTraversableHeight", nmSettings, "minTraversableHeight", 0.01f, 10, 0.1f);
        propertyPanel.addFloatProperty("MaxTraversableStep", nmSettings, "maxTraversableStep", 0.01f, 10, 0.1f);
        propertyPanel.addFloatProperty("MaxTraversableSlope", nmSettings, "maxTraversableSlope", 0.01f, 64, 0.1f);
        propertyPanel.addBooleanProperty("ClipLedges", nmSettings, "clipLedges");
        propertyPanel.addFloatProperty("TraversableAreaBorderSize", nmSettings, "traversableAreaBorderSize", 0.01f, 20, 0.1f);
        propertyPanel.addIntProperty("SmoothingThreshold", nmSettings, "smoothingThreshold", 1, 16, 1);
        propertyPanel.addBooleanProperty("UseConservativeExpansion", nmSettings, "useConservativeExpansion");
        propertyPanel.addIntProperty("MinUnconnectedRegionSize", nmSettings, "minUnconnectedRegionSize", 1, 32, 1);
        propertyPanel.addIntProperty("MergeRegionSize", nmSettings, "mergeRegionSize", 1, 32, 1);
        propertyPanel.addFloatProperty("MaxEdgeLength", nmSettings, "maxEdgeLength", 0.01f, 32, 0.1f);
        propertyPanel.addFloatProperty("EdgeMaxDeviation", nmSettings, "edgeMaxDeviation", 1, 5, 0.1f);
        propertyPanel.addIntProperty("MaxVertsPerPoly", nmSettings, "maxVertsPerPoly", 3, 12, 1);
        propertyPanel.addFloatProperty("ContourSampleDistance", nmSettings, "contourSampleDistance", 0.01f, 32, 0.1f);
        propertyPanel.addFloatProperty("ContourMaxDeviation", nmSettings, "contourMaxDeviation", 0.01f, 32, 0.1f);

        propertyPanel.addBooleanProperty("Show NavMesh", this, "navMeshDebugEnabled");
        propertyPanel.addBooleanProperty("Auto Save", this, "autoSave");

        RollupPanel rollup = new RollupPanel("NavMesh Settings", propertyPanel, "glass");
        rollup.setAlpha(0, false);
        //rollup.setOpen(false);
        container.addChild(rollup);
        
        Button button = container.addChild(new Button("Bake NavMesh"));
        button.addClickCommands(source -> {
            generateNavMesh(nmSettings);
        });
        
        return container;
    }

    /**
     * creates the NavMesh
     */
    public void generateNavMesh(NavMeshBuildSettings nmSettings) {

        navMeshRenderer.clear();

        // the data object to use for storing data related to building the navigation mesh.
        IntermediateData data = new IntermediateData();
        GeometryProviderBuilder provider = new GeometryProviderBuilder(rootNode);
        NavMeshBuilder builder = new NavMeshBuilder();
        builder.setIntermediateData(data);
        builder.setTimeout(40000);
        
        System.out.println("Generating new navmesh... please wait");
        long startTime = System.currentTimeMillis();
        
        Mesh optiMesh = builder.buildNavMesh(provider.build(), nmSettings);

        if (optiMesh != null) {
            logger.log(Level.INFO, ReflectionToStringBuilder.toString(data, ToStringStyle.MULTI_LINE_STYLE));
            
            if (autoSave) {
                Path dir = Paths.get("src/main/resources", "Scenes", "NavMesh");
                File file = new File(dir.toFile(), "NavMesh.j3o");
                NavMeshExporter exporter = new NavMeshExporter(assetManager);
                exporter.save(optiMesh, file);
            }

            navMeshRenderer.drawNavMesh(optiMesh);
            Node debugNode = navMeshRenderer.debugNode;
//            debugNode.setLocalTranslation(0, -127.98f, 0);
            
            long endTime = System.currentTimeMillis();
            System.out.println("Generated navmesh in " + (endTime - startTime) + " ms");

        } else {
            logger.log(Level.SEVERE, "NavMesh generation failed!");
        }
    }
    
    @Override
    public void render(RenderManager rm) {
        if (navMeshDebugEnabled) {
            // display the mesh
            navMeshRenderer.show(rm, viewPort);
        }
    }

    @Override
    protected void cleanup(Application app) {
    }

    @Override
    protected void onEnable() {
    	guiNode.attachChild(container);
    }

    @Override
    protected void onDisable() {
        guiNode.detachChild(container);
    }
    
}
