package com.jme3.ai.test.states;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AppStateManager;
import com.jme3.app.state.BaseAppState;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.input.InputManager;
import com.jme3.math.Vector2f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;

/**
 * 
 * @author capdevon
 */
public abstract class MyBaseState extends BaseAppState {

    // cache fields
    public SimpleApplication app;
    public AppSettings settings;
    public AppStateManager stateManager;
    public AssetManager assetManager;
    public InputManager inputManager;
    public ViewPort viewPort;
    public Camera camera;
    public Node rootNode;
    public Node guiNode;

    protected void refreshCacheFields() {
        this.app = (SimpleApplication) getApplication();
        this.settings       = app.getContext().getSettings();
        this.stateManager   = app.getStateManager();
        this.assetManager   = app.getAssetManager();
        this.inputManager   = app.getInputManager();
        this.viewPort       = app.getViewPort();
        this.camera         = app.getCamera();
        this.rootNode       = app.getRootNode();
        this.guiNode        = app.getGuiNode();
    }

    public PhysicsSpace getPhysicsSpace() {
        return getState(BulletAppState.class).getPhysicsSpace();
    }

    public Vector2f getScreenSize() {
        return new Vector2f(settings.getWidth(), settings.getHeight());
    }

}
