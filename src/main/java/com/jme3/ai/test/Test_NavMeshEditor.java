package com.jme3.ai.test;

import com.jme3.ai.test.states.NavMeshEditorState;
import com.jme3.ai.test.states.TerrainState;
import com.jme3.ai.test.util.TogglePhysicsDebugState;
import com.jme3.app.SimpleApplication;
import com.jme3.bullet.BulletAppState;
import com.jme3.math.Vector3f;
import com.jme3.system.AppSettings;

/**
 *
 * @author capdevon
 */
public class Test_NavMeshEditor extends SimpleApplication {

    /**
     * @param args
     */
    public static void main(String[] args) {
        Test_NavMeshEditor app = new Test_NavMeshEditor();

        AppSettings settings = new AppSettings(true);
        settings.setResolution(1280, 720);
        settings.setFrameRate(60);

        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {

        configureCamera();

        stateManager.attach(new BulletAppState());
        stateManager.attach(new TogglePhysicsDebugState());
        stateManager.attach(new TerrainState());
        stateManager.attach(new NavMeshEditorState());
    }

    private void configureCamera() {
        cam.setLocation(new Vector3f(10, 256, 10));
        cam.lookAt(Vector3f.ZERO, Vector3f.UNIT_Y);
        cam.setFrustumPerspective(45, (float) cam.getWidth() / cam.getHeight(), 0.1f, 2000f);

        flyCam.setMoveSpeed(50);
        flyCam.setDragToRotate(true);
    }
    
}
