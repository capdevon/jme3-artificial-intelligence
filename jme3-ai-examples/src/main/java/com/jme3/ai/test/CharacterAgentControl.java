package com.jme3.ai.test;

import java.util.Objects;

import com.jme3.ai.control.NavMeshAgentMT;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;

/**
 *
 * @author capdevon
 */
public class CharacterAgentControl extends AbstractControl {

    private NavMeshAgentMT agent;
    private Animator animator;

    @Override
    public void setSpatial(Spatial spatial) {
        super.setSpatial(spatial);
        if (spatial != null) {
            this.agent = spatial.getControl(NavMeshAgentMT.class);
            Objects.requireNonNull(agent, "NavMeshAgentMT not found: " + spatial);

            this.animator = spatial.getControl(Animator.class);
            Objects.requireNonNull(animator, "Animator not found: " + spatial);
        }
    }

    @Override
    public void controlUpdate(float tpf) {
        if (agent.remainingDistance() < agent.getStoppingDistance()) {
            animator.setAnimation("Idle");
        } else {
            animator.setAnimation("Walk");
        }
    }

    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }

}
