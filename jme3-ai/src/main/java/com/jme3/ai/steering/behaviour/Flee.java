package com.jme3.ai.steering.behaviour;

import com.jme3.math.Vector3f;

/**
 * Flee is simply the inverse of seek and acts to steer the 
 * character so that its velocity is radially aligned away 
 * from the target. The desired velocity points in the opposite direction.
 * 
 * @author Brent Owens
 */
public class Flee implements Behaviour {
    
    public Vector3f calculateForce(Vector3f location, Vector3f velocity, 
            float speed, Vector3f target) {

        Vector3f desiredVel = target.subtract(location).normalize().mult(speed);
        Vector3f steering = desiredVel.subtract(velocity).negate(); // negate flee

        return steering;
    }
}
