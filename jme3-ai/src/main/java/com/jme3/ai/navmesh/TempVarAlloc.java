package com.jme3.ai.navmesh;

import java.util.ArrayDeque;
import java.util.Deque;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

/**
 * TempVarAlloc class provides temporary variable allocation for each thread.
 * It ensures that a limited number of instances are used and properly released.
 */
public class TempVarAlloc {
    
    /**
     * Maximum number of TempVar instances allowed per thread.
     */
    private static final int STACK_SIZE = 5;

    /**
     * TempVarsStack contains a stack of TempVarAlloc instances for each thread.
     */
    private static class TempVarsStack {
        Deque<TempVarAlloc> tempVars = new ArrayDeque<>(STACK_SIZE);
    }
    
    /**
     * ThreadLocal to store a TempVarsStack for each thread.
     */
    private static final ThreadLocal<TempVarsStack> threadLocal = ThreadLocal.withInitial(TempVarsStack::new);
    
    /**
     * Indicates whether this instance of TempVarAlloc is currently in use.
     */
    private boolean isUsed = false;

    /**
     * Private constructor to prevent direct instantiation.
     */
    private TempVarAlloc() {
    }

    /**
     * Acquires an instance of the TempVarAlloc class.
     * You must release the instance after use by calling the release() method.
     * If more than STACK_SIZE instances are requested in a single thread,
     * an IllegalStateException will be thrown.
     *
     * @return A TempVarAlloc instance
     */
    public static TempVarAlloc get() {
        TempVarsStack stack = threadLocal.get();

        TempVarAlloc instance;
        if (stack.tempVars.isEmpty() || stack.tempVars.size() < STACK_SIZE) {
            // Create a new instance if the stack is empty or not full
            instance = new TempVarAlloc();
            stack.tempVars.push(instance);
        } else {
            // Reuse the existing instance at the top of the stack
            instance = stack.tempVars.peek();
        }

        instance.isUsed = true;
        return instance;
    }

    /**
     * Releases this instance of TempVarAlloc.
     * Once released, the contents of the TempVarAlloc are undefined.
     * The TempVarAlloc must be released in the opposite order that they are retrieved.
     * If not, an IllegalStateException will be thrown.
     */
    public void release() {
        if (!isUsed) {
            throw new IllegalStateException("This instance of TempVar was already released!");
        }

        isUsed = false;
        TempVarsStack stack = threadLocal.get();

        // Ensure the instance being released is the one at the top of the stack
        if (stack.tempVars.peek() != this) {
            throw new IllegalStateException("An instance of TempVar has not been released in the correct order!");
        }

        // Remove the instance from the stack
        stack.tempVars.pop();
    }

    // General vectors
    public final Vector3f vec31 = new Vector3f();
    public final Vector3f vec32 = new Vector3f();
    public final Vector3f vec33 = new Vector3f();
    public final Vector3f vec34 = new Vector3f();
    public final Vector3f vec35 = new Vector3f();

    // 2D vectors
    public final Vector2f vec21 = new Vector2f();
    public final Vector2f vec22 = new Vector2f();
    public final Vector2f vec23 = new Vector2f();
    public final Vector2f vec24 = new Vector2f();
    public final Vector2f vec25 = new Vector2f();

    // General quaternions
    public final Quaternion quat1 = new Quaternion();
    public final Quaternion quat2 = new Quaternion();

}
