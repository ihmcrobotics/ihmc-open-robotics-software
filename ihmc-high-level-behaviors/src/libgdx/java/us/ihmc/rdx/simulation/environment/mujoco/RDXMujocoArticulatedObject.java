package us.ihmc.rdx.simulation.environment.mujoco;

/**
 * An environment object that MuJoCo simulates as its own jointed mechanism, rather than as one
 * mocap or free body. {@link RDXMujocoObjectCollisionSync} takes a {@link #captureDynamics()}
 * snapshot on the UI thread, compiles it into the world, and hands the simulated joint positions
 * back through {@link #applySimulatedJointPositions(double[])}.
 */
public interface RDXMujocoArticulatedObject
{
   /** Physics snapshot of this placed object. Taken on the UI thread; the physics thread only reads it. */
   RDXMujocoArticulatedDynamics captureDynamics();

   /**
    * Joint positions after the last physics tick, in the order the snapshot's
    * {@link RDXMujocoArticulatedDynamics#syncJoints} returned them. Ignored while the user overrides them.
    */
   void applySimulatedJointPositions(double[] jointPositions);
}
