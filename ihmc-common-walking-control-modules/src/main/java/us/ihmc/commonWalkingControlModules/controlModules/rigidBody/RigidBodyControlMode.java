package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

/**
 * The mode of control as part of the {@link RigidBodyControlManager}.
 */
public enum RigidBodyControlMode
{
   /** Manages joint space control via {@link RigidBodyJointspaceControlState} */
   JOINTSPACE,
   /** Manages {@link RigidBodyPositionController position}, {@link RigidBodyOrientationController orientation},
    * or {@link RigidBodyPoseController pose} taskspace control. */
   TASKSPACE,
   /** Simple pass-through of user jointspace acceleration commands via {@link RigidBodyUserControlState} */
   USER,
   /** Mode where the rigid body bears the load of the robot. {@link RigidBodyLoadBearingControlState} */
   LOADBEARING,
}
