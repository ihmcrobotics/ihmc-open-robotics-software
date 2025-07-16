package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

/**
 * Interface for modifying the whole-body posture of the robot.
 * Setpoints from this class are propagated down to the respective control modules for root joint position/orientation and joint angles.
 * This can be used as a diagnostic tool or an API for a posture-adjustment control module.
 */
public interface WholeBodyPostureAdjustmentProvider
{
   /**
    * Should be called each tick
    */
   void update();

   /**
    * Whether the setpoints from this should be consumed
    */
   boolean isEnabled();

   /**
    * Desired position offset of a 1-dof joint. Used in RigidBodyJointControlHelper and
    * affects JOINTSPACE, TASKSPACE (if hybrid mode) and LOAD-BEARING (if jointspace is active)
    */
   double getDesiredJointPositionOffset(OneDoFJointBasics jointName);

   /**
    * Desired velocity offset of a 1-dof joint. Used in RigidBodyJointControlHelper and
    * affects JOINTSPACE, TASKSPACE (if hybrid mode) and LOAD-BEARING (if jointspace is active)
    */
   double getDesiredJointVelocityOffset(OneDoFJointBasics jointName);

   /**
    * Desired feed-forward acceleration offset of a 1-dof joint. Used in RigidBodyJointControlHelper and
    * affects JOINTSPACE, TASKSPACE (if hybrid mode) and LOAD-BEARING (if jointspace is active)
    */
   double getDesiredJointAccelerationOffset(OneDoFJointBasics jointName);

   /**
    * Desired vertical position offset of the floating-base (pelvis). Used in CenterOfMassHeightControlState.
    */
   double getFloatingBasePositionOffsetZ();

   /**
    * Desired vertical velocity offset of the floating-base (pelvis). Used in CenterOfMassHeightControlState.
    */
   double getFloatingBaseVelocityOffsetZ();

   /**
    * Desired vertical feed-forward acceleration offset of the floating-base (pelvis). Used in CenterOfMassHeightControlState.
    */
   double getFloatingBaseAccelerationOffsetZ();

   /**
    * Desired orientation offset of the floating-base (pelvis). Used in ControllerPelvisOrientationManager
    */
   void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationOffsetToPack);

   /**
    * Desired angular velocity offset of the floating-base (pelvis). Used in ControllerPelvisOrientationManager
    */
   void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack);

   /**
    * Desired angular acceleration offset of the floating-base (pelvis). Used in ControllerPelvisOrientationManager
    */
   void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack);

   static WholeBodyPostureAdjustmentProvider createZeroPostureAdjustmentProvider()
   {
      return new WholeBodyPostureAdjustmentProvider()
      {
         @Override
         public boolean isEnabled()
         {
            return false;
         }

         @Override
         public void update()
         {
         }

         @Override
         public double getDesiredJointPositionOffset(OneDoFJointBasics jointName)
         {
            return 0.0;
         }

         @Override
         public double getDesiredJointVelocityOffset(OneDoFJointBasics jointName)
         {
            return 0.0;
         }

         @Override
         public double getDesiredJointAccelerationOffset(OneDoFJointBasics jointName)
         {
            return 0.0;
         }

         @Override
         public double getFloatingBasePositionOffsetZ()
         {
            return 0.0;
         }

         @Override
         public double getFloatingBaseVelocityOffsetZ()
         {
            return 0.0;
         }

         @Override
         public double getFloatingBaseAccelerationOffsetZ()
         {
            return 0.0;
         }

         @Override
         public void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationToPack)
         {
            orientationToPack.setToZero();
         }

         @Override
         public void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack)
         {
            angularVelocityToPack.setToZero();
         }

         @Override
         public void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack)
         {
            angularAccelerationToPack.setToZero();
         }
      };
   }
}
