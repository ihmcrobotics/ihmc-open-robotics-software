package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

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
   double getDesiredJointPositionOffset(String jointName);

   /**
    * Desired velocity offset of a 1-dof joint. Used in RigidBodyJointControlHelper and
    * affects JOINTSPACE, TASKSPACE (if hybrid mode) and LOAD-BEARING (if jointspace is active)
    */
   double getDesiredJointVelocityOffset(String jointName);

   /**
    * Desired feed-forward acceleration offset of a 1-dof joint. Used in RigidBodyJointControlHelper and
    * affects JOINTSPACE, TASKSPACE (if hybrid mode) and LOAD-BEARING (if jointspace is active)
    */
   double getDesiredJointAccelerationOffset(String jointName);

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
         public double getDesiredJointPositionOffset(String jointName)
         {
            return 0.0;
         }

         @Override
         public double getDesiredJointVelocityOffset(String jointName)
         {
            return 0.0;
         }

         @Override
         public double getDesiredJointAccelerationOffset(String jointName)
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
      };
   }
}
