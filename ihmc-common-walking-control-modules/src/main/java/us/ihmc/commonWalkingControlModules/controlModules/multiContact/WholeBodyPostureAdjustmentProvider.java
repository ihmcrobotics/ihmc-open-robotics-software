package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

public interface WholeBodyPostureAdjustmentProvider
{
   void update();

   boolean isEnabled();

   double getDesiredJointPositionOffset(String jointName);

   double getDesiredJointVelocityOffset(String jointName);

   double getDesiredJointAccelerationOffset(String jointName);

   double getFloatingBasePositionOffsetZ();

   double getFloatingBaseVelocityOffsetZ();

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
