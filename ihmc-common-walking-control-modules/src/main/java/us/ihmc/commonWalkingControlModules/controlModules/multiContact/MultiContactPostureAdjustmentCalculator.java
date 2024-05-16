package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MultiContactPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private final CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator;
   private final WholeBodyContactState wholeBodyContactState;

   public MultiContactPostureAdjustmentCalculator(CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator,
                                                  WholeBodyContactState wholeBodyContactState,
                                                  YoRegistry parentRegistry)
   {
      this.staticStabilityRegionCalculator = staticStabilityRegionCalculator;
      this.wholeBodyContactState = wholeBodyContactState;
   }

   @Override
   public void update()
   {
      // pattern match from KinematicSupportRegionOptimizerVisualizer

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////// COMPUTE TORQUE REDUCTION OBJECTIVES //////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////// COMPUTE CONSTRAINT JACOBIAN //////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // project into nullspace of constraint jacobian...

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////// INTEGRATE AND DECAY VELOCITIES  ////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

   }

   @Override
   public boolean isEnabled()
   {
      return true;
   }

   @Override
   public double getDesiredJointPositionOffset(String jointName)
   {
      return 0;
   }

   @Override
   public double getDesiredJointVelocityOffset(String jointName)
   {
      return 0;
   }

   @Override
   public double getDesiredJointAccelerationOffset(String jointName)
   {
      return 0;
   }

   @Override
   public double getFloatingBasePositionOffsetZ()
   {
      return 0;
   }

   @Override
   public double getFloatingBaseVelocityOffsetZ()
   {
      return 0;
   }

   @Override
   public double getFloatingBaseAccelerationOffsetZ()
   {
      return 0;
   }

   @Override
   public void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationOffsetToPack)
   {

   }

   @Override
   public void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack)
   {

   }

   @Override
   public void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack)
   {

   }
}
