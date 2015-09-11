package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.yoUtilities.controllers.EuclideanPositionController;

public class CoMBasedMomentumRateOfChangeControlModule implements MomentumRateOfChangeControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final EuclideanPositionController comPositionController;
   private final FramePoint desiredCoM = new FramePoint();

   public CoMBasedMomentumRateOfChangeControlModule(double dt, ReferenceFrame centerOfMassFrame, CenterOfMassJacobian centerOfMassJacobian,
         YoVariableRegistry parentRegistry)
   {
      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = centerOfMassJacobian;
      boolean visualizeCom = false;
      comPositionController = new EuclideanPositionController("com", centerOfMassFrame, dt, visualizeCom, registry);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      comPositionController.reset();
   }

   @Override
   public void compute()
   {
      FrameVector comd = new FrameVector(centerOfMassFrame);
      centerOfMassJacobian.packCenterOfMassVelocity(comd);
      FrameVector desiredCoMd = new FrameVector(centerOfMassFrame);

      FrameVector feedForward = new FrameVector(centerOfMassFrame);

      FrameVector output = new FrameVector(centerOfMassFrame);
      comPositionController.compute(output, desiredCoM, desiredCoMd, comd, feedForward);

      momentumRateOfChangeData.setLinearMomentumRateOfChange(output);
   }

   @Override
   public void initialize()
   {
      //    empty
   }

   public void setDesiredCoMPosition(YoFramePoint newDesiredCenterOfMass)
   {
      newDesiredCenterOfMass.getFrameTupleIncludingFrame(desiredCoM);
   }

   public void setDesiredCoMPosition(FramePoint newDesiredCenterOfMass)
   {
      desiredCoM.setIncludingFrame(newDesiredCenterOfMass);
   }

   @Override
   public void getMomentumRateOfChange(MomentumRateOfChangeData momentumRateOfChangeDataToPack)
   {
      momentumRateOfChangeDataToPack.set(momentumRateOfChangeData);
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      comPositionController.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      comPositionController.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }
}
