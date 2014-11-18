package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.yoUtilities.controllers.EuclideanPositionController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


public class CoMBasedMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements MomentumRateOfChangeControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ControlFlowInputPort<FramePoint> desiredCenterOfMassInputPort = createInputPort("desiredCenterOfMassInputPort");
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final EuclideanPositionController comPositionController;

   public CoMBasedMomentumRateOfChangeControlModule(double dt, ReferenceFrame centerOfMassFrame, CenterOfMassJacobian centerOfMassJacobian,
           YoVariableRegistry parentRegistry)
   {
      momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = centerOfMassJacobian;
      boolean visualizeCom=false;
      comPositionController = new EuclideanPositionController("com", centerOfMassFrame, dt, visualizeCom, registry);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      comPositionController.reset();
   }

   @Override
   public void startComputation()
   {
      FramePoint desiredCoM = desiredCenterOfMassInputPort.getData();

      FrameVector comd = new FrameVector(centerOfMassFrame);
      centerOfMassJacobian.packCenterOfMassVelocity(comd);
      FrameVector desiredCoMd = new FrameVector(centerOfMassFrame);

      FrameVector feedForward = new FrameVector(centerOfMassFrame);

      FrameVector output = new FrameVector(centerOfMassFrame);
      comPositionController.compute(output, desiredCoM, desiredCoMd, comd, feedForward);

      momentumRateOfChangeData.setLinearMomentumRateOfChange(output);
   }

   @Override
   public void waitUntilComputationIsDone()
   {
//    empty
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

   public ControlFlowInputPort<FramePoint> getDesiredCoMPositionInputPort()
   {
      return desiredCenterOfMassInputPort;
   }

   @Override
   public void initialize()
   {
//    empty
   }
}
