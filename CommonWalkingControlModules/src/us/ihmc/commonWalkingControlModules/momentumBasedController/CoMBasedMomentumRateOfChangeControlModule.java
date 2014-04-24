package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;

public class CoMBasedMomentumRateOfChangeControlModule extends AbstractControlFlowElement implements MomentumRateOfChangeControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ControlFlowInputPort<FramePoint> desiredCenterOfMassInputPort = createInputPort("desiredCenterOfMassInputPort");
   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = createOutputPort("momentumRateOfChangeOutputPort");
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final EuclideanPositionController comPositionController;

   public CoMBasedMomentumRateOfChangeControlModule(double dt, ReferenceFrame centerOfMassFrame, CenterOfMassJacobian centerOfMassJacobian,
           YoVariableRegistry parentRegistry)
   {
      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = centerOfMassJacobian;
      boolean visualizeCom=false;
      this.comPositionController = new EuclideanPositionController("com", centerOfMassFrame, dt, visualizeCom, registry);
      momentumRateOfChangeOutputPort.setData(momentumRateOfChangeData);
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      comPositionController.reset();
   }
   
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

   public void waitUntilComputationIsDone()
   {
//    empty
   }

   public ControlFlowOutputPort<MomentumRateOfChangeData> getMomentumRateOfChangeOutputPort()
   {
      return momentumRateOfChangeOutputPort;
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

   public void initialize()
   {
//    empty
   }
}
