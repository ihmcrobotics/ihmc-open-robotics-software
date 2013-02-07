package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.EuclideanPositionController;

public class CoMBasedMomentumRateOfChangeControlModule implements MomentumRateOfChangeControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ControlFlowInputPort<FramePoint> desiredCenterOfMassInputPort = new ControlFlowInputPort<FramePoint>(this);
   private final ControlFlowOutputPort<MomentumRateOfChangeData> momentumRateOfChangeOutputPort = new ControlFlowOutputPort<MomentumRateOfChangeData>(this);
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final EuclideanPositionController comPositionController;

   public CoMBasedMomentumRateOfChangeControlModule(ReferenceFrame centerOfMassFrame, CenterOfMassJacobian centerOfMassJacobian,
           YoVariableRegistry parentRegistry)
   {
      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.comPositionController = new EuclideanPositionController("com", centerOfMassFrame, registry);
      momentumRateOfChangeOutputPort.setData(momentumRateOfChangeData);
      parentRegistry.addChild(registry);
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

   public ControlFlowInputPort<?>[] getInputPorts()
   {
      return ControlFlowTools.getInputPorts(this);
   }

   public ControlFlowOutputPort<?>[] getOutputPorts()
   {
      return ControlFlowTools.getOutputPorts(this);

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
}
