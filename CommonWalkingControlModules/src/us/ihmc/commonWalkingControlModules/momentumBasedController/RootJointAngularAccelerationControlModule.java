package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class RootJointAngularAccelerationControlModule implements RootJointAccelerationControlModule
{
   private final YoVariableRegistry registry;

   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort =
      new ControlFlowInputPort<OrientationTrajectoryData>(this);

   private final ControlFlowOutputPort<RootJointAccelerationData> rootJointAccelerationOutputPort = new ControlFlowOutputPort<RootJointAccelerationData>(this);
   private final RigidBodyOrientationControlModule rootJointOrientationControlModule;

   private final YoFrameVector desiredPelvisAngularAcceleration;
   private InverseDynamicsJoint rootJoint;

   private final RootJointAccelerationData rootJointAccelerationData;

   public RootJointAngularAccelerationControlModule(InverseDynamicsJoint rootJoint, TwistCalculator twistCalculator, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = rootJoint;
      registry = new YoVariableRegistry(getClass().getSimpleName());
      rootJointOrientationControlModule = new RigidBodyOrientationControlModule("rootJoint", rootJoint.getPredecessor(), rootJoint.getSuccessor(),
              twistCalculator, registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", rootJoint.getFrameAfterJoint(), registry);
      rootJointAccelerationData = new RootJointAccelerationData(rootJoint.getSuccessor().getBodyFixedFrame(), rootJoint.getPredecessor().getBodyFixedFrame(),
              rootJoint.getFrameAfterJoint());
      rootJointAccelerationOutputPort.setData(rootJointAccelerationData);
      parentRegistry.addChild(registry);
   }

   public void startComputation()
   {
      FrameVector rootJointAngularAcceleration = computeDesiredRootJointAngularAcceleration();
      this.desiredPelvisAngularAcceleration.set(rootJointAngularAcceleration);
      rootJointAccelerationData.setAngularAcceleration(rootJointAngularAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   private FrameVector computeDesiredRootJointAngularAcceleration()
   {
      FrameVector ret = new FrameVector(rootJoint.getFrameAfterJoint());
      OrientationTrajectoryData orientationTrajectoryData = desiredPelvisOrientationTrajectoryInputPort.getData();
      rootJointOrientationControlModule.compute(ret, orientationTrajectoryData.getOrientation(), orientationTrajectoryData.getAngularVelocity(),
              orientationTrajectoryData.getAngularAcceleration());
      ret.changeFrame(rootJoint.getFrameAfterJoint());

      return ret;
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      rootJointOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      rootJointOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public ControlFlowInputPort<OrientationTrajectoryData> getDesiredPelvisOrientationTrajectoryInputPort()
   {
      return desiredPelvisOrientationTrajectoryInputPort;
   }

   public ControlFlowInputPort<?>[] getInputPorts()
   {
      return ControlFlowTools.getInputPorts(this);
   }

   public ControlFlowOutputPort<?>[] getOutputPorts()
   {
      return ControlFlowTools.getOutputPorts(this);
   }

   public ControlFlowOutputPort<RootJointAccelerationData> getRootJointAccelerationOutputPort()
   {
      return rootJointAccelerationOutputPort;
   }
}
