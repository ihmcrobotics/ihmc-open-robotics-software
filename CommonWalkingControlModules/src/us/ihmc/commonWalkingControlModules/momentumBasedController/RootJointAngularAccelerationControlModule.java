package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class RootJointAngularAccelerationControlModule
{
   private final YoVariableRegistry registry;

   private final RigidBodyOrientationControlModule rootJointOrientationControlModule;

   private final YoFrameVector controlledPelvisAngularAcceleration;
   private InverseDynamicsJoint rootJoint;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final RigidBody rootPredecessor;
   private final RigidBody rootSuccessor;

   public RootJointAngularAccelerationControlModule(MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      this(momentumBasedController, null, parentRegistry);
   }

   public RootJointAngularAccelerationControlModule(MomentumBasedController momentumBasedController, YoOrientationPIDGainsInterface gains,
         YoVariableRegistry parentRegistry)
   {
      this.rootJoint = momentumBasedController.getFullRobotModel().getRootJoint();

      rootPredecessor = rootJoint.getPredecessor();
      rootSuccessor = rootJoint.getSuccessor();
      spatialAccelerationCommand.set(rootPredecessor, rootSuccessor);

      registry = new YoVariableRegistry(getClass().getSimpleName());
      double controlDT = momentumBasedController.getControlDT();
      rootJointOrientationControlModule = new RigidBodyOrientationControlModule(rootJoint.getName(), rootSuccessor, momentumBasedController.getTwistCalculator(),
            controlDT, gains, registry);
      this.controlledPelvisAngularAcceleration = new YoFrameVector("controlled" + rootJoint.getName() + "AngularAcceleration", rootJoint.getFrameAfterJoint(),
            registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rootJointOrientationControlModule.reset();
   }

   private final FrameVector rootJointAngularAcceleration = new FrameVector();

   public void doControl(OrientationTrajectoryData orientationTrajectoryData)
   {
      computeDesiredRootJointAngularAcceleration(orientationTrajectoryData, rootJointAngularAcceleration);
      spatialAccelerationCommand.setAngularAcceleration(rootSuccessor.getBodyFixedFrame(), rootPredecessor.getBodyFixedFrame(), rootJointAngularAcceleration);

      rootJointAngularAcceleration.changeFrame(this.controlledPelvisAngularAcceleration.getReferenceFrame());
      this.controlledPelvisAngularAcceleration.set(rootJointAngularAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   private void computeDesiredRootJointAngularAcceleration(OrientationTrajectoryData orientationTrajectoryData, FrameVector vectorToPack)
   {
      vectorToPack.setToZero(rootJoint.getFrameAfterJoint());
      rootJointOrientationControlModule.compute(vectorToPack, orientationTrajectoryData.getOrientation(), orientationTrajectoryData.getAngularVelocity(),
            orientationTrajectoryData.getAngularAcceleration(), rootPredecessor);
      //      ret.changeFrame(rootJoint.getFrameAfterJoint());
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      rootJointOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      rootJointOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      rootJointOrientationControlModule.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }
}
