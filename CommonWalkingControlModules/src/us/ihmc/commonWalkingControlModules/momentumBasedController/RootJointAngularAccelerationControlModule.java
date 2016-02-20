package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.SpatialAccelerationCommand;
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
   private final DenseMatrix64F rootJointNullspaceMultipliers = new DenseMatrix64F(0, 1);

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   private final long rootJacobianId;
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
      rootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootPredecessor, rootSuccessor, rootJoint.getFrameAfterJoint());
      spatialAccelerationCommand.set(rootPredecessor, rootSuccessor);
      spatialAccelerationCommand.setJacobianId(rootJacobianId);

      registry = new YoVariableRegistry(getClass().getSimpleName());
      double controlDT = momentumBasedController.getControlDT();
      rootJointOrientationControlModule = new RigidBodyOrientationControlModule(rootJoint.getName(), rootPredecessor, rootSuccessor,
            momentumBasedController.getTwistCalculator(), controlDT, gains, registry);
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
      spatialAccelerationCommand.setAngularAcceleration(rootSuccessor.getBodyFixedFrame(), rootPredecessor.getBodyFixedFrame(), rootJointAngularAcceleration,
            rootJointNullspaceMultipliers);

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
            orientationTrajectoryData.getAngularAcceleration());
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
