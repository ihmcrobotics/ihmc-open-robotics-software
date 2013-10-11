package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class RootJointAngularAccelerationControlModule
{
   private final YoVariableRegistry registry;

   private final RigidBodyOrientationControlModule rootJointOrientationControlModule;

   private final YoFrameVector desiredPelvisAngularAcceleration;
   private InverseDynamicsJoint rootJoint;
   private final DenseMatrix64F rootJointNullspaceMultipliers = new DenseMatrix64F(0, 1);

   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   private MomentumBasedController momentumBasedController;

   public RootJointAngularAccelerationControlModule(MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = momentumBasedController.getFullRobotModel().getRootJoint();
      registry = new YoVariableRegistry(getClass().getSimpleName());
      rootJointOrientationControlModule = new RigidBodyOrientationControlModule(rootJoint.getName(), rootJoint.getPredecessor(), rootJoint.getSuccessor(),
              momentumBasedController.getTwistCalculator(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desired" + rootJoint.getName() + "AngularAcceleration", rootJoint.getFrameAfterJoint(),
              registry);

      this.momentumBasedController = momentumBasedController;
      parentRegistry.addChild(registry);
   }

   public void doControl(OrientationTrajectoryData orientationTrajectoryData)
   {
      FrameVector rootJointAngularAcceleration = computeDesiredRootJointAngularAcceleration(orientationTrajectoryData);
      taskspaceConstraintData.setAngularAcceleration(rootJoint.getSuccessor().getBodyFixedFrame(), rootJoint.getPredecessor().getBodyFixedFrame(), rootJointAngularAcceleration, rootJointNullspaceMultipliers);
      momentumBasedController.setDesiredSpatialAcceleration(rootJoint.getMotionSubspace(), taskspaceConstraintData);

      rootJointAngularAcceleration.changeFrame(this.desiredPelvisAngularAcceleration.getReferenceFrame());
      this.desiredPelvisAngularAcceleration.set(rootJointAngularAcceleration);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   private FrameVector computeDesiredRootJointAngularAcceleration(OrientationTrajectoryData orientationTrajectoryData)
   {
      FrameVector ret = new FrameVector(rootJoint.getFrameAfterJoint());
      rootJointOrientationControlModule.compute(ret, orientationTrajectoryData.getOrientation(), orientationTrajectoryData.getAngularVelocity(),
              orientationTrajectoryData.getAngularAcceleration());
//      ret.changeFrame(rootJoint.getFrameAfterJoint());

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
}
