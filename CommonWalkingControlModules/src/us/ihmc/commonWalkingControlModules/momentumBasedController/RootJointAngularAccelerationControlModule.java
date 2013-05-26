package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyOrientationControlModule;
import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class RootJointAngularAccelerationControlModule extends AbstractControlFlowElement
{
   private final YoVariableRegistry registry;

   private final ControlFlowInputPort<OrientationTrajectoryData> desiredPelvisOrientationTrajectoryInputPort = createInputPort
         ("desiredPelvisOrientationTrajectoryInputPort");
   private final ControlFlowOutputPort<RootJointAccelerationData> rootJointAccelerationOutputPort = createOutputPort("rootJointAccelerationOutputPort");

   private final RigidBodyOrientationControlModule rootJointOrientationControlModule;

   private final YoFrameVector desiredPelvisAngularAcceleration;
   private InverseDynamicsJoint rootJoint;

   private final RootJointAccelerationData rootJointAccelerationData;
   private MomentumBasedController momentumBasedController;

   public RootJointAngularAccelerationControlModule(MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = momentumBasedController.getFullRobotModel().getRootJoint();
      registry = new YoVariableRegistry(getClass().getSimpleName());
      rootJointOrientationControlModule = new RigidBodyOrientationControlModule(rootJoint.getName(), rootJoint.getPredecessor(), rootJoint.getSuccessor(),
              momentumBasedController.getTwistCalculator(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desired" + rootJoint.getName() + "AngularAcceleration", rootJoint.getFrameAfterJoint(),
              registry);
      rootJointAccelerationData = new RootJointAccelerationData(rootJoint.getSuccessor().getBodyFixedFrame(), rootJoint.getPredecessor().getBodyFixedFrame(),
              rootJoint.getFrameAfterJoint());
      rootJointAccelerationOutputPort.setData(rootJointAccelerationData);
      this.momentumBasedController = momentumBasedController;
      parentRegistry.addChild(registry);
   }

   public void startComputation()
   {
      FrameVector rootJointAngularAcceleration = computeDesiredRootJointAngularAcceleration();
      this.desiredPelvisAngularAcceleration.set(rootJointAngularAcceleration);
      rootJointAccelerationData.setAngularAcceleration(rootJointAngularAcceleration);
      setRootJointAcceleration();
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

   public ControlFlowOutputPort<RootJointAccelerationData> getRootJointAccelerationOutputPort()
   {
      return rootJointAccelerationOutputPort;
   }

   private void setRootJointAcceleration()
   {
      TaskspaceConstraintData rootJointTaskSpaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector rootJointAcceleration = new SpatialAccelerationVector();
      DenseMatrix64F rootJointAccelerationMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
      DenseMatrix64F rootJointNullspaceMultipliers = new DenseMatrix64F(0, 1);
      DenseMatrix64F rootJointSelectionMatrix = new DenseMatrix64F(1, 1);

      CommonOps.mult(rootJointAccelerationData.getAccelerationSubspace(), rootJointAccelerationData.getAccelerationMultipliers(),
            rootJointAccelerationMatrix);
      rootJointAcceleration.set(rootJointAccelerationData.getBodyFrame(), rootJointAccelerationData.getBaseFrame(),
            rootJointAccelerationData.getExpressedInFrame(), rootJointAccelerationMatrix, 0);
      rootJointAcceleration.changeFrameNoRelativeMotion(rootJointAccelerationData.getBodyFrame());
      DenseMatrix64F accelerationSubspace = rootJointAccelerationData.getAccelerationSubspace();
      rootJointSelectionMatrix.reshape(accelerationSubspace.getNumCols(), accelerationSubspace.getNumRows());
      CommonOps.transpose(accelerationSubspace, rootJointSelectionMatrix);
      rootJointTaskSpaceConstraintData.set(rootJointAcceleration, rootJointNullspaceMultipliers, rootJointSelectionMatrix);
      momentumBasedController.setDesiredSpatialAcceleration(rootJoint.getMotionSubspace(), rootJointTaskSpaceConstraintData);
   }
}
