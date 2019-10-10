package us.ihmc.exampleSimulations.externalForceEstimation;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;

/*package private*/ class ExternalForceEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double dt;

   private int index = 0;
   private final int integrationWindow;

   private static final double integrationTime = 2.0;
   private final YoDouble wrenchEstimationGain = new YoDouble("wrenchEstimationGain", registry);

   private final OneDoFJointBasics[] joints;
   private final DenseMatrix64F tau;
   private final DenseMatrix64F qd;

   // Frame at which external force is expected to be applied. Aligned with world frame
   private final ReferenceFrame externalForcePointFrame;
   private final FramePoint3D externalForcePoint;
   private final Vector3D externalForcePointOffset = new Vector3D();

   private final DenseMatrix64F currentIntegrandValue;
   private final DenseMatrix64F currentIntegratedValue;
   private final DenseMatrix64F observedExternalJointTorque;
   private final DenseMatrix64F estimatedExternalWrenchMatrix;
   private final DenseMatrix64F hqd;
   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F massMatrixPrev;
   private final DenseMatrix64F massMatrixDot;
   private final DenseMatrix64F coriolisGravityTerm;
   private final DenseMatrix64F observedExternalJointTorqueHistory;
   private final DenseMatrix64F integrandHistory;
   private final DenseMatrix64F hqdHistory;

   private final LinearSolver<DenseMatrix64F> forceEstimateSolver;

   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisCalculator;

   private final GeometricJacobian geometricJacobian;
   private final PointJacobian pointJacobian;
   private final DenseMatrix64F jacobianMatrix;
   private final DenseMatrix64F jacobianMatrixInverse;

   private final YoDouble[] yoObservedExternalJointTorque;
   private final YoDouble[] yoSimulatedTorqueSensingError;

   private final SpatialForce estimatedExternalWrench = new SpatialForce();
   private final YoFixedFrameSpatialForce yoEstimatedExternalWrench;
   private final YoFramePoint3D yoExternalForcePosition = new YoFramePoint3D("externalForcePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicVector estimatedForceVectorGraphic;
   private boolean firstTick = true;

   /**
    * Estimates external force applied at the end of the given joint chain. The force is assumed to be applied at the given offset
    */
   public ExternalForceEstimator(OneDoFJointBasics[] joints, Tuple3DReadOnly externalForcePointOffset, double dt, YoVariableRegistry parentRegistry)
   {
      this.joints = joints;
      this.dt = dt;
      this.wrenchEstimationGain.set(3.0);

      RigidBodyBasics baseLink = joints[0].getPredecessor();
      RigidBodyBasics endEffector = joints[joints.length - 1].getSuccessor();

      this.externalForcePoint = new FramePoint3D(joints[joints.length - 1].getFrameAfterJoint(), externalForcePointOffset);
      this.externalForcePointFrame = new ReferenceFrame("externalForcePointFrame", ReferenceFrame.getWorldFrame(), false, true)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            externalForcePoint.changeFrame(worldFrame);
            transformToParent.setTranslationAndIdentityRotation(externalForcePoint);
         }
      };
      this.externalForcePointOffset.set(externalForcePointOffset);
      this.geometricJacobian = new GeometricJacobian(baseLink, endEffector, externalForcePointFrame);
      this.pointJacobian = new PointJacobian();

      double gravity = 9.81;
      this.gravityCoriolisCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(baseLink, new ArrayList<>(), gravity);
      this.massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(baseLink);

      this.integrationWindow = (int) (integrationTime / dt);

      this.observedExternalJointTorqueHistory = new DenseMatrix64F(joints.length, integrationWindow);
      this.integrandHistory = new DenseMatrix64F(joints.length, integrationWindow);
      this.hqdHistory = new DenseMatrix64F(joints.length, integrationWindow);
      this.currentIntegrandValue = new DenseMatrix64F(joints.length, 1);
      this.currentIntegratedValue = new DenseMatrix64F(joints.length, 1);
      this.observedExternalJointTorque = new DenseMatrix64F(joints.length, 1);
      this.estimatedExternalWrenchMatrix = new DenseMatrix64F(6, 1);
      this.hqd = new DenseMatrix64F(joints.length, 1);
      this.massMatrix = new DenseMatrix64F(joints.length, joints.length);
      this.massMatrixPrev = new DenseMatrix64F(joints.length, joints.length);
      this.massMatrixDot = new DenseMatrix64F(joints.length, joints.length);
      this.coriolisGravityTerm = new DenseMatrix64F(joints.length, 1);
      this.tau = new DenseMatrix64F(joints.length, 1);
      this.qd = new DenseMatrix64F(joints.length, 1);
      this.jacobianMatrix = new DenseMatrix64F(6, joints.length);
      this.jacobianMatrixInverse = new DenseMatrix64F(joints.length, 6);

      forceEstimateSolver = new SolvePseudoInverseSvd(10, 10);

      yoEstimatedExternalWrench = new YoFixedFrameSpatialForce("estimatedWrench", externalForcePointFrame, registry);

      yoObservedExternalJointTorque = new YoDouble[joints.length];
      yoSimulatedTorqueSensingError = new YoDouble[joints.length];
      estimatedForceVectorGraphic = new YoGraphicVector("estimatedForceGraphic",
                                                        yoExternalForcePosition, yoEstimatedExternalWrench.getLinearPart(), 0.05, YoAppearance.Green());

      for (int i = 0; i < joints.length; i++)
      {
         yoObservedExternalJointTorque[i] = new YoDouble("estimatedExternalTau_" + i, registry);
         yoSimulatedTorqueSensingError[i] = new YoDouble("simulatedTauSensorError_" + i, registry);
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      firstTick = true;
      CommonOps.fill(hqdHistory, 0.0);
      CommonOps.fill(observedExternalJointTorqueHistory, 0.0);
      CommonOps.fill(integrandHistory, 0.0);
      CommonOps.fill(observedExternalJointTorque, 0.0);
      index = 0;
   }

   public void compute(double time)
   {
      int nextIndex = getNextIndexWrapped(index, integrationWindow);

      massMatrixPrev.set(massMatrix);
      massMatrix.set(massMatrixCalculator.getMassMatrix());

      if (!firstTick)
      {
         CommonOps.subtract(massMatrix, massMatrixPrev, massMatrixDot);
         CommonOps.scale(1.0 / dt, massMatrixDot);
      }

      gravityCoriolisCalculator.compute();
      for (int i = 0; i < joints.length; i++)
      {
         gravityCoriolisCalculator.getJointCoriolisMatrix(joints[i], coriolisGravityTerm, i);
      }

      MultiBodySystemTools.extractJointsState(joints, JointStateType.EFFORT, tau);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd);

      for (int i = 0; i < joints.length; i++)
      {
         tau.set(i, 0, tau.get(i, 0) + yoSimulatedTorqueSensingError[i].getValue());
      }

      // update current integral
      currentIntegrandValue.set(tau);
      CommonOps.subtractEquals(currentIntegrandValue, coriolisGravityTerm);
      CommonOps.multAdd(massMatrixDot, qd, currentIntegrandValue);
      CommonOps.addEquals(currentIntegrandValue, observedExternalJointTorque);

      // update torque estimate
      MatrixTools.setMatrixBlock(integrandHistory, 0, index, currentIntegrandValue, 0, 0, joints.length, 1, dt);
      CommonOps.sumRows(integrandHistory, currentIntegratedValue);

      CommonOps.mult(massMatrix, qd, hqd);
      MatrixTools.setMatrixBlock(hqdHistory, 0, index, hqd, 0, 0, joints.length, 1, 1.0);

      observedExternalJointTorque.set(hqd);
      MatrixTools.addMatrixBlock(observedExternalJointTorque, 0, 0, hqdHistory, 0, nextIndex, joints.length, 1, -1.0);
      CommonOps.subtractEquals(observedExternalJointTorque, currentIntegratedValue);
      CommonOps.scale(wrenchEstimationGain.getDoubleValue(), observedExternalJointTorque);

      MatrixTools.addMatrixBlock(observedExternalJointTorque, 0, 0, observedExternalJointTorqueHistory, 0, nextIndex, joints.length, 1, 1.0);
      MatrixTools.setMatrixBlock(observedExternalJointTorqueHistory, 0, index, observedExternalJointTorque, 0, 0, joints.length, 1, 1.0);

      externalForcePoint.setIncludingFrame(joints[joints.length - 1].getFrameAfterJoint(), externalForcePointOffset);
      externalForcePointFrame.update();

//      solveUsingFullJacobian();
      solveUsingPointJacobian();

      estimatedExternalWrench.setReferenceFrame(externalForcePointFrame);
      estimatedExternalWrench.set(estimatedExternalWrenchMatrix);
      yoEstimatedExternalWrench.set(estimatedExternalWrench);

      externalForcePoint.changeFrame(worldFrame);
      yoExternalForcePosition.set(externalForcePoint);

      for (int i = 0; i < joints.length; i++)
      {
         yoObservedExternalJointTorque[i].set(observedExternalJointTorque.get(i, 0));
      }

      index = getNextIndexWrapped(index, integrationWindow);
      firstTick = false;
   }

   private void solveUsingFullJacobian()
   {
      geometricJacobian.changeFrame(externalForcePointFrame);
      geometricJacobian.compute();

      jacobianMatrix.set(geometricJacobian.getJacobianMatrix());
      forceEstimateSolver.setA(jacobianMatrix);
      forceEstimateSolver.invert(jacobianMatrixInverse);
      CommonOps.multTransA(jacobianMatrixInverse, observedExternalJointTorque, estimatedExternalWrenchMatrix);
   }

   private final DenseMatrix64F estimatedExternalForceMatrix = new DenseMatrix64F(3, 1);

   private void solveUsingPointJacobian()
   {
      geometricJacobian.changeFrame(geometricJacobian.getBaseFrame());
      geometricJacobian.compute();

      pointJacobian.set(geometricJacobian, externalForcePoint);
      pointJacobian.compute();

      DenseMatrix64F jacobian = pointJacobian.getJacobianMatrix();
      CommonOps.transpose(jacobian);

      forceEstimateSolver.setA(jacobian);
      forceEstimateSolver.solve(observedExternalJointTorque, estimatedExternalForceMatrix);
      CommonOps.fill(estimatedExternalWrenchMatrix, 0.0);
      MatrixTools.setMatrixBlock(estimatedExternalWrenchMatrix, 3, 0, estimatedExternalForceMatrix, 0, 0, 3, 1, 1.0);
   }

   public SpatialForceReadOnly getEstimatedExternalWrench()
   {
      return estimatedExternalWrench;
   }

   private static int getNextIndexWrapped(int index, int size)
   {
      int nextIndex = index + 1;
      return nextIndex >= size ? 0 : nextIndex;
   }

   public YoGraphicVector getEstimatedForceVectorGraphic()
   {
      return estimatedForceVectorGraphic;
   }
}
