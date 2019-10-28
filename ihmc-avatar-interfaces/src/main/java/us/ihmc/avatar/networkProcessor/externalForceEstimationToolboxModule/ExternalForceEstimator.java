package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialForce;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

/**
 * Module to estimate unknown external wrenches, i.e. contacts that are not expected from the controller
 *
 * Implementation based on Haddadin, et. al:
 * <a href="www.repo.uni-hannover.de/bitstream/handle/123456789/3543/VorndammeSchHad2017_accepted.pdf">Collision Detection, Isolation and Identification for Humanoids</a>
 */
public class ExternalForceEstimator implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean requestInitialize = new YoBoolean("requestInitialize", registry);

   private final double dt;

   private int index = 0;
   private final int integrationWindow;
   private final boolean hasFloatingBase;

   private static final double integrationTime = 2.5;
   private static final double defaultEstimatorGain = 0.7;
   private final YoDouble wrenchEstimationGain = new YoDouble("wrenchEstimationGain", registry);

   private final JointBasics[] joints;
   private final int dofs;
   private final DenseMatrix64F tau;
   private final DenseMatrix64F qd;

   private final DenseMatrix64F currentIntegrandValue;
   private final DenseMatrix64F currentIntegratedValue;
   private final DenseMatrix64F observedExternalJointTorque;
   private final DenseMatrix64F estimatedExternalForceMatrix;
   private final DenseMatrix64F hqd;
   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F massMatrixPrev;
   private final DenseMatrix64F massMatrixDot;
   private final DenseMatrix64F coriolisGravityTerm;
   private final DenseMatrix64F observedExternalJointTorqueHistory;
   private final DenseMatrix64F integrandHistory;
   private final DenseMatrix64F hqdHistory;

   private final BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter;
   private final Consumer<DenseMatrix64F> tauSetter;
   private final LinearSolver<DenseMatrix64F> forceEstimateSolver;

   private final FramePoint3D externalForcePoint = new FramePoint3D();
   private final Vector3D externalForcePointOffset = new Vector3D();

   private GeometricJacobian geometricJacobian;
   private RigidBodyBasics endEffector;
   private DenseMatrix64F observedExternalTorqueAlongEndEffectorPath;
   private int[] torqueIndicesAlongEndEffectorPath;

   private PointJacobian pointJacobian = new PointJacobian();
   private DenseMatrix64F pointJacobianTranspose;

   private final YoDouble[] yoObservedExternalJointTorque;
   private final YoDouble[] yoSimulatedTorqueSensingError;

   private final YoFramePoint3D externalForcePosition = new YoFramePoint3D("externalForcePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D estimatedExternalForce = new YoFrameVector3D("estimatedExternalForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D yoLinkFramePose = new YoFramePose3D("linkFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicVector estimatedForceVectorGraphic;
   private boolean firstTick = true;

   /**
    * Estimates external force on the given system. To set the end-effector and external force point position, call {@link #setEndEffector}
    *
    * @param dynamicMatrixSetter sets the mass matrix and coriolis-gravity matrices respectively
    */
   public ExternalForceEstimator(JointBasics[] joints,
                                 double dt,
                                 BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter,
                                 Consumer<DenseMatrix64F> tauSetter,
                                 YoVariableRegistry parentRegistry)
   {
      this.joints = joints;
      this.tauSetter = tauSetter;
      this.dt = dt;
      this.wrenchEstimationGain.set(defaultEstimatorGain);
      this.dynamicMatrixSetter = dynamicMatrixSetter;
      this.hasFloatingBase = joints[0] instanceof FloatingJointBasics;

      this.integrationWindow = (int) (integrationTime / dt);
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.observedExternalJointTorqueHistory = new DenseMatrix64F(dofs, integrationWindow);
      this.integrandHistory = new DenseMatrix64F(dofs, integrationWindow);
      this.hqdHistory = new DenseMatrix64F(dofs, integrationWindow);
      this.currentIntegrandValue = new DenseMatrix64F(dofs, 1);
      this.currentIntegratedValue = new DenseMatrix64F(dofs, 1);
      this.observedExternalJointTorque = new DenseMatrix64F(dofs, 1);
      this.estimatedExternalForceMatrix = new DenseMatrix64F(3, 1);
      this.hqd = new DenseMatrix64F(dofs, 1);
      this.massMatrix = new DenseMatrix64F(dofs, dofs);
      this.massMatrixPrev = new DenseMatrix64F(dofs, dofs);
      this.massMatrixDot = new DenseMatrix64F(dofs, dofs);
      this.coriolisGravityTerm = new DenseMatrix64F(dofs, 1);
      this.tau = new DenseMatrix64F(dofs, 1);
      this.qd = new DenseMatrix64F(dofs, 1);
      this.forceEstimateSolver = new DampedLeastSquaresSolver(dofs, 0.05); // new SolvePseudoInverseSvd(dofs, dofs); //

      this.yoObservedExternalJointTorque = new YoDouble[dofs];
      this.yoSimulatedTorqueSensingError = new YoDouble[dofs];
      this.estimatedForceVectorGraphic = new YoGraphicVector("estimatedForceGraphic",
                                                             externalForcePosition,
                                                             estimatedExternalForce,
                                                             0.05,
                                                             YoAppearance.Green());

      for (int i = 0; i < dofs; i++)
      {
         String nameSuffix;
         if (i < 6 && hasFloatingBase)
         {
            nameSuffix = (i < 3 ? "Ang" : "Lin") + Axis.values[i % 3];
         }
         else
         {
            nameSuffix = joints[i + (hasFloatingBase ? -5 : 0)].getName();
         }

         yoObservedExternalJointTorque[i] = new YoDouble("estimatedExternalTau_" + nameSuffix, registry);
         yoSimulatedTorqueSensingError[i] = new YoDouble("simulatedTauSensorError_" + nameSuffix, registry);
      }

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * Specifies the rigid body and position at which the external force is assumed to be applied. The position is in the rigid body's body-fixed frame
    */
   public void setEndEffector(RigidBodyBasics endEffector, Tuple3DReadOnly externalForcePointOffset)
   {
      RigidBodyBasics baseLink = joints[0].getPredecessor();
      this.endEffector = endEffector;

      this.geometricJacobian = new GeometricJacobian(baseLink, endEffector, baseLink.getBodyFixedFrame());
      this.pointJacobianTranspose = new DenseMatrix64F(geometricJacobian.getNumberOfColumns(), 3);
      this.observedExternalTorqueAlongEndEffectorPath = new DenseMatrix64F(geometricJacobian.getNumberOfColumns(), 1);
      this.torqueIndicesAlongEndEffectorPath = new int[geometricJacobian.getNumberOfColumns()];

      if (hasFloatingBase)
      {
         JointBasics[] jointPathFromBaseToEndEffector = geometricJacobian.getJointPathFromBaseToEndEffector();
         for (int i = 0; i < 6; i++)
         {
            torqueIndicesAlongEndEffectorPath[i] = i;
         }

         ToIntFunction<JointBasics> jointIndexFunction = joint -> IntStream.range(1, joints.length).filter(i -> joint == joints[i]).findFirst().orElseThrow(() -> new RuntimeException("Could not find joint"));
         int jointTorqueIndex = 6;
         for (int i = 1; i < jointPathFromBaseToEndEffector.length; i++)
         {
            torqueIndicesAlongEndEffectorPath[jointTorqueIndex++] = 5 + jointIndexFunction.applyAsInt(jointPathFromBaseToEndEffector[i]);
         }
      }
      else
      {
         for (int i = 0; i < torqueIndicesAlongEndEffectorPath.length; i++)
         {
            torqueIndicesAlongEndEffectorPath[i] = i;
         }
      }

      this.externalForcePoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), externalForcePointOffset);
      this.externalForcePointOffset.set(externalForcePointOffset);
   }

   public void setEstimatorGain(double estimatorGain)
   {
      this.wrenchEstimationGain.set(estimatorGain);
   }

   @Override
   public void initialize()
   {
      firstTick = true;
      CommonOps.fill(hqdHistory, 0.0);
      CommonOps.fill(observedExternalJointTorqueHistory, 0.0);
      CommonOps.fill(integrandHistory, 0.0);
      CommonOps.fill(observedExternalJointTorque, 0.0);
      index = 0;
   }

   @Override
   public void doControl()
   {
      if (requestInitialize.getValue())
      {
         initialize();
         requestInitialize.set(false);
      }

      try
      {
         computeForceEstimate();
      }
      catch(Exception e)
      {
         e.printStackTrace();
         index = getPreviousIndexWrapped(index, integrationWindow);
      }

      index = getNextIndexWrapped(index, integrationWindow);
      firstTick = false;
   }

   private void computeForceEstimate()
   {
      int nextIndex = getNextIndexWrapped(index, integrationWindow);
      massMatrixPrev.set(massMatrix);

      dynamicMatrixSetter.accept(massMatrix, coriolisGravityTerm);

      if (!firstTick)
      {
         CommonOps.subtract(massMatrix, massMatrixPrev, massMatrixDot);
         CommonOps.scale(1.0 / dt, massMatrixDot);
      }

      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd);
      tauSetter.accept(tau);

      for (int i = 0; i < dofs; i++)
      {
         tau.set(i, 0, tau.get(i, 0) - yoSimulatedTorqueSensingError[i].getValue());
      }

      // update current integral
      currentIntegrandValue.set(tau);
      CommonOps.subtractEquals(currentIntegrandValue, coriolisGravityTerm);
      CommonOps.multAdd(massMatrixDot, qd, currentIntegrandValue);
      CommonOps.addEquals(currentIntegrandValue, observedExternalJointTorque);

      // update torque estimate
      MatrixTools.setMatrixBlock(integrandHistory, 0, index, currentIntegrandValue, 0, 0, dofs, 1, dt);
      CommonOps.sumRows(integrandHistory, currentIntegratedValue);

      CommonOps.mult(massMatrix, qd, hqd);
      MatrixTools.setMatrixBlock(hqdHistory, 0, index, hqd, 0, 0, dofs, 1, 1.0);

      observedExternalJointTorque.set(hqd);
      MatrixTools.addMatrixBlock(observedExternalJointTorque, 0, 0, hqdHistory, 0, nextIndex, dofs, 1, -1.0);
      CommonOps.subtractEquals(observedExternalJointTorque, currentIntegratedValue);
      CommonOps.scale(wrenchEstimationGain.getDoubleValue(), observedExternalJointTorque);

      MatrixTools.addMatrixBlock(observedExternalJointTorque, 0, 0, observedExternalJointTorqueHistory, 0, nextIndex, dofs, 1, 1.0);
      MatrixTools.setMatrixBlock(observedExternalJointTorqueHistory, 0, index, observedExternalJointTorque, 0, 0, dofs, 1, 1.0);

      externalForcePoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), externalForcePointOffset);
      
      for (int i = 0; i < torqueIndicesAlongEndEffectorPath.length; i++)
      {
         int jointIndex = torqueIndicesAlongEndEffectorPath[i];
         observedExternalTorqueAlongEndEffectorPath.set(i, 0, observedExternalJointTorque.get(jointIndex, 0));
      }

      solveUsingPointJacobian();

      externalForcePoint.changeFrame(worldFrame);
      externalForcePosition.set(externalForcePoint);

      for (int i = 0; i < dofs; i++)
      {
         yoObservedExternalJointTorque[i].set(observedExternalJointTorque.get(i, 0));
      }
   }

   private void solveUsingPointJacobian()
   {
      geometricJacobian.changeFrame(geometricJacobian.getBaseFrame());
      geometricJacobian.compute();

      pointJacobian.set(geometricJacobian, externalForcePoint);
      pointJacobian.compute();
      CommonOps.transpose(pointJacobian.getJacobianMatrix(), pointJacobianTranspose);

      forceEstimateSolver.setA(pointJacobianTranspose);
      forceEstimateSolver.solve(observedExternalTorqueAlongEndEffectorPath, estimatedExternalForceMatrix);

      estimatedExternalForce.set(estimatedExternalForceMatrix);
   }

   public YoFrameVector3D getEstimatedExternalForce()
   {
      return estimatedExternalForce;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private static int getPreviousIndexWrapped(int index, int size)
   {
      int previousIndex = index - 1;
      return previousIndex < 0 ? size - 1 : previousIndex;
   }

   private static int getNextIndexWrapped(int index, int size)
   {
      int nextIndex = index + 1;
      return nextIndex >= size ? 0 : nextIndex;
   }

   public void requestInitialize()
   {
      this.requestInitialize.set(true);
   }

   public YoGraphicVector getEstimatedForceVectorGraphic()
   {
      return estimatedForceVectorGraphic;
   }

   public YoFramePose3D getYoLinkFramePose()
   {
      return yoLinkFramePose;
   }
}
