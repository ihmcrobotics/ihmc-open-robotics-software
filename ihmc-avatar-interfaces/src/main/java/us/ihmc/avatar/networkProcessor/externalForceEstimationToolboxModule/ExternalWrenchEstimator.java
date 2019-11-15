package us.ihmc.avatar.networkProcessor.externalForceEstimationToolboxModule;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.IntUnaryOperator;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

/**
 * Module to estimate unknown external wrenches, i.e. contacts that are not expected from the controller
 *
 * Implementation based on Haddadin, et. al:
 * <a href="www.repo.uni-hannover.de/bitstream/handle/123456789/3543/VorndammeSchHad2017_accepted.pdf">Collision Detection, Isolation and Identification for Humanoids</a>
 */
public class ExternalWrenchEstimator implements RobotController
{
   public static final double forceGraphicScale = 0.035;
   private static final int maximumNumberOfContactPoints = 10;
   private static final double defaultEstimatorGain = 0.7;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoBoolean requestInitialize = new YoBoolean("requestInitialize", registry);

   private final YoDouble wrenchEstimationGain = new YoDouble("wrenchEstimationGain", registry);
   private final YoDouble solverAlpha = new YoDouble("solverAlpha", registry);
   private final double dt;

   private final JointBasics[] joints;
   private final int dofs;
   private final DenseMatrix64F tau;
   private final DenseMatrix64F qd;

   private final DenseMatrix64F currentIntegrandValue;
   private final DenseMatrix64F currentIntegratedValue;
   private final DenseMatrix64F observedExternalJointTorque;
   private final DenseMatrix64F hqd0;
   private final DenseMatrix64F hqd;
   private final DenseMatrix64F massMatrix;
   private final DenseMatrix64F massMatrixPrev;
   private final DenseMatrix64F massMatrixDot;
   private final DenseMatrix64F coriolisGravityTerm;

   private final BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter;
   private final Consumer<DenseMatrix64F> tauSetter;

   private final YoDouble[] yoObservedExternalJointTorque;
   private final YoDouble[] yoSimulatedTorqueSensingError;

   private final List<EstimatorContactPoint> contactPoints = new ArrayList<>();
   private final PointJacobian pointJacobian = new PointJacobian();
   private DenseMatrix64F externalWrenchJacobian;
   private DenseMatrix64F externalWrenchJacobianTranspose;
   private DenseMatrix64F estimatedExternalWrenchMatrix;
   private DampedLeastSquaresSolver forceEstimateSolver;

   private final YoFixedFrameSpatialVector[] estimatedExternalWrenches = new YoFixedFrameSpatialVector[maximumNumberOfContactPoints];
   private final YoFramePoint3D[] contactPointPositions = new YoFramePoint3D[maximumNumberOfContactPoints];

   private final FramePoint3D tempPoint = new FramePoint3D();
   private boolean firstTick = true;

   /**
    * Estimates external force on the given system. To set the end-effector and external force point position, call {@link #addContactPoint}
    *
    * @param dynamicMatrixSetter sets the mass matrix and coriolis-gravity matrices respectively
    */
   public ExternalWrenchEstimator(JointBasics[] joints,
                                  double dt,
                                  BiConsumer<DenseMatrix64F, DenseMatrix64F> dynamicMatrixSetter,
                                  Consumer<DenseMatrix64F> tauSetter,
                                  YoGraphicsListRegistry graphicsListRegistry,
                                  YoVariableRegistry parentRegistry)
   {
      this.joints = joints;
      this.tauSetter = tauSetter;
      this.dt = dt;
      this.wrenchEstimationGain.set(defaultEstimatorGain);
      this.dynamicMatrixSetter = dynamicMatrixSetter;

      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

      this.currentIntegrandValue = new DenseMatrix64F(dofs, 1);
      this.currentIntegratedValue = new DenseMatrix64F(dofs, 1);
      this.observedExternalJointTorque = new DenseMatrix64F(dofs, 1);
      this.hqd = new DenseMatrix64F(dofs, 1);
      this.massMatrix = new DenseMatrix64F(dofs, dofs);
      this.massMatrixPrev = new DenseMatrix64F(dofs, dofs);
      this.massMatrixDot = new DenseMatrix64F(dofs, dofs);
      this.coriolisGravityTerm = new DenseMatrix64F(dofs, 1);
      this.tau = new DenseMatrix64F(dofs, 1);
      this.qd = new DenseMatrix64F(dofs, 1);
      this.hqd0 = new DenseMatrix64F(dofs, 1);
      this.solverAlpha.set(0.001);

      this.yoObservedExternalJointTorque = new YoDouble[dofs];
      this.yoSimulatedTorqueSensingError = new YoDouble[dofs];

      for (int i = 0; i < maximumNumberOfContactPoints; i++)
      {
         estimatedExternalWrenches[i] = new YoFixedFrameSpatialVector("estimatedExternalWrench" + i, ReferenceFrame.getWorldFrame(), registry);
         contactPointPositions[i] = new YoFramePoint3D("contactPoint" + i, ReferenceFrame.getWorldFrame(), registry);

         if(graphicsListRegistry != null)
         {
            YoGraphicVector forceGraphic = new YoGraphicVector("estimatedForceGraphic" + i, contactPointPositions[i], estimatedExternalWrenches[i].getLinearPart(), forceGraphicScale);
            graphicsListRegistry.registerYoGraphic(name, forceGraphic);
         }
      }

      boolean hasFloatingBase = joints[0] instanceof FloatingJointBasics;
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

   private class EstimatorContactPoint
   {
      final RigidBodyBasics endEffector;
      final Tuple3DReadOnly contactPointOffset;
      final GeometricJacobian contactPointJacobian;
      final ReferenceFrame contactPointFrame;
      final boolean assumeZeroTorque;
      final int numberOfDecisionVariables;

      // indexing helper to go from this contactPointJacobian to externalWrenchJacobian
      // contactPointJacobian column i ==> externalWrenchJacobian column indexMap[i]
      final int[] indexMap;

      EstimatorContactPoint(RigidBodyBasics endEffector, Tuple3DReadOnly contactPointOffset, GeometricJacobian contactPointJacobian, boolean assumeZeroTorque)
      {
         this.endEffector = endEffector;
         this.contactPointOffset = contactPointOffset;
         this.contactPointJacobian = contactPointJacobian;
         this.indexMap = new int[contactPointJacobian.getJacobianMatrix().getNumCols()];
         this.assumeZeroTorque = assumeZeroTorque;
         this.numberOfDecisionVariables = assumeZeroTorque ? 3 : 6;

         this.contactPointFrame = new ReferenceFrame(endEffector.getName() + "ContactFrame", ReferenceFrame.getWorldFrame())
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               tempPoint.setIncludingFrame(endEffector.getParentJoint().getFrameAfterJoint(), contactPointOffset);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               transformToParent.setTranslationAndIdentityRotation(tempPoint);
            }
         };

         contactPointJacobian.changeFrame(contactPointFrame);

         JointBasics[] jointPath = contactPointJacobian.getJointsInOrder();
         ToIntFunction<JointBasics> jointIndexFunction = joint -> IntStream.range(0, joints.length)
                                                                           .filter(i -> joint == joints[i])
                                                                           .findFirst()
                                                                           .orElseThrow(() -> new RuntimeException("Could not find joint"));
         IntUnaryOperator indexOffset = i -> IntStream.range(0, i).map(j -> joints[j].getDegreesOfFreedom()).sum();

         for (int jointIndex = 0, mappedIndex = 0; jointIndex < jointPath.length; jointIndex++)
         {
            JointBasics joint = jointPath[jointIndex];
            int offset = indexOffset.applyAsInt(jointIndexFunction.applyAsInt(joint));

            for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
            {
               indexMap[mappedIndex++] = offset + i;
            }
         }
      }
   }

   public void clearContactPoints()
   {
      contactPoints.clear();
   }

   /**
    * Adds a contact point to the estimator.
    *
    * @param rigidBody body that the contact point is on
    * @param contactPointOffset the contact point's position in the rigid body's parent joint's "frame after"
    * @param assumeZeroTorque true if only linear force should be estimated at the contact point
    */
   public void addContactPoint(RigidBodyBasics rigidBody, Tuple3DReadOnly contactPointOffset, boolean assumeZeroTorque)
   {
      if(contactPoints.size() == maximumNumberOfContactPoints)
         throw new RuntimeException("The maximum number of contact points (" + maximumNumberOfContactPoints + ") has been reached. Increase to add more points");

      RigidBodyBasics baseLink = joints[0].getPredecessor();
      GeometricJacobian jacobian = new GeometricJacobian(baseLink, rigidBody, baseLink.getBodyFixedFrame());
      contactPoints.add(new EstimatorContactPoint(rigidBody, contactPointOffset, jacobian, assumeZeroTorque));
   }

   @Override
   public void initialize()
   {
      firstTick = true;
      CommonOps.fill(observedExternalJointTorque, 0.0);
      CommonOps.fill(currentIntegratedValue, 0.0);

      for (int i = 0; i < maximumNumberOfContactPoints; i++)
      {
         estimatedExternalWrenches[i].setToNaN();
         contactPointPositions[i].setToNaN();
      }

      int decisionVariables = contactPoints.stream().mapToInt(p -> p.numberOfDecisionVariables).sum();
      externalWrenchJacobian = new DenseMatrix64F(decisionVariables, dofs);
      externalWrenchJacobianTranspose = new DenseMatrix64F(dofs, decisionVariables);
      estimatedExternalWrenchMatrix = new DenseMatrix64F(decisionVariables, 1);

      forceEstimateSolver = new DampedLeastSquaresSolver(dofs, solverAlpha.getDoubleValue());
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
      }
   }

   private void computeForceEstimate()
   {
      massMatrixPrev.set(massMatrix);

      dynamicMatrixSetter.accept(massMatrix, coriolisGravityTerm);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, qd);
      tauSetter.accept(tau);

      if (firstTick)
      {
         CommonOps.mult(massMatrix, qd, hqd0);
         firstTick = false;
      }
      else
      {
         CommonOps.subtract(massMatrix, massMatrixPrev, massMatrixDot);
         CommonOps.scale(1.0 / dt, massMatrixDot);
      }

      for (int i = 0; i < dofs; i++)
      {
         tau.set(i, 0, tau.get(i, 0) - yoSimulatedTorqueSensingError[i].getValue());
      }

      // update integral
      currentIntegrandValue.set(tau);
      CommonOps.subtractEquals(currentIntegrandValue, coriolisGravityTerm);
      CommonOps.multAdd(massMatrixDot, qd, currentIntegrandValue);
      CommonOps.addEquals(currentIntegrandValue, observedExternalJointTorque);
      CommonOps.addEquals(currentIntegratedValue, dt, currentIntegrandValue);

      // calculate observed external joint torque
      CommonOps.mult(massMatrix, qd, hqd);
      CommonOps.subtract(hqd, hqd0, observedExternalJointTorque);
      CommonOps.subtractEquals(observedExternalJointTorque, currentIntegratedValue);
      CommonOps.scale(wrenchEstimationGain.getDoubleValue(), observedExternalJointTorque);

      for (int i = 0; i < dofs; i++)
      {
         yoObservedExternalJointTorque[i].set(observedExternalJointTorque.get(i, 0));
      }

      // compute jacobian
      CommonOps.fill(externalWrenchJacobian, 0.0);
      for (int i = 0; i < contactPoints.size(); i++)
      {
         EstimatorContactPoint estimatorContactPoint = contactPoints.get(i);

         int numberOfRows = estimatorContactPoint.numberOfDecisionVariables;
         int rowOffset = IntStream.range(0, i).map(index -> contactPoints.get(index).numberOfDecisionVariables).sum();

         DenseMatrix64F contactJacobianMatrix;
         if(estimatorContactPoint.assumeZeroTorque)
         {
            ReferenceFrame baseFrame = estimatorContactPoint.contactPointJacobian.getBaseFrame();
            estimatorContactPoint.contactPointJacobian.changeFrame(baseFrame);
            estimatorContactPoint.contactPointJacobian.compute();

            tempPoint.setIncludingFrame(estimatorContactPoint.endEffector.getParentJoint().getFrameAfterJoint(), estimatorContactPoint.contactPointOffset);
            pointJacobian.set(estimatorContactPoint.contactPointJacobian, tempPoint);
            pointJacobian.compute();
            contactJacobianMatrix = pointJacobian.getJacobianMatrix();
         }
         else
         {
            estimatorContactPoint.contactPointFrame.update();
            estimatorContactPoint.contactPointJacobian.changeFrame(estimatorContactPoint.contactPointFrame);
            estimatorContactPoint.contactPointJacobian.compute();
            contactJacobianMatrix = estimatorContactPoint.contactPointJacobian.getJacobianMatrix();
         }

         for (int j = 0; j < contactJacobianMatrix.getNumCols(); j++)
         {
            int column = estimatorContactPoint.indexMap[j];
            MatrixTools.setMatrixBlock(externalWrenchJacobian, rowOffset, column, contactJacobianMatrix, 0, j, numberOfRows, 1, 1.0);
         }
      }

      // solve for external wrench
      CommonOps.transpose(externalWrenchJacobian, externalWrenchJacobianTranspose);
      forceEstimateSolver.setA(externalWrenchJacobianTranspose);
      forceEstimateSolver.solve(observedExternalJointTorque, estimatedExternalWrenchMatrix);

      // pack result and update graphics variables
      for (int i = 0; i < contactPoints.size(); i++)
      {
         int rowOffset = IntStream.range(0, i).map(index -> contactPoints.get(index).numberOfDecisionVariables).sum();
         if(contactPoints.get(i).assumeZeroTorque)
         {
            estimatedExternalWrenches[i].getAngularPart().setToZero();
            estimatedExternalWrenches[i].getLinearPart().set(rowOffset, estimatedExternalWrenchMatrix);
         }
         else
         {
            estimatedExternalWrenches[i].set(rowOffset, estimatedExternalWrenchMatrix);
         }

         tempPoint.setIncludingFrame(contactPoints.get(i).endEffector.getParentJoint().getFrameAfterJoint(), contactPoints.get(i).contactPointOffset);
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPointPositions[i].set(tempPoint);
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void requestInitialize()
   {
      this.requestInitialize.set(true);
   }

   public void setEstimatorGain(double estimatorGain)
   {
      this.wrenchEstimationGain.set(estimatorGain);
   }

   public YoFixedFrameSpatialVector[] getEstimatedExternalWrenches()
   {
      return estimatedExternalWrenches;
   }

   public int getNumberOfContactPoints()
   {
      return contactPoints.size();
   }

   public void setSolverAlpha(double alpha)
   {
      solverAlpha.set(alpha);
   }
}
