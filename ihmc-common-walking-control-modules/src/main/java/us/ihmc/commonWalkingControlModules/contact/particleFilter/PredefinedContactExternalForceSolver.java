package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

public class PredefinedContactExternalForceSolver implements RobotController
{
   public static final double forceGraphicScale = 0.035;
   private static final int maximumNumberOfContactPoints = 10;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble solverAlpha = new YoDouble("solverAlpha", registry);
   private final JointBasics[] joints;
   private final int dofs;

   private final List<PredefinedContactPoint> contactPoints = new ArrayList<>();
   private final JointspaceExternalContactEstimator jointspaceExternalContactEstimator;

   private final YoFixedFrameSpatialVector[] estimatedExternalWrenches = new YoFixedFrameSpatialVector[maximumNumberOfContactPoints];
   private final YoFramePoint3D[] contactPointPositions = new YoFramePoint3D[maximumNumberOfContactPoints];

   private DMatrixRMaj externalWrenchJacobian;
   private DMatrixRMaj externalWrenchJacobianTranspose;
   private DMatrixRMaj estimatedExternalWrenchMatrix;
   private DampedLeastSquaresSolver forceEstimateSolver;

   private final FramePoint3D tempPoint = new FramePoint3D();

   public PredefinedContactExternalForceSolver(JointBasics[] joints,
                                               double dt,
                                               ForceEstimatorDynamicMatrixUpdater dynamicMatrixUpdater,
                                               YoGraphicsListRegistry graphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      this.solverAlpha.set(0.001);
      this.joints = joints;

      this.jointspaceExternalContactEstimator = new JointspaceExternalContactEstimator(joints, dt, dynamicMatrixUpdater, registry);
      this.dofs = Arrays.stream(joints).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();

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

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
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

      PredefinedContactPoint contactPoint = new PredefinedContactPoint(joints, rigidBody, assumeZeroTorque);
      contactPoint.setContactPointOffset(contactPointOffset);
      contactPoints.add(contactPoint);
   }

   public void clearContactPoints()
   {
      contactPoints.clear();
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < maximumNumberOfContactPoints; i++)
      {
         estimatedExternalWrenches[i].setToNaN();
         contactPointPositions[i].setToNaN();
      }

      int decisionVariables = contactPoints.stream().mapToInt(PredefinedContactPoint::getNumberOfDecisionVariables).sum();
      externalWrenchJacobian = new DMatrixRMaj(decisionVariables, dofs);
      externalWrenchJacobianTranspose = new DMatrixRMaj(dofs, decisionVariables);
      estimatedExternalWrenchMatrix = new DMatrixRMaj(decisionVariables, 1);

      forceEstimateSolver = new DampedLeastSquaresSolver(dofs, solverAlpha.getDoubleValue());
      jointspaceExternalContactEstimator.initialize();
   }

   @Override
   public void doControl()
   {
      jointspaceExternalContactEstimator.doControl();

      // compute jacobian
      CommonOps_DDRM.fill(externalWrenchJacobian, 0.0);
      for (int i = 0; i < contactPoints.size(); i++)
      {
         PredefinedContactPoint forceEstimatorContactPoint = contactPoints.get(i);

         int numberOfRows = forceEstimatorContactPoint.getNumberOfDecisionVariables();
         int rowOffset = IntStream.range(0, i).map(index -> contactPoints.get(index).getNumberOfDecisionVariables()).sum();

         DMatrixRMaj contactJacobianMatrix = forceEstimatorContactPoint.computeContactJacobian();

         for (int j = 0; j < contactJacobianMatrix.getNumCols(); j++)
         {
            int column = forceEstimatorContactPoint.getSystemJacobianIndex(j);
            MatrixTools.setMatrixBlock(externalWrenchJacobian, rowOffset, column, contactJacobianMatrix, 0, j, numberOfRows, 1, 1.0);
         }
      }

      // solve for external wrench
      DMatrixRMaj observedExternalJointTorque = jointspaceExternalContactEstimator.getObservedExternalJointTorque();

      CommonOps_DDRM.transpose(externalWrenchJacobian, externalWrenchJacobianTranspose);
      forceEstimateSolver.setA(externalWrenchJacobianTranspose);
      forceEstimateSolver.solve(observedExternalJointTorque, estimatedExternalWrenchMatrix);

      // pack result and update graphics variables
      for (int i = 0; i < contactPoints.size(); i++)
      {
         int rowOffset = IntStream.range(0, i).map(index -> contactPoints.get(index).getNumberOfDecisionVariables()).sum();
         if(contactPoints.get(i).getAssumeZeroTorque())
         {
            estimatedExternalWrenches[i].getAngularPart().setToZero();
            estimatedExternalWrenches[i].getLinearPart().set(rowOffset, estimatedExternalWrenchMatrix);
         }
         else
         {
            estimatedExternalWrenches[i].set(rowOffset, estimatedExternalWrenchMatrix);
         }

         tempPoint.setIncludingFrame(contactPoints.get(i).getRigidBody().getParentJoint().getFrameAfterJoint(), contactPoints.get(i).getContactPointOffset());
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         contactPointPositions[i].set(tempPoint);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
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

   public void setEstimatorGain(double estimatorGain)
   {
      jointspaceExternalContactEstimator.setEstimatorGain(estimatorGain);
   }
}
