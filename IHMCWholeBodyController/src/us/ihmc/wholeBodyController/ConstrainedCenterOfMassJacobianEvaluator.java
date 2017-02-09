package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import com.google.common.primitives.Doubles;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.ColumnSpaceProjector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ConstrainedCenterOfMassJacobianCalculator;
import us.ihmc.robotics.screwTheory.ConstrainedCentroidalMomentumMatrixCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

/**
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianEvaluator implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ConstrainedCenterOfMassJacobianCalculator constrainedCenterOfMassJacobianCalculator;
   private final ConstrainedCentroidalMomentumMatrixCalculator constrainedCentroidalMomentumMatrixCalculator;
   private final ReferenceFrame centerOfMassFrame;

   private final DoubleYoVariable comJacobianConditionNumber = new DoubleYoVariable("comJacCondition", registry);
   private final DoubleYoVariable comJacobianSigmaMin = new DoubleYoVariable("comJacobianSigmaMin", registry);
   private final DoubleYoVariable constrainedComJacobianConditionNumber = new DoubleYoVariable("constrComJacCondition", registry);
   private final DoubleYoVariable constrainedComJacobianSigmaMin = new DoubleYoVariable("constrComJacobianSigmaMin", registry);
   private final YoFrameVector comVelocity = new YoFrameVector("comJacComVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector constrainedComVelocity = new YoFrameVector("constrComJacComVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable cmmConditionNumber = new DoubleYoVariable("CMMCondition", registry);
   private final DoubleYoVariable cmmSigmaMin = new DoubleYoVariable("CMMSigmaMin", registry);
   private final DoubleYoVariable constrainedCMMConditionNumber = new DoubleYoVariable("constrCMMCondition", registry);
   private final DoubleYoVariable constrainedCMMSigmaMin = new DoubleYoVariable("constrCMMSigmaMin", registry);

   private final InverseDynamicsJoint[] allJoints;
   private final DenseMatrix64F v;
   private final DenseMatrix64F vActuated;
   private final DenseMatrix64F tempCoMVelocityMatrix = new DenseMatrix64F(3, 1);
   private final Vector3d tempCoMVelocity = new Vector3d();
   private final Collection<InverseDynamicsJoint> actuatedJoints;
   private final ColumnSpaceProjector projector;

   private final YoFrameVector centroidalLinearMomentum;
   private final YoFrameVector centroidalAngularMomentum;

   private final YoFrameVector centroidalLinearMomentumPlus;
   private final YoFrameVector centroidalAngularMomentumPlus;
   private final DenseMatrix64F momentumSelectionMatrix;

   public ConstrainedCenterOfMassJacobianEvaluator(FullHumanoidRobotModel fullRobotModel)
   {
      constrainedCenterOfMassJacobianCalculator = new ConstrainedCenterOfMassJacobianCalculator(fullRobotModel.getRootJoint());

      momentumSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, SpatialMotionVector.SIZE);
      momentumSelectionMatrix.set(0, 3, 1.0);
      momentumSelectionMatrix.set(1, 4, 1.0);
      momentumSelectionMatrix.set(2, 5, 1.0);

//      DenseMatrix64F momentumSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
//      CommonOps.setIdentity(momentumSelectionMatrix);

      centerOfMassFrame = new CenterOfMassReferenceFrame("CoM", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      constrainedCentroidalMomentumMatrixCalculator = new ConstrainedCentroidalMomentumMatrixCalculator(fullRobotModel.getRootJoint(), centerOfMassFrame,
            momentumSelectionMatrix);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         constrainedCenterOfMassJacobianCalculator.addConstraint(foot, selectionMatrix);
         constrainedCentroidalMomentumMatrixCalculator.addConstraint(foot, selectionMatrix);
      }

      allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      v = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(allJoints), 1);

      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, SpatialMotionVector.SIZE);
      orientationSelectionMatrix.set(0, 0, 1.0);
      orientationSelectionMatrix.set(1, 1, 1.0);
      orientationSelectionMatrix.set(2, 2, 1.0);
      constrainedCenterOfMassJacobianCalculator.addConstraint(fullRobotModel.getPelvis(), orientationSelectionMatrix);
      constrainedCentroidalMomentumMatrixCalculator.addConstraint(fullRobotModel.getPelvis(), orientationSelectionMatrix);

      actuatedJoints = new ArrayList<InverseDynamicsJoint>();

      for (RobotSide robotSide : RobotSide.values)
      {
         InverseDynamicsJoint[] jointPath = ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getFoot(robotSide));
         actuatedJoints.addAll(Arrays.asList(jointPath));
      }

//    actuatedJoints.addAll(Arrays.asList(allJoints));
      actuatedJoints.remove(fullRobotModel.getRootJoint());

      for (InverseDynamicsJoint actuatedJoint : actuatedJoints)
      {
         constrainedCenterOfMassJacobianCalculator.addActuatedJoint(actuatedJoint);
         constrainedCentroidalMomentumMatrixCalculator.addActuatedJoint(actuatedJoint);
      }

      int nActuatedDoFs = ScrewTools.computeDegreesOfFreedom(actuatedJoints);
      vActuated = new DenseMatrix64F(nActuatedDoFs, 1);

      DampedLeastSquaresSolver columnSpaceProjectorSolver = new DampedLeastSquaresSolver(momentumSelectionMatrix.getNumRows());
      columnSpaceProjectorSolver.setAlpha(0.02);
//      LinearSolver<DenseMatrix64F> columnSpaceProjectorSolver = LinearSolverFactory.pseudoInverse(true);
      projector = new ColumnSpaceProjector(columnSpaceProjectorSolver, momentumSelectionMatrix.getNumRows(), nActuatedDoFs);

      centroidalLinearMomentum = new YoFrameVector("centroidalLinearMomentum", centerOfMassFrame, registry);
      centroidalAngularMomentum = new YoFrameVector("centroidalAngularMomentum", centerOfMassFrame, registry);

      centroidalLinearMomentumPlus = new YoFrameVector("centroidalLinearMomentumPlus", centerOfMassFrame, registry);
      centroidalAngularMomentumPlus = new YoFrameVector("centroidalAngularMomentumPlus", centerOfMassFrame, registry);

      centroidalLinearMomentum.setZ(10.0);
   }

   public void doControl()
   {
      centerOfMassFrame.update();

      constrainedCenterOfMassJacobianCalculator.compute();
      DenseMatrix64F centerOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getCenterOfMassJacobian();
      DenseMatrix64F constrainedCenterOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getConstrainedCenterOfMassJacobian();

      this.comJacobianConditionNumber.set(NormOps.conditionP2(centerOfMassJacobian));
      this.comJacobianSigmaMin.set(computeSmallestSingularValue(centerOfMassJacobian));
      this.constrainedComJacobianConditionNumber.set(NormOps.conditionP2(constrainedCenterOfMassJacobian));
      this.constrainedComJacobianSigmaMin.set(computeSmallestSingularValue(constrainedCenterOfMassJacobian));

      ScrewTools.getJointVelocitiesMatrix(allJoints, v);
      CommonOps.mult(centerOfMassJacobian, v, tempCoMVelocityMatrix);
      MatrixTools.denseMatrixToVector3d(tempCoMVelocityMatrix, tempCoMVelocity, 0, 0);
      comVelocity.set(tempCoMVelocity);

      ScrewTools.getJointVelocitiesMatrix(actuatedJoints, vActuated);

//    CommonOps.mult(constrainedCenterOfMassJacobian, vActuated, tempCoMVelocityMatrix);
//    MatrixTools.denseMatrixToVector3d(tempCoMVelocityMatrix, tempCoMVelocity, 0, 0);
//    constrainedComVelocity.set(tempCoMVelocity);

      constrainedCentroidalMomentumMatrixCalculator.compute();
      DenseMatrix64F centroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getCentroidalMomentumMatrix();
      DenseMatrix64F constrainedCentroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getConstrainedCentroidalMomentumMatrix();

      this.cmmConditionNumber.set(NormOps.conditionP2(centroidalMomentumMatrix));
      this.cmmSigmaMin.set(computeSmallestSingularValue(centroidalMomentumMatrix));
      this.constrainedCMMConditionNumber.set(NormOps.conditionP2(constrainedCentroidalMomentumMatrix));
      this.constrainedCMMSigmaMin.set(computeSmallestSingularValue(constrainedCentroidalMomentumMatrix));

      projector.setA(constrainedCentroidalMomentumMatrix);
      DenseMatrix64F centroidalMomentumPlusDenseMatrix = new DenseMatrix64F(momentumSelectionMatrix.getNumRows(), 1);
      Momentum centroidalMomentum = new Momentum(centerOfMassFrame, centroidalLinearMomentum.getFrameVectorCopy().getVector(),
                                       centroidalAngularMomentum.getFrameVectorCopy().getVector());
      DenseMatrix64F centroidalMomentumSelection = new DenseMatrix64F(momentumSelectionMatrix.getNumRows(), 1);
      CommonOps.mult(momentumSelectionMatrix, centroidalMomentum.toDenseMatrix(), centroidalMomentumSelection);
      projector.project(centroidalMomentumSelection, centroidalMomentumPlusDenseMatrix);


      //TODO: hack:
      Vector3d linearMomentumPlus = new Vector3d();
      MatrixTools.denseMatrixToVector3d(centroidalMomentumPlusDenseMatrix, linearMomentumPlus, 0, 0);
      centroidalLinearMomentumPlus.set(linearMomentumPlus);

//      Momentum centroidalMomentumPlus = new Momentum(centerOfMassFrame, centroidalMomentumPlusDenseMatrix);
//      centroidalLinearMomentumPlus.set(centroidalMomentumPlus.getLinearPartCopy());
//      centroidalAngularMomentumPlus.set(centroidalMomentumPlus.getAngularPartCopy());
   }

   public void initialize()
   {
      // empty
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   private static double computeSmallestSingularValue(DenseMatrix64F A)
   {
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(A.numRows, A.numCols, false, false, true);
      svd.decompose(A);
      double[] singularValues = svd.getSingularValues();

      return Doubles.min(singularValues);
   }
}
