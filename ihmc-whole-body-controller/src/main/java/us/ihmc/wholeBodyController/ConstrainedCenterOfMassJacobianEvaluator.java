package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import com.google.common.primitives.Doubles;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.linearAlgebra.ColumnSpaceProjector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ConstrainedCenterOfMassJacobianCalculator;
import us.ihmc.robotics.screwTheory.ConstrainedCentroidalMomentumMatrixCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

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

   private final YoDouble comJacobianConditionNumber = new YoDouble("comJacCondition", registry);
   private final YoDouble comJacobianSigmaMin = new YoDouble("comJacobianSigmaMin", registry);
   private final YoDouble constrainedComJacobianConditionNumber = new YoDouble("constrComJacCondition", registry);
   private final YoDouble constrainedComJacobianSigmaMin = new YoDouble("constrComJacobianSigmaMin", registry);
   private final YoFrameVector3D comVelocity = new YoFrameVector3D("comJacComVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D constrainedComVelocity = new YoFrameVector3D("constrComJacComVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble cmmConditionNumber = new YoDouble("CMMCondition", registry);
   private final YoDouble cmmSigmaMin = new YoDouble("CMMSigmaMin", registry);
   private final YoDouble constrainedCMMConditionNumber = new YoDouble("constrCMMCondition", registry);
   private final YoDouble constrainedCMMSigmaMin = new YoDouble("constrCMMSigmaMin", registry);

   private final JointBasics[] allJoints;
   private final DenseMatrix64F v;
   private final DenseMatrix64F vActuated;
   private final DenseMatrix64F tempCoMVelocityMatrix = new DenseMatrix64F(3, 1);
   private final Vector3D tempCoMVelocity = new Vector3D();
   private final List<JointBasics> actuatedJoints;
   private final ColumnSpaceProjector projector;

   private final YoFrameVector3D centroidalLinearMomentum;
   private final YoFrameVector3D centroidalAngularMomentum;

   private final YoFrameVector3D centroidalLinearMomentumPlus;
   private final YoFrameVector3D centroidalAngularMomentumPlus;
   private final DenseMatrix64F momentumSelectionMatrix;

   public ConstrainedCenterOfMassJacobianEvaluator(FullHumanoidRobotModel fullRobotModel)
   {
      constrainedCenterOfMassJacobianCalculator = new ConstrainedCenterOfMassJacobianCalculator(fullRobotModel.getRootJoint());

      momentumSelectionMatrix = new DenseMatrix64F(SpatialVector.SIZE / 2, SpatialVector.SIZE);
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
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialVector.SIZE, SpatialVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         constrainedCenterOfMassJacobianCalculator.addConstraint(foot, selectionMatrix);
         constrainedCentroidalMomentumMatrixCalculator.addConstraint(foot, selectionMatrix);
      }

      allJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      v = new DenseMatrix64F(MultiBodySystemTools.computeDegreesOfFreedom(allJoints), 1);

      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(SpatialVector.SIZE / 2, SpatialVector.SIZE);
      orientationSelectionMatrix.set(0, 0, 1.0);
      orientationSelectionMatrix.set(1, 1, 1.0);
      orientationSelectionMatrix.set(2, 2, 1.0);
      constrainedCenterOfMassJacobianCalculator.addConstraint(fullRobotModel.getPelvis(), orientationSelectionMatrix);
      constrainedCentroidalMomentumMatrixCalculator.addConstraint(fullRobotModel.getPelvis(), orientationSelectionMatrix);

      actuatedJoints = new ArrayList<JointBasics>();

      for (RobotSide robotSide : RobotSide.values)
      {
         JointBasics[] jointPath = MultiBodySystemTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getFoot(robotSide));
         actuatedJoints.addAll(Arrays.asList(jointPath));
      }

//    actuatedJoints.addAll(Arrays.asList(allJoints));
      actuatedJoints.remove(fullRobotModel.getRootJoint());

      for (JointBasics actuatedJoint : actuatedJoints)
      {
         constrainedCenterOfMassJacobianCalculator.addActuatedJoint(actuatedJoint);
         constrainedCentroidalMomentumMatrixCalculator.addActuatedJoint(actuatedJoint);
      }

      int nActuatedDoFs = MultiBodySystemTools.computeDegreesOfFreedom(actuatedJoints);
      vActuated = new DenseMatrix64F(nActuatedDoFs, 1);

      DampedLeastSquaresSolver columnSpaceProjectorSolver = new DampedLeastSquaresSolver(momentumSelectionMatrix.getNumRows());
      columnSpaceProjectorSolver.setAlpha(0.02);
//      LinearSolver<DenseMatrix64F> columnSpaceProjectorSolver = LinearSolverFactory.pseudoInverse(true);
      projector = new ColumnSpaceProjector(columnSpaceProjectorSolver, momentumSelectionMatrix.getNumRows(), nActuatedDoFs);

      centroidalLinearMomentum = new YoFrameVector3D("centroidalLinearMomentum", centerOfMassFrame, registry);
      centroidalAngularMomentum = new YoFrameVector3D("centroidalAngularMomentum", centerOfMassFrame, registry);

      centroidalLinearMomentumPlus = new YoFrameVector3D("centroidalLinearMomentumPlus", centerOfMassFrame, registry);
      centroidalAngularMomentumPlus = new YoFrameVector3D("centroidalAngularMomentumPlus", centerOfMassFrame, registry);

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

      MultiBodySystemTools.extractJointsState(allJoints, JointStateType.VELOCITY, v);
      CommonOps.mult(centerOfMassJacobian, v, tempCoMVelocityMatrix);
      tempCoMVelocity.set(tempCoMVelocityMatrix);
      comVelocity.set(tempCoMVelocity);

      MultiBodySystemTools.extractJointsState(actuatedJoints, JointStateType.VELOCITY, vActuated);

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
      Momentum centroidalMomentum = new Momentum(centerOfMassFrame, centroidalAngularMomentum,
                                       centroidalLinearMomentum);
      DenseMatrix64F centroidalMomentumSelection = new DenseMatrix64F(momentumSelectionMatrix.getNumRows(), 1);
      DenseMatrix64F centroidalMomentumMatrix2 = new DenseMatrix64F(6, 1);
      centroidalMomentum.get(centroidalMomentumMatrix2);
      CommonOps.mult(momentumSelectionMatrix, centroidalMomentumMatrix2, centroidalMomentumSelection);
      projector.project(centroidalMomentumSelection, centroidalMomentumPlusDenseMatrix);


      //TODO: hack:
      Vector3D linearMomentumPlus = new Vector3D();
      linearMomentumPlus.set(centroidalMomentumPlusDenseMatrix);
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
