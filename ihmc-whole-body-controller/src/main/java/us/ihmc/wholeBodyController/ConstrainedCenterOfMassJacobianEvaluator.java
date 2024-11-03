package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianEvaluator implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
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
   private final DMatrixRMaj v;
   private final DMatrixRMaj vActuated;
   private final DMatrixRMaj tempCoMVelocityMatrix = new DMatrixRMaj(3, 1);
   private final Vector3D tempCoMVelocity = new Vector3D();
   private final List<JointBasics> actuatedJoints;
   private final ColumnSpaceProjector projector;

   private final YoFrameVector3D centroidalLinearMomentum;
   private final YoFrameVector3D centroidalAngularMomentum;

   private final YoFrameVector3D centroidalLinearMomentumPlus;
   private final YoFrameVector3D centroidalAngularMomentumPlus;
   private final DMatrixRMaj momentumSelectionMatrix;

   public ConstrainedCenterOfMassJacobianEvaluator(FullHumanoidRobotModel fullRobotModel)
   {
      constrainedCenterOfMassJacobianCalculator = new ConstrainedCenterOfMassJacobianCalculator(fullRobotModel.getRootJoint());

      momentumSelectionMatrix = new DMatrixRMaj(SpatialVector.SIZE / 2, SpatialVector.SIZE);
      momentumSelectionMatrix.set(0, 3, 1.0);
      momentumSelectionMatrix.set(1, 4, 1.0);
      momentumSelectionMatrix.set(2, 5, 1.0);

//      DMatrixRMaj momentumSelectionMatrix = new DMatrixRMaj(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
//      CommonOps_DDRM.setIdentity(momentumSelectionMatrix);

      centerOfMassFrame = new CenterOfMassReferenceFrame("CoM", ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      constrainedCentroidalMomentumMatrixCalculator = new ConstrainedCentroidalMomentumMatrixCalculator(fullRobotModel.getRootJoint(), centerOfMassFrame,
            momentumSelectionMatrix);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         DMatrixRMaj selectionMatrix = new DMatrixRMaj(SpatialVector.SIZE, SpatialVector.SIZE);
         CommonOps_DDRM.setIdentity(selectionMatrix);
         constrainedCenterOfMassJacobianCalculator.addConstraint(foot, selectionMatrix);
         constrainedCentroidalMomentumMatrixCalculator.addConstraint(foot, selectionMatrix);
      }

      allJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      v = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(allJoints), 1);

      DMatrixRMaj orientationSelectionMatrix = new DMatrixRMaj(SpatialVector.SIZE / 2, SpatialVector.SIZE);
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
      vActuated = new DMatrixRMaj(nActuatedDoFs, 1);

      DampedLeastSquaresSolver columnSpaceProjectorSolver = new DampedLeastSquaresSolver(momentumSelectionMatrix.getNumRows());
      columnSpaceProjectorSolver.setAlpha(0.02);
//      LinearSolverDense<DMatrixRMaj> columnSpaceProjectorSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
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
      DMatrixRMaj centerOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getCenterOfMassJacobian();
      DMatrixRMaj constrainedCenterOfMassJacobian = constrainedCenterOfMassJacobianCalculator.getConstrainedCenterOfMassJacobian();

      this.comJacobianConditionNumber.set(NormOps_DDRM.conditionP2(centerOfMassJacobian));
      this.comJacobianSigmaMin.set(computeSmallestSingularValue(centerOfMassJacobian));
      this.constrainedComJacobianConditionNumber.set(NormOps_DDRM.conditionP2(constrainedCenterOfMassJacobian));
      this.constrainedComJacobianSigmaMin.set(computeSmallestSingularValue(constrainedCenterOfMassJacobian));

      MultiBodySystemTools.extractJointsState(allJoints, JointStateType.VELOCITY, v);
      CommonOps_DDRM.mult(centerOfMassJacobian, v, tempCoMVelocityMatrix);
      tempCoMVelocity.set(tempCoMVelocityMatrix);
      comVelocity.set(tempCoMVelocity);

      MultiBodySystemTools.extractJointsState(actuatedJoints, JointStateType.VELOCITY, vActuated);

//    CommonOps_DDRM.mult(constrainedCenterOfMassJacobian, vActuated, tempCoMVelocityMatrix);
//    MatrixTools.denseMatrixToVector3d(tempCoMVelocityMatrix, tempCoMVelocity, 0, 0);
//    constrainedComVelocity.set(tempCoMVelocity);

      constrainedCentroidalMomentumMatrixCalculator.compute();
      DMatrixRMaj centroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getCentroidalMomentumMatrix();
      DMatrixRMaj constrainedCentroidalMomentumMatrix = constrainedCentroidalMomentumMatrixCalculator.getConstrainedCentroidalMomentumMatrix();

      this.cmmConditionNumber.set(NormOps_DDRM.conditionP2(centroidalMomentumMatrix));
      this.cmmSigmaMin.set(computeSmallestSingularValue(centroidalMomentumMatrix));
      this.constrainedCMMConditionNumber.set(NormOps_DDRM.conditionP2(constrainedCentroidalMomentumMatrix));
      this.constrainedCMMSigmaMin.set(computeSmallestSingularValue(constrainedCentroidalMomentumMatrix));

      projector.setA(constrainedCentroidalMomentumMatrix);
      DMatrixRMaj centroidalMomentumPlusDenseMatrix = new DMatrixRMaj(momentumSelectionMatrix.getNumRows(), 1);
      Momentum centroidalMomentum = new Momentum(centerOfMassFrame, centroidalAngularMomentum,
                                       centroidalLinearMomentum);
      DMatrixRMaj centroidalMomentumSelection = new DMatrixRMaj(momentumSelectionMatrix.getNumRows(), 1);
      DMatrixRMaj centroidalMomentumMatrix2 = new DMatrixRMaj(6, 1);
      centroidalMomentum.get(centroidalMomentumMatrix2);
      CommonOps_DDRM.mult(momentumSelectionMatrix, centroidalMomentumMatrix2, centroidalMomentumSelection);
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

   public YoRegistry getYoRegistry()
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

   private static double computeSmallestSingularValue(DMatrixRMaj A)
   {
      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(A.numRows, A.numCols, false, false, true);
      svd.decompose(A);
      double[] singularValues = svd.getSingularValues();

      return Doubles.min(singularValues);
   }
}
