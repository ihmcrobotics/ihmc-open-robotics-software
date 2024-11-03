package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.QRNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.SVDNullspaceCalculator;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;

import java.util.ArrayList;
import java.util.List;

public class WholeBodyContactNullspaceCalculator
{
   private static final boolean USE_SVD = false;
   private static final int LINEAR_DIMENSIONS = 3;
   private static final int SPATIAL_DIMENSIONS = 6;
   private static final boolean INCLUDE_COM_Z = false;

   private final JointBasics[] controlledJoints;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final DMatrixRMaj stackedContactJacobian = new DMatrixRMaj(0);
   private final SVDNullspaceCalculator svdNullspaceCalculator = new SVDNullspaceCalculator(40, false);
   private final QRNullspaceCalculator qrNullspaceCalculator = new QRNullspaceCalculator(40);
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);

   private final SideDependentList<GeometricJacobian> handJacobians = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> feetJacobians = new SideDependentList<>();
   private final PointJacobian pointJacobian = new PointJacobian();
   private final SideDependentList<FramePoint3DReadOnly> handControlPoints = new SideDependentList<>();
   private final List<OneDoFJointBasics> jointsToIgnore = new ArrayList<>();

   public WholeBodyContactNullspaceCalculator(JointBasics[] controlledJoints, ReferenceFrame centerOfMassFrame, FullHumanoidRobotModel fullRobotModel)
   {
      this.controlledJoints = controlledJoints;
      this.controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);

      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(multiBodySystemInput, centerOfMassFrame);

      for (int jointIdx = 0; jointIdx < controlledOneDoFJoints.length; jointIdx++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[jointIdx];
         jointToIndexMap.put(joint, jointIdx);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handJacobians.put(robotSide, new GeometricJacobian(fullRobotModel.getElevator(), fullRobotModel.getHand(robotSide), fullRobotModel.getHand(robotSide).getBodyFixedFrame()));
         feetJacobians.put(robotSide, new GeometricJacobian(fullRobotModel.getElevator(), fullRobotModel.getFoot(robotSide), fullRobotModel.getFoot(robotSide).getBodyFixedFrame()));
         handControlPoints.put(robotSide, new FramePoint3D(fullRobotModel.getHandControlFrame(robotSide)));
      }

      // TODO make a parameter

      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);
      OneDoFJointBasics wristYaw = fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_YAW);
      OneDoFJointBasics wristRoll = fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_ROLL);
      OneDoFJointBasics gripperYaw = fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.GRIPPER_YAW);

      if (spinePitch != null)
         jointsToIgnore.add(spinePitch);
      if (spineRoll != null)
         jointsToIgnore.add(spineRoll);
      if (wristYaw != null)
         jointsToIgnore.add(wristYaw);
      if (wristRoll != null)
         jointsToIgnore.add(wristRoll);
      if (gripperYaw != null)
         jointsToIgnore.add(gripperYaw);
   }

   public void compute()
   {
      /* Setup matrix based on constraint dimensionality */
      int numConstraints = 0;
      numConstraints += 2; // xy centroidal motion
      numConstraints += INCLUDE_COM_Z ? 1 : 0; // z centroidal motion
      numConstraints += 2 * LINEAR_DIMENSIONS; // hand linear position
      numConstraints += 2 * SPATIAL_DIMENSIONS; // foot spatial pose
      numConstraints += jointsToIgnore.size(); // joints to ignore

      stackedContactJacobian.reshape(numConstraints, SPATIAL_DIMENSIONS + controlledOneDoFJoints.length);
      stackedContactJacobian.zero();

      /* Compute and set centroidal mass momentum matrix */
      centroidalMomentumCalculator.reset();
      DMatrixRMaj centroidalMomentumMatrix = centroidalMomentumCalculator.getCentroidalMomentumMatrix();

      int centroidalMomentumRowOffset = 3;
      int centroidalMomentumNumRows = INCLUDE_COM_Z ? 3 : 2;
      int rowOffset = 0;

      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, centroidalMomentumMatrix, centroidalMomentumRowOffset, 0, centroidalMomentumNumRows, centroidalMomentumMatrix.getNumCols(), 1.0);
      rowOffset += centroidalMomentumNumRows;

      for (RobotSide robotSide : RobotSide.values)
      {
         /* Compute and set foot jacobian */
         GeometricJacobian footJacobian = feetJacobians.get(robotSide);
         footJacobian.compute();
         rowOffset = stackJacobian(rowOffset, footJacobian.getJacobianMatrix(), footJacobian.getJointsInOrder());

         /* Compute and set hand jacobian */
         GeometricJacobian handJacobian = handJacobians.get(robotSide);
         handJacobian.changeFrame(handJacobian.getBaseFrame());
         handJacobian.compute();
         pointJacobian.set(handJacobian, handControlPoints.get(robotSide));
         pointJacobian.compute();
         rowOffset = stackJacobian(rowOffset, pointJacobian.getJacobianMatrix(), handJacobian.getJointsInOrder());
      }

      for (int i = 0; i < jointsToIgnore.size(); i++)
      {
         stackedContactJacobian.set(rowOffset, getSystemJacobianColumn(jointsToIgnore.get(i)), 1.0);
         rowOffset++;
      }

      if (USE_SVD)
      {
         svdNullspaceCalculator.setMatrix(stackedContactJacobian, stackedContactJacobian.getNumCols() - stackedContactJacobian.getNumRows());
      }
      else
      {
         qrNullspaceCalculator.computeNullspaceProjector(stackedContactJacobian, null);
      }

//      nullspaceCalculator.projectOntoNullspace(matrixToProjectOntoNullspace, stackedContactJacobian, projectedMatrixToPack);
//      qrNullspaceCalculator.projectOntoNullspace(matrixToProjectOntoNullspace, stackedContactJacobian, projectedMatrixToPack);
   }

   public DMatrixRMaj getNullspace()
   {
      if (USE_SVD)
      {
         return svdNullspaceCalculator.getNullspace();
      }
      else
      {
         return qrNullspaceCalculator.getNullspace();
      }
   }

   public NullspaceCalculator getSvdNullspaceCalculator()
   {
      return svdNullspaceCalculator;
   }

   public void project(DMatrixRMaj velocityToProject, DMatrixRMaj projectedVelocity)
   {
      if (USE_SVD)
      {
         svdNullspaceCalculator.projectOntoNullspace(velocityToProject, stackedContactJacobian, projectedVelocity);
      }
      else
      {
         qrNullspaceCalculator.projectOntoNullspace(velocityToProject, stackedContactJacobian, projectedVelocity);
      }
   }

   private int stackJacobian(int rowOffset, DMatrixRMaj jacobianMatrix, JointBasics[] joints)
   {
      int numRows = jacobianMatrix.getNumRows();

      // floating root joint is zero index in all cases
      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, jacobianMatrix, 0, 0, numRows, SPATIAL_DIMENSIONS, 1.0);

      // copy joint entries using the index map
      for (int jointIdx = 1; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = (OneDoFJointBasics) joints[jointIdx];
         int systemColumnIndex = getSystemJacobianColumn(joint);
         int jacobianColumn = SPATIAL_DIMENSIONS + jointIdx - 1;

         for (int rowIdx = 0; rowIdx < numRows; rowIdx++)
         {
            double jacobianEntry = jacobianMatrix.get(rowIdx, jacobianColumn);
            stackedContactJacobian.set(rowOffset + rowIdx, systemColumnIndex, jacobianEntry);
         }
      }

      return rowOffset + numRows;
   }

   private int getSystemJacobianColumn(OneDoFJointBasics joint)
   {
      return SPATIAL_DIMENSIONS + jointToIndexMap.get(joint);
   }

   public void addJointToIgnore(OneDoFJointBasics jointToIgnore)
   {
      if (!jointsToIgnore.contains(jointToIgnore))
      {
         jointsToIgnore.add(jointToIgnore);
      }
   }

   public void clearJointsToIgnore()
   {
      jointsToIgnore.clear();
   }
}
