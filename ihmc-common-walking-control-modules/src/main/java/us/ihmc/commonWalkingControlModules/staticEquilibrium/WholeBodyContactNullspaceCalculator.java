package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class WholeBodyContactNullspaceCalculator
{
   private static final int LINEAR_DIMENSIONS = 3;
   private static final int SPATIAL_DIMENSIONS = 6;

   private final JointBasics[] controlledJoints;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final DMatrixRMaj stackedContactJacobian = new DMatrixRMaj(0);
   private final SVDNullspaceCalculator nullspaceCalculator = new SVDNullspaceCalculator(40, false);
   private final QRNullspaceCalculator qrNullspaceCalculator = new QRNullspaceCalculator(40);
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);

   private final SideDependentList<GeometricJacobian> handJacobians = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> feetJacobians = new SideDependentList<>();

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
      }

      int numConstraints = 0;
      numConstraints += 2; // xy centroidal motion
      numConstraints += 2 * LINEAR_DIMENSIONS; // hand linear position
      numConstraints += 2 * SPATIAL_DIMENSIONS; // foot spatial pose
      stackedContactJacobian.reshape(numConstraints, SPATIAL_DIMENSIONS + controlledOneDoFJoints.length);
   }

   public void project(DMatrixRMaj matrixToProjectOntoNullspace, DMatrixRMaj projectedMatrixToPack)
   {
      centroidalMomentumCalculator.reset();
      DMatrixRMaj centroidalMomentumMatrix = centroidalMomentumCalculator.getCentroidalMomentumMatrix();

      int centroidalMomentumRowOffset = 3;
      int centroidalMomentumNumRows = 2;
      int rowOffset = 0;

      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, centroidalMomentumMatrix, centroidalMomentumRowOffset, 0, centroidalMomentumNumRows, centroidalMomentumMatrix.getNumCols(), 1.0);
      rowOffset += centroidalMomentumNumRows;

      for (RobotSide robotSide : RobotSide.values)
      {
         rowOffset = stackJacobian(rowOffset, feetJacobians.get(robotSide), false);
         rowOffset = stackJacobian(rowOffset, handJacobians.get(robotSide), true);
      }

//      nullspaceCalculator.setMatrix(stackedContactJacobian, stackedContactJacobian.getNumCols() - stackedContactJacobian.getNumRows());
//      nullspaceCalculator.projectOntoNullspace(matrixToProjectOntoNullspace, stackedContactJacobian, projectedMatrixToPack);
      qrNullspaceCalculator.projectOntoNullspace(matrixToProjectOntoNullspace, stackedContactJacobian, projectedMatrixToPack);
   }

   public DMatrixRMaj getNullspace()
   {
      return nullspaceCalculator.getNullspace();
   }

   public NullspaceCalculator getNullspaceCalculator()
   {
      return nullspaceCalculator;
   }

   private int stackJacobian(int rowOffset, GeometricJacobian jacobian, boolean linearOnly)
   {
      jacobian.compute();

      JointBasics[] joints = jacobian.getJointsInOrder();
      DMatrixRMaj jacobianMatrix = jacobian.getJacobianMatrix();
      int srcStartRow = linearOnly ? 3 : 0;
      int srcNumRows = linearOnly ? 3 : 6;

      // floating root joint is zero index in all cases
      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, jacobianMatrix, srcStartRow, 0, srcNumRows, SPATIAL_DIMENSIONS, 1.0);

      // copy joint entries using the index map
      for (int jointIdx = 1; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = (OneDoFJointBasics) joints[jointIdx];
         int systemColumnIndex = getSystemJacobianColumn(joint);
         int jacobianColumn = SPATIAL_DIMENSIONS + jointIdx - 1;

         for (int rowIdx = 0; rowIdx < srcNumRows; rowIdx++)
         {
            double jacobianEntry = jacobianMatrix.get(srcStartRow + rowIdx, jacobianColumn);
            stackedContactJacobian.set(rowOffset + rowIdx, systemColumnIndex, jacobianEntry);
         }
      }

      return rowOffset + srcNumRows;
   }

   private int getSystemJacobianColumn(OneDoFJointBasics joint)
   {
      return SPATIAL_DIMENSIONS + jointToIndexMap.get(joint);
   }
}
