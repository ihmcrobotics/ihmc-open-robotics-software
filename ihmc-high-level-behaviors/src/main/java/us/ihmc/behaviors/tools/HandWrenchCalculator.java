package us.ihmc.behaviors.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HandWrenchCalculator
{
   private FullHumanoidRobotModel fullRobotModel;
   HumanoidJointNameMap jointNameMap;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private SideDependentList<DMatrixRMaj> armJacobianMatrix = new SideDependentList<>();
   private SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> referenceFrame = new SideDependentList<>();
   private SideDependentList<Wrench> wrenches = new SideDependentList<>(new Wrench(), new Wrench());

   public HandWrenchCalculator(ROS2SyncedRobotModel syncedRobot)
   {
      fullRobotModel = syncedRobot.getFullRobotModel();
      jointNameMap = syncedRobot.getRobotModel().getJointMap();
   }

   private void updateJacobians()
   {
      OneDoFJointBasics[] joints = new OneDoFJointBasics[jointNameMap.getArmJointNames().length];
      for (RobotSide side : RobotSide.values)
      {
         // TODO: use jacobianCalculator.getJointsFromBasedToEndEffector()
         for (int i = 0; i < jointNameMap.getArmJointNames().length; ++i)
         {
            ArmJointName jointName = jointNameMap.getArmJointNames()[i];
//            LogTools.info(jointName.toString());
            joints[i] = fullRobotModel.getArmJoint(side, jointName);
         }

         jacobianCalculator.setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(side));
         referenceFrame.set(side, jacobianCalculator.getJacobianFrame());
         armJacobianMatrix.set(side, jacobianCalculator.getJacobianMatrix());
         armJoints.set(side, joints);
      }
   }

   private DMatrixRMaj leftPseudoInverse(DMatrixRMaj matrix)
   {
      // PREVIOUSLY . . . >>>
      /*
      double lambda = 1e-6;

      DMatrixRMaj matrixTransposed = CommonOps_DDRM.transpose(matrix, null);
      int numRow = matrixTransposed.getNumRows();

      DMatrixRMaj AT_A = new DMatrixRMaj(numRow, numRow);
      CommonOps_DDRM.mult(matrixTransposed, matrix, AT_A);

      DMatrixRMaj identity = CommonOps_DDRM.identity(numRow);
      CommonOps_DDRM.scale(lambda, identity);

      DMatrixRMaj result = new DMatrixRMaj(identity.getNumRows(),identity.getNumRows());
      CommonOps_DDRM.add(AT_A, identity, result);

      CommonOps_DDRM.invert(result);

      DMatrixRMaj matrixDagger = new DMatrixRMaj(result.getNumRows(),matrixTransposed.getNumCols());
      CommonOps_DDRM.mult(result, matrixTransposed, matrixDagger);

      return matrixDagger;

       */
      // <<< PREVIOUSLY . . .

      // TODO: check this works the same
      DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(matrix.getNumRows(), 1e-6);
      pseudoInverseSolver.setA(matrix);
      DMatrixRMaj leftPseudoInverseOfMatrix = new DMatrixRMaj(matrix);
      pseudoInverseSolver.invert(leftPseudoInverseOfMatrix);
      return leftPseudoInverseOfMatrix;
   }

   private void calculateTaskForces()
   {
      for (RobotSide side : RobotSide.values)
      {
         double[] jointTorques = new double[armJoints.get(side).length];
         for (int i = 0; i < armJoints.get(side).length; ++i)
         {
            jointTorques[i] = armJoints.get(side)[i].getTau();
         }

         DMatrixRMaj armJacobian = armJacobianMatrix.get(side);
         DMatrixRMaj armJacobianTransposed = CommonOps_DDRM.transpose(armJacobian, null);
         DMatrixRMaj armJacobianTransposedDagger = leftPseudoInverse(armJacobianTransposed);
         DMatrixRMaj jointTorqueVector = new DMatrixRMaj(jointTorques);
         DMatrixRMaj wrenchVector = new DMatrixRMaj(6,1);
         CommonOps_DDRM.mult(armJacobianTransposedDagger, jointTorqueVector, wrenchVector);

         wrenches.set(side, makeWrench(getReferenceFrame().get(side), wrenchVector));
      }
   }

   // Wrench expressed in world-aligned frame
   private static Wrench makeWrench(ReferenceFrame wrenchFrame, DMatrixRMaj wrenchVector)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Linear and angular part into spatial vector
      SpatialVector spatialVector = new SpatialVector();
      spatialVector.set(wrenchVector);
      // Express in world-frame
      spatialVector.changeFrame(worldFrame);

      Wrench wrench = new Wrench();
      wrench.setBodyFrame(wrenchFrame);
      wrench.setReferenceFrame(worldFrame);
      wrench.set(spatialVector);

      return wrench;
   }

   public void update()
   {
      updateJacobians();
      calculateTaskForces();
   }

   public SideDependentList<Wrench> getWrench()
   {
      return wrenches;
   }

   public SideDependentList<ReferenceFrame> getReferenceFrame()
   {
      return referenceFrame;
   }

   // expressed in world-aligned frame
   public FrameVector3D getWrenchLinear(RobotSide side)
   {
      return new FrameVector3D(wrenches.get(side).getLinearPart().getReferenceFrame(), wrenches.get(side).getLinearPart());
   }

   // expressed in world-aligned frame
   public FrameVector3D getWrenchAngular(RobotSide side)
   {
      return new FrameVector3D(wrenches.get(side).getAngularPart().getReferenceFrame(), wrenches.get(side).getAngularPart());
   }
}