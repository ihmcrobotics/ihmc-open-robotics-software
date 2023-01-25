package us.ihmc.behaviors.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.directionalControlToolboxModule.RobotModelUpdater;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class ForceWrenchCalculator
{
   private final OneDoFJointBasics[] allJoints;
   private FullHumanoidRobotModel fullRobotModel;
   private final int jointNameHash;
   private final ROS2SyncedRobotModel syncedRobot;
   HumanoidJointNameMap jointNameMap;
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private SideDependentList<DMatrixRMaj> armJacobianMatrix = new SideDependentList<>();
   private SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();
   private SideDependentList<ArrayList<Double>> armTorques = new SideDependentList<>();

   private SideDependentList<DMatrixRMaj> wrench = new SideDependentList<>();

   public ForceWrenchCalculator(ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      fullRobotModel = syncedRobot.getFullRobotModel();
      jointNameMap = syncedRobot.getRobotModel().getJointMap();
      allJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = RobotModelUpdater.calculateJointNameHash(allJoints, fullRobotModel.getForceSensorDefinitions(), fullRobotModel.getIMUDefinitions());

   }

   private void updateJacobians()
   {
      OneDoFJointBasics[] joints = new OneDoFJointBasics[jointNameMap.getArmJointNames().length];
      for (RobotSide side : RobotSide.values)
      {
         for (int i = 0; i < jointNameMap.getArmJointNames().length; ++i)
         {
            ArmJointName jointName = jointNameMap.getArmJointNames()[i];
            LogTools.info(jointName.toString());
            joints[i] = fullRobotModel.getArmJoint(side, jointName);
         }
//         jacobianCalculator.setKinematicChain(joints);
         jacobianCalculator.setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(side));
         armJacobianMatrix.set(side, jacobianCalculator.getJacobianMatrix());
         armJoints.set(side, joints);
      }
   }

   private DMatrixRMaj pseudoInverse(DMatrixRMaj matrix)
   {
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
   }

   private void calculateTaskForces()
   {
      for (RobotSide side : RobotSide.values)
      {
         double[] torques = new double[armJoints.get(side).length];
         for (int i = 0; i < armJoints.get(side).length; ++i)
         {
            torques[i] = armJoints.get(side)[i].getTau();
         }

         DMatrixRMaj armJacob = armJacobianMatrix.get(side);
         DMatrixRMaj armJacobTransposed = CommonOps_DDRM.transpose(armJacob, null);
         DMatrixRMaj armJacobTransposedDagger = pseudoInverse(armJacobTransposed);
         DMatrixRMaj torqueVector = new DMatrixRMaj(torques);
         DMatrixRMaj forceVector = new DMatrixRMaj(6,1);
//         LogTools.info("armJacobTransposedDagger shape: {} x {}", armJacobTransposedDagger.getNumRows(), armJacobTransposedDagger.getNumCols());
//         LogTools.info("torqueVector shape: {} x {}",torqueVector.getNumRows(), torqueVector.getNumCols());
         CommonOps_DDRM.mult(armJacobTransposedDagger, torqueVector, forceVector);
         wrench.set(side, forceVector);
      }
   }

   public void update()
   {
      updateJacobians();
      calculateTaskForces();
   }

   public SideDependentList<DMatrixRMaj> getWrench()
   {
      return wrench;
   }
}