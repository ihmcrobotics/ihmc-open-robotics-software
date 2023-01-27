package us.ihmc.behaviors.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.directionalControlToolboxModule.RobotModelUpdater;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class HandWrenchCalculator
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
   private SideDependentList<ReferenceFrame> referenceFrame = new SideDependentList<>();

   private SideDependentList<DMatrixRMaj> wrench = new SideDependentList<>();
   private SideDependentList<FrameVector3D> wrenchLinear = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());
   private SideDependentList<FrameVector3D> wrenchAngular = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());

   public HandWrenchCalculator(ROS2SyncedRobotModel syncedRobot)
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
         DMatrixRMaj armJacobTransposedDagger = leftPseudoInverse(armJacobTransposed);
         DMatrixRMaj torqueVector = new DMatrixRMaj(torques);
         DMatrixRMaj forceVector = new DMatrixRMaj(6,1);

         CommonOps_DDRM.mult(armJacobTransposedDagger, torqueVector, forceVector);
         wrench.set(side, forceVector);
         saveLinearAngularInWorld(side);
      }
   }

   private void saveLinearAngularInWorld(RobotSide side)
   {
      // LINEAR PART
      FrameVector3D linearPart = wrenchLinear.get(side);
      linearPart.setReferenceFrame(getReferenceFrame().get(side));
      linearPart.set(0,0,getWrench().get(side));
      linearPart.changeFrame(ReferenceFrame.getWorldFrame());
      wrenchLinear.set(side,linearPart);

      // ANGULAR PART
      FrameVector3D angularPart = wrenchLinear.get(side);
      angularPart.setReferenceFrame(getReferenceFrame().get(side));
      angularPart.set(3,0,getWrench().get(side));
      angularPart.changeFrame(ReferenceFrame.getWorldFrame());
      wrenchLinear.set(side,angularPart);
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

   public SideDependentList<ReferenceFrame> getReferenceFrame()
   {
      return referenceFrame;
   }

   // Wrench expressed in world-aligned frame
   public SideDependentList<FrameVector3D> getWrenchLinear()
   {
      return wrenchLinear;
   }

   // Wrench expressed in world-aligned frame
   public SideDependentList<FrameVector3D> getWrenchAngular()
   {
      return wrenchAngular;
   }
}