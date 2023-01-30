package us.ihmc.behaviors.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.directionalControlToolboxModule.RobotModelUpdater;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.awt.*;
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

   private SideDependentList<Wrench> wrenches = new SideDependentList<>(new Wrench(), new Wrench());

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
         makeWrench(side, forceVector);
      }
   }

   private void makeWrench(RobotSide side, DMatrixRMaj wrenchVector)
   {
      ReferenceFrame wrenchFrame = getReferenceFrame().get(side);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // LINEAR PART
      FrameVector3D linearPart = new FrameVector3D();
      linearPart.setReferenceFrame(wrenchFrame);
      linearPart.set(0,0, wrenchVector);

      // ANGULAR PART
      FrameVector3D angularPart = new FrameVector3D();
      angularPart.setReferenceFrame(wrenchFrame);
      angularPart.set(3,0,wrenchVector);

      Wrench wrench = wrenches.get(side);
      wrench.setBodyFrame(wrenchFrame);
      wrench.setReferenceFrame(wrenchFrame);

      wrench.set(wrenchFrame, wrenchFrame, linearPart, angularPart);
      // Express in world frame
      wrench.changeFrame(worldFrame);
      wrenches.set(side, wrench);
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