package us.ihmc.avatar.inverseKinematics;

import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.TestInfo;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.MemoryTools;

public abstract class ArmIKSolverTest
{
   private static final double PRECISION = 5.0e-1;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testArmIKForAGivenSelectionMatrix(TestInfo testInfo) throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      Random r = new Random();

      // Forward Kinematics
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0).initializeFullRobotModel(fullRobotModel);
      ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(RobotSide.LEFT);
      for (ArmJointName armJointName : armJointNames)
      {
         OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(RobotSide.LEFT, armJointName);
         double lowerJointLimit = fullRobotModel.getOneDoFJointByName(armJoint.getName()).getJointLimitLower();
         double upperJointLimit = fullRobotModel.getOneDoFJointByName(armJoint.getName()).getJointLimitUpper();
         double randomArmJointPosition = lowerJointLimit + (upperJointLimit-lowerJointLimit)*r.nextDouble();
         armJoint.setQ(randomArmJointPosition);
      }
      fullRobotModel.updateFrames();

      FramePose3D taskspacePose = new FramePose3D(fullRobotModel.getHandControlFrame(RobotSide.LEFT));
      taskspacePose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());

      // Inverse Kinematics
      ArmIKSolver armIKSolver = new ArmIKSolver(RobotSide.LEFT, robotModel, fullRobotModel);

      armIKSolver.copySourceToWork();
      armIKSolver.update(fullRobotModel.getChest().getBodyFixedFrame(), taskspacePose.getReferenceFrame());
      armIKSolver.solve();

      for (int i = 0; i < armJointNames.length; i++)
      {
         OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(RobotSide.LEFT, armJointNames[i]);
         assertEquals(armJoint.getQ(), armIKSolver.getSolutionOneDoFJoints()[i].getQ(), PRECISION, "Joint position didnt match");
      }
   }

   public abstract DRCRobotModel getRobotModel();
}
