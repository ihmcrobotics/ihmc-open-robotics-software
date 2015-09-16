package us.ihmc.atlas.hikSim;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTest;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTestHelper;

@DeployableTestClass(targets = {TestPlanTarget.InDevelopment, TestPlanTarget.Slow})
public class AtlasWholeBodyTrajectoryTest extends WholeBodyTrajectoryTest
{
   @Override 
   public WholeBodyTrajectoryTestHelper getWholeBodyTrajectoryTestHelper()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);

      SDFFullHumanoidRobotModel actualRobotModel =  atlasRobotModel.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW)).setQ(0.500); //arm_shy
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL)).setQ(robotSide.negateIfRightSide(-1.0)); //arm_shx
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_PITCH)).setQ(2.00); //arm_ely
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.ELBOW_ROLL)).setQ(robotSide.negateIfRightSide(0.6)); //arm_elx
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH)).setQ(0.000); //arm_wry
         actualRobotModel.getOneDoFJointByName(atlasRobotModel.getJointMap().getArmJointName(robotSide, ArmJointName.WRIST_ROLL)).setQ(robotSide.negateIfRightSide(0)); //arm_wrx
      }
      
      WholeBodyIkSolver wholeBodyIkSolver = new AtlasWholeBodyIK(atlasRobotModel);
      Robot robot = atlasRobotModel.createSdfRobot(false);
      
      WholeBodyTrajectoryTestHelper wholeBodyTrajectoryTestHelper = new WholeBodyTrajectoryTestHelper(atlasRobotModel, robot, actualRobotModel, wholeBodyIkSolver);
      return wholeBodyTrajectoryTestHelper;
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 2.8)
   @Test(timeout = 30000)
   public void testPointToPointLeft() throws Exception
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPointToPointLeft();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 1.3)
   @Test(timeout = 30000)
   public void testPointToPointRight() throws Exception
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPointToPointRight();
   }
   
   @Override
   @DeployableTestMethod(estimatedDuration = 4.1)
   @Test(timeout = 22445)
   public void testTrajectory() throws Exception
   {
      TestPlanTarget.assumeRunningLocally();
      super.testTrajectory();
   }
}
