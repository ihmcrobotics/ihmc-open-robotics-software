package us.ihmc.atlas.hikSim;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIK;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTest;
import us.ihmc.wholeBodyController.WholeBodyTrajectoryTestHelper;

@BambooPlan(planType = {BambooPlanType.InDevelopment})
public class AtlasWholeBodyTrajectoryTest extends WholeBodyTrajectoryTest
{
   @Override 
   public WholeBodyTrajectoryTestHelper getWholeBodyTrajectoryTestHelper()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);

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

}
