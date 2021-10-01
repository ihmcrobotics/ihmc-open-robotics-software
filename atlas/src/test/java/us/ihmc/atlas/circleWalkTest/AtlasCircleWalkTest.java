package us.ihmc.atlas.circleWalkTest;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.circleWalkTest.HumanoidCircleWalkTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@Tag("humanoid-flat-ground")
public class AtlasCircleWalkTest extends HumanoidCircleWalkTest
{

   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);
   private final AtlasJointMap jointMap = new AtlasJointMap(version, robotModel.getPhysicalProperties());

   /*
    * TODO This test was not running for a long time and now it's broken. I think it's because the arms
    * are not moving the robot is walking. -Sylvain
    */
   @Test
   @Disabled
   @Override
   public void testCircleWalk() throws SimulationExceededMaximumTimeException
   {
      super.testCircleWalk();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public double getRadiusForCircle()
   {
      return 2.00;
   }

   @Override
   public double getStepWidth()
   {
      return 0.35;
   }

   @Override
   public double getStepLength()
   {
      return 0.5;
   }

   @Override
   public int getArmDoF()
   {
      return jointMap.getArmJointNames().length;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return jointMap.getArmJointNames();
   }
}
