package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.controller.ZuluContactPointParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidPartialFootholdWalkingTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

// This doesn't work if the partial foothold module isn't created, which it's not by default.
@Disabled
@Tag("humanoid-rough-terrain-slow")
public class ZuluPartialFootholdWalkingTest extends HumanoidPartialFootholdWalkingTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      ZuluVersion version = ZuluVersion.V1_FULL_ROBOT;
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 10, 5, true, false);
      ZuluContactPointParameters contactPointParameters=  new ZuluContactPointParameters(version.getJointMap(), version.getPhysicalProperties(), simulationContactPoints, false);
      ZuluRobotModel robotModel = new ZuluRobotModel(version, RobotTarget.SCS, contactPointParameters)
      {
         @Override
         public double getSimulateDT()
         {
            return 0.00025;
         }
      };

      return robotModel;
   }

   @Override
   public String getLeftAnkleXName()
   {
      return "LEFT_ANKLE_X";
   }

   @Override
   public String getLeftAnkleYName()
   {
      return "LEFT_ANKLE_Y";
   }

   @Override
   public String getRightAnkleXName()
   {
      return "RIGHT_ANKLE_X";
   }

   @Override
   public String getRightAnkleYName()
   {
      return "RIGHT_ANKLE_Y";
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Test
   public void testSteppingOntoBlock()
   {
      super.testSteppingOntoBlock();
   }

   @Test
   public void testWalkingOverBlock()
   {
      super.testWalkingOverBlock();
   }
}
