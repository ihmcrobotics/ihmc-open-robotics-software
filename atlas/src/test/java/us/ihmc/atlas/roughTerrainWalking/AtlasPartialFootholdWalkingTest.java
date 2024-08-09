package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidPartialFootholdWalkingTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@Tag("humanoid-rough-terrain-slow")
public class AtlasPartialFootholdWalkingTest extends HumanoidPartialFootholdWalkingTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 10, 5, true, false);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints)
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
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
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
