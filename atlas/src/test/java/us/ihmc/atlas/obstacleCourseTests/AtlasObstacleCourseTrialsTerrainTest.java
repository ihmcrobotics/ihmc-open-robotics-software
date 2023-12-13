package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@Tag("humanoid-obstacle-slow-2")
public class AtlasObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   protected DRCRobotModel getRobotModelWithAdditionalFootContactPoints()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 5, 3, true, false);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Test
   public void testTrialsTerrainSlopeScript()
   {
      super.testTrialsTerrainSlopeScript(0.0);
   }

   @Override
   @Test
   public void testTrialsTerrainSlopeScriptRandomFootSlip()
   {
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }

   @Override
   @Test
   public void testTrialsTerrainZigzagHurdlesScript()
   {
      super.testTrialsTerrainZigzagHurdlesScript();
   }

   @Override
   @Disabled
   @Test
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip()
   {
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }

   @Override
   @Test
   public void testWalkingOntoAndOverSlopesSideways()
   {
      super.testWalkingOntoAndOverSlopesSideways();
   }

}
