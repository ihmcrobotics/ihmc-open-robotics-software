package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;

@Tag("humanoid-obstacle-slow")
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{
   private final ValkyrieRobotModel robotModel = createRobotModel();

   private static ValkyrieRobotModel createRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      robotModel.setSimulationContactPoints(new AdditionalSimulationContactPoints<RobotSide>(RobotSide.values, 3, 4, true, false));
      return robotModel;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   protected DRCRobotModel getRobotModelWithAdditionalFootContactPoints()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      robotModel.setSimulationContactPoints(new AdditionalSimulationContactPoints<RobotSide>(RobotSide.values, 5, 4, true, false));
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Disabled // SCS ground contact points are really not adequate for that kind of test.
   @Override
   @Test
   public void testTrialsTerrainZigzagHurdlesScript()
   {
      super.testTrialsTerrainZigzagHurdlesScript();
   }

   @Override
   @Test
   public void testWalkingOntoAndOverSlopesSideways()
   {
      super.testWalkingOntoAndOverSlopesSideways();
   }

   @Override
   @Test
   public void testTrialsTerrainSlopeScriptRandomFootSlip()
   {
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }

   @Test
   public void testTrialsTerrainSlopeScript()
   {
      super.testTrialsTerrainSlopeScript(0.05);
   }

   /**
    * This test doesn't make any sense. We have plenty of tests evaluating robustness to foot slip, and
    * that one adds random foot slip while stepping narrow cinder blocks. The test will often fail
    * because the foot contact points are on the edge of a cinder block.
    */
   @Override
   @Disabled
   @Test
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip()
   {
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
