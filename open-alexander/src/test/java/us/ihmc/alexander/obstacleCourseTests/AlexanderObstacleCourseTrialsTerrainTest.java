package us.ihmc.alexander.obstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.alexander.parameters.controller.AlexanderContactPointParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@Tag("humanoid-obstacle-slow-2")
public class AlexanderObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   protected DRCRobotModel getRobotModelWithAdditionalFootContactPoints()
   {
      OpenAlexanderVersion version = OpenAlexanderVersion.V0_FULL_ROBOT;
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 5, 3, true, false);
      AlexanderContactPointParameters contactPointParameters = new AlexanderContactPointParameters(version.getJointMap(), version.getPhysicalProperties(), simulationContactPoints, false);
      return new OpenAlexanderRobotModel(version, RobotTarget.SCS, contactPointParameters);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
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
