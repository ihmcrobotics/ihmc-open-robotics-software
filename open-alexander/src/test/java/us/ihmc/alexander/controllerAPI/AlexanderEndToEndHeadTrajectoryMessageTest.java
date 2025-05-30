package us.ihmc.alexander.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndHeadTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndHeadTrajectoryMessageTest extends EndToEndHeadTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);

   @Tag("controller-api")
   @Override
   @Test
   public void testSingleWaypoint()
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testLookingLeftAndRight()
   {
      super.testLookingLeftAndRight();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }
}
