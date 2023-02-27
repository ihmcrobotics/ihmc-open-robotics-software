package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseEveryBuildTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("fast")
public class ValkyrieObstacleCourseEveryBuildTest extends DRCObstacleCourseEveryBuildTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
   
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Tag("humanoid-obstacle")
   @Override
   @Test
   public void testSimpleFlatGroundScript()
   {
      super.testSimpleFlatGroundScript();
   }

   @Tag("humanoid-obstacle")
   @Override
   @Test
   public void testWalkingUpToRampWithLongSteps()
   {
      super.testWalkingUpToRampWithLongSteps();
   }
}
