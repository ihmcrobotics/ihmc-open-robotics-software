package us.ihmc.valkyrie.obstacleCourse;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatWithErrorsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieObstacleCourseFlatWithErrorsTest extends DRCObstacleCourseFlatWithErrorsTest
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

   @Override
   protected Vector3D getFootSlipVector()
   {
      return new Vector3D(0.02, -0.02, 0.0);
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.1;
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testSimpleFlatGroundScriptWithOscillatingFeet()
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testSimpleFlatGroundScriptWithRandomFootSlip()
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testSideStepsWithSlipping()
   {
      super.testSideStepsWithSlipping();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testStandingWithOscillatingFeet()
   {
      super.testStandingWithOscillatingFeet();
   }

   @Tag("humanoid-flat-ground-slow")
   @Override
   @Test
   public void testSideStepsWithRandomSlipping()
   {
      super.testSideStepsWithRandomSlipping();
   }
}
