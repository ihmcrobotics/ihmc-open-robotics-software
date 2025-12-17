package us.ihmc.openAlexander.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatWithErrorsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-flat-ground-slow-3")
public class AlexanderObstacleCourseFlatWithErrorsTest extends DRCObstacleCourseFlatWithErrorsTest
{
   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testSideStepsWithRandomSlipping()
   {
      super.testSideStepsWithRandomSlipping();
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testSideStepsWithSlipping()
   {
      super.testSideStepsWithSlipping();
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testSimpleFlatGroundScriptWithOscillatingFeet()
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testSimpleFlatGroundScriptWithRandomFootSlip()
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testStandingWithOscillatingFeet()
   {
      super.testStandingWithOscillatingFeet();
   }

   @Tag("humanoid-flat-ground-slow-3")
   @Override
   @Test
   public void testStandingWithStateEstimatorDrift()
   {
      super.testStandingWithStateEstimatorDrift();
   }

   @Override
   protected Vector3D getFootSlipVector()
   {
      return new Vector3D(0.05, -0.07, 0.0);//(0.06, -0.06, 0.0);
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.0;
   }
}
