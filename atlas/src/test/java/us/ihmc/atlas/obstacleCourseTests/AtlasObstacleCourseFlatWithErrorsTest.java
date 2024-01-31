package us.ihmc.atlas.obstacleCourseTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatWithErrorsTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

@Tag("humanoid-flat-ground-slow-3")
public class AtlasObstacleCourseFlatWithErrorsTest extends DRCObstacleCourseFlatWithErrorsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testSideStepsWithRandomSlipping()
   {
      super.testSideStepsWithRandomSlipping();
   }

   @Override
   @Test
   public void testSideStepsWithSlipping()
   {
      super.testSideStepsWithSlipping();
   }

   @Override
   @Test
   public void testSimpleFlatGroundScriptWithOscillatingFeet()
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @Test
   public void testSimpleFlatGroundScriptWithRandomFootSlip()
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Override
   @Test
   public void testStandingWithOscillatingFeet()
   {
      super.testStandingWithOscillatingFeet();
   }

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
