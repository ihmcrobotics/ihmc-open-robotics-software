package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseFlatWithErrorsTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

import static us.ihmc.continuousIntegration.IntegrationCategory.SLOW;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
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
   @ContinuousIntegrationTest(estimatedDuration = 84.6)
   @Test(timeout = 420000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithRandomSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.6, categoriesOverride = SLOW)
   @Test(timeout = 370000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      super.testSideStepsWithSlipping();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 92.6)
   @Test(timeout = 460000)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 82.6)
   @Test(timeout = 410000)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.2)
   @Test(timeout = 220000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithOscillatingFeet();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.2)
   @Test(timeout = 220000)
   public void testStandingWithStateEstimatorDrift() throws SimulationExceededMaximumTimeException
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
