package us.ihmc.alexander.icpPlannerTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderJointMap;
import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.alexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.icpPlannerTests.AvatarICPPlannerFlatGroundTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-flat-ground")
public class AlexanderICPPlannerFlatGroundTest extends AvatarICPPlannerFlatGroundTest
{
   private final DRCRobotModel robotModel = new TestModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);

   @Override
   @Disabled
   @Test
   /** {@inheritDoc} */
   public void testChangeOfSupport()
   {
      super.testChangeOfSupport();
   }

   @Override
   @Test
   /** {@inheritDoc} */
   public void testPauseWalkingInSwing()
   {
      super.testPauseWalkingInSwing();
   }

   @Override
   @Test
   /** {@inheritDoc} */
   public void testPauseWalkingInTransferFirstStep()
   {
      super.testPauseWalkingInTransferFirstStep();
   }

   @Override
   @Test
   /** {@inheritDoc} */
   public void testPauseWalkingInTransfer()
   {
      super.testPauseWalkingInTransfer();
   }

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

   private static class TestModel extends OpenAlexanderRobotModel
   {
      private final TestWalkingParameters walkingParameters;

      public TestModel(OpenAlexanderVersion version, RobotTarget target)
      {
         super(version, target);
         walkingParameters = new TestWalkingParameters(version, target, getJointMap(), getPhysicalProperties());
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return walkingParameters;
      }

   }

   private static class TestWalkingParameters extends OpenAlexanderWalkingControllerParameters
   {
      public TestWalkingParameters(OpenAlexanderVersion version, RobotTarget target, AlexanderJointMap jointMap, AlexanderPhysicalProperties physicalProperties)
      {
         super(version, target, jointMap,physicalProperties);
      }

      @Override
      public boolean createFootholdExplorationTools()
      {
         return true;
      }
   }

}
