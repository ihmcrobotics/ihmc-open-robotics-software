package us.ihmc.openAlexander.icpPlannerTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluJointMap;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.openAlexander.parameters.controller.OpenAlexanderWalkingControllerParameters;
import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.icpPlannerTests.AvatarICPPlannerFlatGroundTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-flat-ground")
public class AlexanderICPPlannerFlatGroundTest extends AvatarICPPlannerFlatGroundTest
{
   private final DRCRobotModel robotModel = new TestModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

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

      public TestModel(ZuluVersion version, RobotTarget target)
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
      public TestWalkingParameters(ZuluVersion version, RobotTarget target, ZuluJointMap jointMap, AlexanderPhysicalProperties physicalProperties)
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
