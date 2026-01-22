package us.ihmc.zulu.icpPlannerTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluJointMap;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.parameters.controller.ZuluWalkingControllerParameters;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.icpPlannerTests.AvatarICPPlannerFlatGroundTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-flat-ground")
public class ZuluICPPlannerFlatGroundTest extends AvatarICPPlannerFlatGroundTest
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
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   private static class TestModel extends ZuluRobotModel
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

   private static class TestWalkingParameters extends ZuluWalkingControllerParameters
   {
      public TestWalkingParameters(ZuluVersion version, RobotTarget target, ZuluJointMap jointMap, ZuluPhysicalProperties physicalProperties)
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
