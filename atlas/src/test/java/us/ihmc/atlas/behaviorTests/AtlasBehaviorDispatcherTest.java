package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.HumanoidBehaviorDispatcherTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasBehaviorDispatcherTest extends HumanoidBehaviorDispatcherTest
{
   private final AtlasRobotModel robotModel;

   public AtlasBehaviorDispatcherTest()
   {

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testDispatchKarateKidDiagnosticBehavior()
   {
      super.testDispatchKarateKidDiagnosticBehavior();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testDispatchPelvisPoseBehavior()
   {
      super.testDispatchPelvisPoseBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testDispatchWalkToLocationBehavior()
   {
      super.testDispatchWalkToLocationBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testDispatchWalkToLocationBehaviorAndStop()
   {
      super.testDispatchWalkToLocationBehaviorAndStop();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testDispatchWalkToLocationBehaviorPauseAndResume()
   {
      super.testDispatchWalkToLocationBehaviorPauseAndResume();
   }
}
