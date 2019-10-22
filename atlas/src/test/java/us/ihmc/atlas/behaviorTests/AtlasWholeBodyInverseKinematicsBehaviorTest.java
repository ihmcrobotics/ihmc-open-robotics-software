package us.ihmc.atlas.behaviorTests;

import java.io.IOException;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.WholeBodyInverseKinematicsBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasWholeBodyInverseKinematicsBehaviorTest extends WholeBodyInverseKinematicsBehaviorTest
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
      return robotModel.getSimpleRobotName();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForAHandPose();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForBothHandPoses();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForChestAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForChestAngularControl();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandAngularLinearControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandAngularLinearControl();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandRollConstraint() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandRollConstraint();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandSelectionMatrix() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandSelectionMatrix();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForPelvisAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForPelvisAngularControl();
   }
}
