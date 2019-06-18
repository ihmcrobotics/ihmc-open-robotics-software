package us.ihmc.valkyrie.behaviorTests;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.behaviorTests.KinematicsPlanningBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsPlanningBehaviorTest extends KinematicsPlanningBehaviorTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

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
   @Test
   public void testReachToDoorKnob() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testReachToDoorKnob();
   }

   @Override
   @Test
   public void testSingleKeyFrameInput() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSingleKeyFrameInput();
   }
}
