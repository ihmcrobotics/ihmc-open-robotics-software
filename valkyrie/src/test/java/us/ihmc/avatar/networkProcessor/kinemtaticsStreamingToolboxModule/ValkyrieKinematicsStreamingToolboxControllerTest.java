package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   @Override
   @Test
   public void testHandMotionWithCollision()
   {
      super.testHandMotionWithCollision();
   }

   @Test
   @Override
   public void testStreamingToController() throws SimulationExceededMaximumTimeException
   {
      super.testStreamingToController();
   }
}
