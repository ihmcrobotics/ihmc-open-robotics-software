package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.IOException;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsPlanningToolboxControllerTest extends AvatarKinematicsPlanningToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

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

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @Test
   public void testDualHandTrajectory() throws Exception, UnreasonableAccelerationException
   {
      super.testDualHandTrajectory();
   }

   @Test
   public void testLinearInterpolatedTrajectory() throws Exception, UnreasonableAccelerationException
   {
      super.testLinearInterpolatedTrajectory();
   }

   @Test
   public void testReachToAPoint() throws Exception, UnreasonableAccelerationException
   {
      super.testReachToAPoint();
   }

   @Test
   public void testDifferentDistanceBetweenKeyFrames() throws Exception, UnreasonableAccelerationException
   {
      super.testDifferentDistanceBetweenKeyFrames();
   }

   @Test
   public void testLastKeyFrameBadPositionPlanning() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, IOException
   {
      super.testLastKeyFrameBadPositionPlanning();
   }
}
