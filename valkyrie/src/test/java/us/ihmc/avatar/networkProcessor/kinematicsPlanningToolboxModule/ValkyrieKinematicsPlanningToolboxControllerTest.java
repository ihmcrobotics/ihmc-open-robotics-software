package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.io.IOException;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsPlanningToolboxControllerTest extends AvatarKinematicsPlanningToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);

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

   @Tag("humanoid-toolbox")
   @Test
   public void testDualHandTrajectory() throws Exception, UnreasonableAccelerationException
   {
      super.testDualHandTrajectory();
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testLinearInterpolatedTrajectory() throws Exception, UnreasonableAccelerationException
   {
      super.testLinearInterpolatedTrajectory();
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testReachToAPoint() throws Exception, UnreasonableAccelerationException
   {
      super.testReachToAPoint();
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testDifferentDistanceBetweenKeyFrames() throws Exception, UnreasonableAccelerationException
   {
      super.testDifferentDistanceBetweenKeyFrames();
   }

   @Tag("humanoid-toolbox")
   @Test
   public void testLastKeyFrameBadPositionPlanning() throws UnreasonableAccelerationException, SimulationExceededMaximumTimeException, IOException
   {
      super.testLastKeyFrameBadPositionPlanning();
   }
}
