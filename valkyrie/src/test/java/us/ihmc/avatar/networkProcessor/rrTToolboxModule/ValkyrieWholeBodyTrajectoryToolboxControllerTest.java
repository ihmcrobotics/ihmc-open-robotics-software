package us.ihmc.avatar.networkProcessor.rrTToolboxModule;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxSettings;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieWholeBodyTrajectoryToolboxControllerTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
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

   @Override
   public void testHandCirclePositionAndYaw() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      super.testHandCirclePositionAndYaw();
   }

   @Override
   public void testHandCirclePositionAndYawPitchRoll() throws Exception, UnreasonableAccelerationException
   {
      handControlFrames = WholeBodyTrajectoryToolboxSettings.getValkyrieHandControlFrames();
      super.testHandCirclePositionAndYawPitchRoll();
   }
}
