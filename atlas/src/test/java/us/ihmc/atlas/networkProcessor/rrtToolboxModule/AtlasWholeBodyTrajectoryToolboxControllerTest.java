package us.ihmc.atlas.networkProcessor.rrtToolboxModule;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.AvatarWholeBodyTrajectoryToolboxControllerTest;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class AtlasWholeBodyTrajectoryToolboxControllerTest extends AvatarWholeBodyTrajectoryToolboxControllerTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   private DRCRobotModel ghostRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);

   @Override
   public void testHandCirclePositionOnly() throws Exception, UnreasonableAccelerationException
   {
      super.testHandCirclePositionOnly();
   }

   @Override
   public void testHandCircleFullyConstrained() throws Exception, UnreasonableAccelerationException
   {
      super.testHandCircleFullyConstrained();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return getRobotModel().getSimpleRobotName();
   }
}
