package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingWithWaypointsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class AtlasSwingWithWaypointsTest extends AvatarSwingWithWaypointsTest
{
   @Override
   @Test
   public void testSwingWithWaypointsAndNotTrustingHeight()
   {
      super.testSwingWithWaypointsAndNotTrustingHeight();
   }

   @Override
   @Test
   public void testCrazySwingIsRejected()
   {
      super.testCrazySwingIsRejected();
   }

   @Override
   @Test
   public void testRegularSwingWithWaypoints()
   {
      super.testRegularSwingWithWaypoints();
   }

   @Override
   @Test
   public void testSwingWithWaypointsRotated()
   {
      super.testSwingWithWaypointsRotated();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

}
