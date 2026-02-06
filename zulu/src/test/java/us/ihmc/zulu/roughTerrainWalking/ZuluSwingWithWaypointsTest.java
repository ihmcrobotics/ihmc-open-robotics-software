package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingWithWaypointsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class ZuluSwingWithWaypointsTest extends AvatarSwingWithWaypointsTest
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
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

}
