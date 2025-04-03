package us.ihmc.alexander.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarSwingWithWaypointsTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain")
public class AlexanderSwingWithWaypointsTest extends AvatarSwingWithWaypointsTest
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
      return new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

}
