package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisOrientationTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasEndToEndPelvisOrientationTest extends EndToEndPelvisOrientationTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testGoHome()
   {
      super.testGoHome();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleTrajectoryPoint()
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testQueue()
   {
      super.testQueue();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalking()
   {
      super.testWalking();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingAfterTrajectory()
   {
      super.testWalkingAfterTrajectory();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMultipleTrajectoryPoints()
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithUserControl()
   {
      super.testWalkingWithUserControl();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testCustomControlFrame()
   {
      super.testCustomControlFrame();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }
}
