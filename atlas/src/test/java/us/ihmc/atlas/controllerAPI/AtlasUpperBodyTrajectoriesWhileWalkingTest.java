package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndUpperBodyTrajectoriesWhileWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("controller-api-2")
public class AtlasUpperBodyTrajectoriesWhileWalkingTest extends EndToEndUpperBodyTrajectoriesWhileWalkingTest
{

   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithRandomArmTrajectoryMovements() throws Exception
   {
      super.testWalkingWithRandomArmTrajectoryMovements();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithArmsHoldingInFeetFrame() throws Exception
   {
      super.testWalkingWithArmsHoldingInFeetFrame();
   }
}
