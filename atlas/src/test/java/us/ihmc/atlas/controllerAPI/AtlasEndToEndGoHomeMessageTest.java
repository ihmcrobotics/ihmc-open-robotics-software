package us.ihmc.atlas.controllerAPI;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndGoHomeMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndGoHomeMessageTest extends EndToEndGoHomeMessageTest
{
   private DRCRobotModel robotModel;

   @Override
   @Test(timeout = 300000)
   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeArms();
   }

   @Override
   @Test(timeout = 300000)
   public void testGoHomeChest() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeChest();
   }

   @Override
   @Test(timeout = 300000)
   public void testGoHomePelvis() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomePelvis();
   }

   @Before
   public void createRobotModel()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

}
