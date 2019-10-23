package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndGoHomeMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndGoHomeMessageTest extends EndToEndGoHomeMessageTest
{
   private DRCRobotModel robotModel;

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomeArms() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeArms();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomeChest() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomeChest();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testGoHomePelvis() throws SimulationExceededMaximumTimeException
   {
      super.testGoHomePelvis();
   }

   @BeforeEach
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
