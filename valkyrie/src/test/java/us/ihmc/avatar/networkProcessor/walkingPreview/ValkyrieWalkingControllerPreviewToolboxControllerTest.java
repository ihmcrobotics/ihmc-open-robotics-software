package us.ihmc.avatar.networkProcessor.walkingPreview;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final ValkyrieRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   @Test(timeout = 300000)
   public void testWalkingPreviewAlone() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testWalkingPreviewAlone();
   }

   @Override
   @Test(timeout = 300000)
   public void testStepsInPlacePreviewAtControllerDT() throws SimulationExceededMaximumTimeException
   {
      super.testStepsInPlacePreviewAtControllerDT();
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
}
