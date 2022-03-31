package us.ihmc.avatar.networkProcessor.walkingPreview;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
   private final ValkyrieRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testWalkingPreviewAlone() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testWalkingPreviewAlone();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testStepsInPlacePreviewAtControllerDT() throws SimulationExceededMaximumTimeException
   {
      super.testStepsInPlacePreviewAtControllerDT();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testResetFeature() throws SimulationExceededMaximumTimeException
   {
      super.testResetFeature();
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
