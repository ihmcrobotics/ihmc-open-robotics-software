package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class ToggleCameraKeyModeAction extends SCSAction
{
   private ToggleCameraKeyModeCommandExecutor executor;

   public ToggleCameraKeyModeAction(ToggleCameraKeyModeCommandExecutor executor)
   {
      super("Toggle Camera Keys",
              "",
              KeyEvent.VK_T,
              "Toggle Camera Keying Mode",
              "Turns Camera Keying on and off."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.toggleCameraKeyMode();
   }
}

