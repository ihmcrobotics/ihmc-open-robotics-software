package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class ToggleCameraKeyModeAction extends SCSAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private ToggleCameraKeyModeCommandExecutor executor;

   public ToggleCameraKeyModeAction(ToggleCameraKeyModeCommandExecutor executor)
   {
      super("Toggle Camera Keys",
              "",
              KeyEvent.VK_T,
              "Turns on and off",
              "Turns Camera Keying on and off."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.toggleCameraKeyMode();
   }
}

