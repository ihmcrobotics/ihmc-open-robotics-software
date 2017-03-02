package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;

public class ToggleCameraKeyModeAction extends AbstractAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private ToggleCameraKeyModeCommandExecutor executor;

   public ToggleCameraKeyModeAction(ToggleCameraKeyModeCommandExecutor executor)
   {
      super("Toggle Camera Keys");

      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_T));
      this.putValue(Action.LONG_DESCRIPTION, "Turns Camera Keying on and on.");
      this.putValue(Action.SHORT_DESCRIPTION, "turns on and off");
   }


   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.toggleCameraKeyMode();
   }
}
