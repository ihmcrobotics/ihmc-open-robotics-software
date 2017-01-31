package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.PreviousCameraKeyCommandExecutor;

public class PreviousCameraKeyAction extends AbstractAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private PreviousCameraKeyCommandExecutor executor;

   public PreviousCameraKeyAction(PreviousCameraKeyCommandExecutor executor)
   {
      super("Previous Camera Key");

      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_R));
      this.putValue(Action.LONG_DESCRIPTION, "Procedes to the prvious chronological CameraKey.");
      this.putValue(Action.SHORT_DESCRIPTION, "previous camera key");
   }


   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.previousCameraKey();
   }
}
