package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.RemoveCameraKeyCommandExecutor;

public class RemoveCameraKeyAction extends AbstractAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private RemoveCameraKeyCommandExecutor executor;

   public RemoveCameraKeyAction(RemoveCameraKeyCommandExecutor executor)
   {
      super("Remove Camera Key");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_R));
      this.putValue(Action.LONG_DESCRIPTION, "Removes the camera key that is stored at the time indicated on the graph when this button is pressed.");
      this.putValue(Action.SHORT_DESCRIPTION, "removes current camera key");
   }


   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.removeCameraKey();
   }
}
