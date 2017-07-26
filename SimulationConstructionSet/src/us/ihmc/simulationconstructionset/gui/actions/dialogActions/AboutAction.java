package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;

public class AboutAction extends AbstractAction
{
   private static final long serialVersionUID = 6795904392077029991L;

   private final AboutDialogConstructor constructor;

   public AboutAction(AboutDialogConstructor constructor)
   {
      super("About...");
      this.constructor = constructor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_A));
      this.putValue(Action.LONG_DESCRIPTION, "Display About Information");
      this.putValue(Action.SHORT_DESCRIPTION, "About");
   }

   @Override
   public void actionPerformed(ActionEvent actionEventb)
   {
      constructor.constructDialog();
   }

}
