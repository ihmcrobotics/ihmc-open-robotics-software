package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AboutDialogConstructor;

public class AboutAction extends SCSAction
{
   private static final long serialVersionUID = 6795904392077029991L;

   private final AboutDialogConstructor constructor;

   public AboutAction(AboutDialogConstructor constructor)
   {
      super("About...",
              "",
              KeyEvent.VK_A,
              "About",
              "Display About Information"
      );

      this.constructor = constructor;
   }

   public void doAction()
   {
      constructor.constructDialog();
   }

}
