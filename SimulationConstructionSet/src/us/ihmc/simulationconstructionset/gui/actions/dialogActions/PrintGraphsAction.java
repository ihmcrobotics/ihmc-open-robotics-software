package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;

public class PrintGraphsAction extends SCSAction
{
   private static final long serialVersionUID = -4403022630360741566L;
   private PrintGraphsDialogConstructor constructor;

   public PrintGraphsAction(PrintGraphsDialogConstructor constructor)
   {
      super("Print Graphs",
              "icons/Print24.gif",
              KeyEvent.VK_P,
              "Print Graphs",
              "Print Graphs"
      );

      this.constructor = constructor;
   }

   public void closeAndDispose()
   {
      constructor.closeAndDispose();
      constructor = null;
   }

   public void doAction()
   {
      constructor.constructDialog();
   }
   
}
