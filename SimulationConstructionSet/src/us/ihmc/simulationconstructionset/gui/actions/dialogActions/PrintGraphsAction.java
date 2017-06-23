package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class PrintGraphsAction extends SCSAction
{
   private PrintGraphsDialogConstructor constructor;

   public PrintGraphsAction(PrintGraphsDialogConstructor constructor)
   {
      super("Print Graphs",
              "icons/PrintGraphs.png",
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

   @Override
   public void doAction()
   {
      constructor.constructDialog();
   }
}
