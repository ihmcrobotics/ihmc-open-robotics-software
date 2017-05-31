package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportGraphsToFileConstructor;

public class ExportGraphsToFileAction extends SCSAction
{
   /**
    * 
    */
   private static final long serialVersionUID = 7076583053729816960L;
   private ExportGraphsToFileConstructor constructor;

   public ExportGraphsToFileAction(ExportGraphsToFileConstructor constructor)
   {
      super("Export Graphs To File",
              "icons/exportGraph.png",
              KeyEvent.VK_UNDEFINED,
              "Export Graphs To File",
              "Export Graphs To File"
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
