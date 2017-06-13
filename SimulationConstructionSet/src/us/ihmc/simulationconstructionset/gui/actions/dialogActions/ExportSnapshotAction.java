package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class ExportSnapshotAction extends SCSAction
{
   private final ExportSnapshotDialogConstructor constructor;
   
   public ExportSnapshotAction(ExportSnapshotDialogConstructor constructor)
   {
      super("Export Snapshot...",
              "icons/Screenshot.png",
              KeyEvent.VK_S,
              "Export Snapshot",
              "Export snapshot to a file."
      );

      this.constructor = constructor;
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   @Override
   public void doAction()
   {
     constructor.constructDialog();
   }
}
