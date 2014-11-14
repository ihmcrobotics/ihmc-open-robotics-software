package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface ExportSnapshotDialogConstructor
{
   public abstract void constructExportSnapshotDialog();
   public abstract void setCurrentDirectory(String directory);
   public abstract void setCurrentDirectory(File dir);

}
