package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface ImportDataDialogConstructor
{
   public abstract void setCurrentDirectory(File directory);
   public abstract void setCurrentDirectory(String directory);
   public abstract void constructImportDataDialog();
   public abstract void closeAndDispose();
}
