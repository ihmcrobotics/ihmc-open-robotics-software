package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface LoadGraphGroupDialogConstructor
{
   public abstract void setCurrentDirectory(File directory);
   public abstract void setCurrentDirectory(String directory);
   public abstract void constructLoadConfigurationDialog();
   public abstract void loadGraphGroupFile(File file);
   public abstract void closeAndDispose();
}
