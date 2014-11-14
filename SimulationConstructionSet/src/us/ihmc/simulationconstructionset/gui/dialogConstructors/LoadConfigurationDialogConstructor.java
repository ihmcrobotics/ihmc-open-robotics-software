package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface LoadConfigurationDialogConstructor
{
   public abstract void setCurrentDirectory(File directory);
   public abstract void setCurrentDirectory(String directory);
   public abstract void constructLoadConfigurationDialog();
   public abstract void loadGUIConfigurationFile(File file);
}
