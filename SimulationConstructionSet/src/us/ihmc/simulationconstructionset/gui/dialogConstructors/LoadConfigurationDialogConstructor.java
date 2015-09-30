package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface LoadConfigurationDialogConstructor extends DialogConstructorWithDirectory, DialogConstructor
{
   void loadGUIConfigurationFile(File file);
}
