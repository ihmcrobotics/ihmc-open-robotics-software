package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface LoadGraphGroupDialogConstructor extends DialogConstructor, DialogConstructorWithDirectory, DialogDestructor
{
   void loadGraphGroupFile(File file);
}
