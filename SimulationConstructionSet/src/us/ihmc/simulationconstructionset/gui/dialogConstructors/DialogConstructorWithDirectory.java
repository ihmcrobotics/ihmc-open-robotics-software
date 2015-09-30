package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface DialogConstructorWithDirectory
{
    void setCurrentDirectory(File directory);

    void setCurrentDirectory(String directory);
}
