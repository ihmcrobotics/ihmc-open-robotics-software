package us.ihmc.simulationconstructionset.commands;

import java.io.File;

public interface ExportSnapshotCommandExecutor
{
   public abstract void exportSnapshot(File snapshotFile);
}
