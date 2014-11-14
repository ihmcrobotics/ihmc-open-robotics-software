package us.ihmc.simulationconstructionset.commands;

import java.io.File;

import us.ihmc.graphics3DAdapter.camera.CaptureDevice;

public interface ExportMovieCommandExecutor
{
   public abstract void createMovie(File file);
   public void createMovie(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate);

}
