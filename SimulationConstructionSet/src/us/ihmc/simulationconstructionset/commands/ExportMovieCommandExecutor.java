package us.ihmc.simulationconstructionset.commands;

import java.awt.Dimension;
import java.io.File;

import us.ihmc.graphics3DAdapter.camera.CameraController;
import us.ihmc.graphics3DAdapter.camera.CaptureDevice;

public interface ExportMovieCommandExecutor
{
   public abstract void createMovie(File file);
   public void createMovie(CameraController cameraController, File selectedFile, Dimension dimension, Boolean isSequanceSelected, double playBackRate, double frameRate);
   public void createMovie(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate);

}
