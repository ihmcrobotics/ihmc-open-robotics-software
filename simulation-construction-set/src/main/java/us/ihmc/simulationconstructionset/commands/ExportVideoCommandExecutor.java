package us.ihmc.simulationconstructionset.commands;

import java.awt.Dimension;
import java.io.File;

import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;

public interface ExportVideoCommandExecutor
{
   public abstract void createVideo(File file);
   public void createVideo(CameraController cameraController, File selectedFile, Dimension dimension, Boolean isSequanceSelected, double playBackRate, double frameRate);
   public void createVideo(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate);

}
