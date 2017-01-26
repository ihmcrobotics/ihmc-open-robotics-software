package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;
import java.io.File;

public interface CaptureDevice
{

   public abstract BufferedImage exportSnapshotAsBufferedImage();

   public abstract void exportSnapshot(File snapshotFile);

   public abstract int getHeight();

   public abstract int getWidth();

   public abstract void setSize(int width, int height);

   public abstract void streamTo(CameraStreamer cameraStreamer, int framesPerSecond);

}