package us.ihmc.avatar.stereoVision.sensor;

import us.ihmc.avatar.stereoVision.ImageCropInfo;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public abstract class StereoVisionSensorReader
{
   protected final List<StereoVisionSensorReaderListener> listeners = new ArrayList<>();

   private volatile boolean running = false;

   protected abstract void startInternal(SideDependentList<String> serialNumbers, ImageCropInfo imageCropInfo);

   public void start(SideDependentList<String> serialNumbers, ImageCropInfo imageCropInfo)
   {
      if (running)
      {
         throw new RuntimeException(getClass() + " already running");
      }

      running = true;

      startInternal(serialNumbers, imageCropInfo);
   }

   protected abstract void stopInternal();

   public void stop()
   {
      running = false;

      stopInternal();
   }

   public boolean isRunning()
   {
      return running;
   }

   public void registerListener(StereoVisionSensorReaderListener listener)
   {
      listeners.add(listener);
   }

   public void unregisterListener(StereoVisionSensorReaderListener listener)
   {
      listeners.remove(listener);
   }
}
