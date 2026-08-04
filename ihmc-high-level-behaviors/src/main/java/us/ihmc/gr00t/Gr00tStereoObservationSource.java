package us.ihmc.gr00t;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;

/** Packs caller-defined robot state and configurable stereo images into a GR00T client. */
public final class Gr00tStereoObservationSource implements Gr00tObservationSource
{
   private final Gr00tClient client;
   private final Gr00tStatePacker statePacker;
   private final SideDependentList<Integer> sensorImageKeys;
   private final SideDependentList<Mat> images;
   private final SideDependentList<Mat> rgbImages = new SideDependentList<>(side -> new Mat());
   private final SideDependentList<Mat> resizedImages = new SideDependentList<>(side -> new Mat());

   public Gr00tStereoObservationSource(Gr00tClient client, Gr00tStatePacker statePacker, SideDependentList<Integer> sensorImageKeys)
   {
      this.client = client;
      this.statePacker = statePacker;
      this.sensorImageKeys = new SideDependentList<>(sensorImageKeys.get(RobotSide.LEFT), sensorImageKeys.get(RobotSide.RIGHT));
      images = new SideDependentList<>(new Mat(client.getImageHeight(), client.getImageWidth(), opencv_core.CV_8UC3),
                                           new Mat(client.getImageHeight(), client.getImageWidth(), opencv_core.CV_8UC3));
   }

   @Override
   public boolean pack(ImageSensor sensor)
   {
      client.getState().clear();
      boolean stateValid = statePacker.pack(client.getState());
      boolean imagesValid = packImages(sensor);
      return stateValid && imagesValid;
   }

   private boolean packImages(ImageSensor sensor)
   {
      if (sensor == null)
         return false;
      boolean valid = true;
      for (RobotSide side : RobotSide.values)
      {
         RawImage image = sensor.getImage(sensorImageKeys.get(side));
         if (image == null)
         {
            valid = false;
            continue;
         }
         try
         {
            int width = client.getImageWidth();
            int height = client.getImageHeight();
            Mat rgbColor = rgbImages.get(side);
            image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), rgbColor, PixelFormat.RGB8);
            try (Size cropSize = new Size(width, height);
                 Size scaleSize = new Size(image.getWidth() * height / image.getHeight(), height))
            {
               Mat resized = resizedImages.get(side);
               opencv_imgproc.resize(rgbColor, resized, scaleSize);
               try (Point cropOffset = new Point((resized.cols() - width) / 2, 0);
                    Rect roi = new Rect(cropOffset, cropSize);
                    Mat cropped = new Mat(resized, roi))
               {
                  cropped.copyTo(images.get(side));
               }
            }
            images.get(side).data().get(client.getImages().get(side).array());
         }
         finally
         {
            image.release();
         }
      }
      return valid;
   }

   @Override
   public void close()
   {
      for (RobotSide side : RobotSide.values)
      {
         images.get(side).close();
         rgbImages.get(side).close();
         resizedImages.get(side).close();
      }
   }
}
