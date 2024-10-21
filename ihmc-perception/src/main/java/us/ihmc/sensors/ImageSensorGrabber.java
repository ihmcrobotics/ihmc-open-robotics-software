package us.ihmc.sensors;

import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;

public interface ImageSensorGrabber extends AutoCloseable
{
   /**
    * Initializes and starts the sensor.
    * @return Whether the sensor was successfully initialized and started.
    */
   boolean startSensor();

   /**
    * @return Whether the sensor is running.
    */
   boolean isRunning();

   /**
    * Grab all images the sensor provides. Blocks until new images are available.
    * @return Whether new images were successfully grabbed.
    */
   boolean grab();

   /**
    * Retrieve a grabbed image by its key
    * @param imageKey Key of the image to retrieve
    * @return The latest image matching the passed in key.
    */
   RawImage getImage(int imageKey);

   /**
    * @return The name of the sensor.
    */
   String getSensorName();

   default CameraModel getCameraModel()
   {
      return CameraModel.PINHOLE;
   }
}
