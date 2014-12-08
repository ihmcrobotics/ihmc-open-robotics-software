package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

/**
 * Listens for stereo images
 *
 * @author Peter Abeles
 */
public interface DRCStereoListener
{
   /**
    *
    * @param image
    * @param timestamp Time stamp in nano-seconds
    * @param fov
    */
   public void leftImage( BufferedImage image , long timestamp, double fov );

   public void rightImage( BufferedImage image , long timestamp, double fov );
}
