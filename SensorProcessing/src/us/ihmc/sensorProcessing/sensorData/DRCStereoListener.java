package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import us.ihmc.utilities.robotSide.RobotSide;

/**
 * Listens for stereo images
 *
 * @author Peter Abeles
 */
public interface DRCStereoListener
{
   /**
    *
    * @param robotSide TODO
    * @param image
    * @param timestamp Time stamp in nano-seconds
    * @param fov
    */
   public void newImageAvailable(RobotSide robotSide , BufferedImage image, long timestamp, double fov );

}
