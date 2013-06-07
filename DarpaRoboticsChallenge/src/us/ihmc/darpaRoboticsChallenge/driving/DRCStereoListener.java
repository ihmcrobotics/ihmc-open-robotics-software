package us.ihmc.darpaRoboticsChallenge.driving;

import java.awt.image.BufferedImage;

/**
 * Listens for stereo images
 *
 * @author Peter Abeles
 */
public interface DRCStereoListener
{
   public void leftImage( BufferedImage image , long timestamp, double fov );

   public void rightImage( BufferedImage image , long timestamp, double fov );
}
