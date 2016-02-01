package us.ihmc.imageProcessing;

import java.awt.image.BufferedImage;

/**
 * @author Peter Abeles
 */
public interface SimpleNavigation {

   public boolean process( BufferedImage image );

   public double getSteer();
}
