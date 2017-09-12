package us.ihmc.imageProcessing.utilities.stereo;

import java.awt.image.BufferedImage;

/**
 * User: Matt
 * Date: 12/4/12
 */
public interface StereoVideoListener
{
   public void updateImage(BufferedImage leftEye, BufferedImage rightEye);
}
