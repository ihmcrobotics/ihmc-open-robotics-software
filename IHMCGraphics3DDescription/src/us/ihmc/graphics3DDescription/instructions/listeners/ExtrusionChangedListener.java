package us.ihmc.graphics3DDescription.instructions.listeners;

import java.awt.image.BufferedImage;

public interface ExtrusionChangedListener
{
   public void extrusionChanged(BufferedImage bufferedImageToExtrude, double height);
}
