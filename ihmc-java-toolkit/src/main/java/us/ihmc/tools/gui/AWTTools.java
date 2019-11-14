package us.ihmc.tools.gui;

import java.awt.*;

public class AWTTools
{
   public static Dimension getDimensionOfSmallestScreenScaled(double scalar)
   {
      Dimension smallestDimension = getDimensionOfSmallestScreen();
      smallestDimension.setSize(smallestDimension.getWidth() * scalar, smallestDimension.getHeight() * scalar);
      return smallestDimension;
   }

   public static Dimension getDimensionOfSmallestScreen()
   {
      Dimension smallestDimension = new Dimension(Integer.MAX_VALUE, Integer.MAX_VALUE);
      for (GraphicsDevice graphicsDevice : GraphicsEnvironment.getLocalGraphicsEnvironment().getScreenDevices())
      {
         for (GraphicsConfiguration configuration : graphicsDevice.getConfigurations())
         {
            Rectangle bounds = configuration.getBounds();

            if (bounds.getWidth() < smallestDimension.getWidth())
            {
               smallestDimension.setSize((int) bounds.getWidth(), smallestDimension.getHeight());
            }
            if (bounds.getHeight() < smallestDimension.getHeight())
            {
               smallestDimension.setSize(smallestDimension.getWidth(), (int) bounds.getHeight());
            }
         }
      }

      return smallestDimension;
   }
}
