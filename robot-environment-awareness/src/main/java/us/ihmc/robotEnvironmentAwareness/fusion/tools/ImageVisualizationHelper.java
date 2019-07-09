package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import controller_msgs.msg.dds.Image32;
import sensor_msgs.msg.dds.RegionOfInterest;

public class ImageVisualizationHelper
{
   private static final float fontSize = 30f;
   private static final int defaultThickness = 5;

   public static BufferedImage convertImageMessageToBufferedImage(Image32 imageMessage)
   {
      int width = imageMessage.getWidth();
      int height = imageMessage.getHeight();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
      int pixelIndex = 0;
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {
            bufferedImage.setRGB(j, i, imageMessage.getRgbdata().get(pixelIndex));
            pixelIndex++;
         }
      }
      return bufferedImage;
   }

   public static void drawROIOnImage(BufferedImage image, RegionOfInterest roiToDraw, Color color)
   {
      drawROIOnImage(image, roiToDraw, color, defaultThickness, null);
   }

   public static void drawROIOnImage(BufferedImage image, RegionOfInterest roiToDraw, Color color, int thickness)
   {
      drawROIOnImage(image, roiToDraw, color, thickness, null);
   }

   public static void drawROIOnImage(BufferedImage image, RegionOfInterest roiToDraw, Color color, String text)
   {
      drawROIOnImage(image, roiToDraw, color, defaultThickness, text);
   }

   public static void drawROIOnImage(BufferedImage image, RegionOfInterest roiToDraw, Color color, int thickness, String text)
   {
      int width = image.getWidth() - 1;
      int height = image.getHeight() - 1;

      int topLimit = (int) Math.max(roiToDraw.getYOffset() - thickness, 0);
      int bottomLimit = (int) Math.min(roiToDraw.getYOffset() + roiToDraw.getHeight() + thickness, height);
      int leftLimit = (int) Math.max(roiToDraw.getXOffset() - thickness, 0);
      int rightLimit = (int) Math.min(roiToDraw.getXOffset() + roiToDraw.getWidth() + thickness, width);

      for (int i = leftLimit; i < rightLimit; i++)
      {
         for (int j = topLimit; j < topLimit + thickness; j++)
         {
            image.setRGB(i, j, color.getRGB());
         }

         for (int j = bottomLimit; j > bottomLimit - thickness; j--)
         {
            image.setRGB(i, j, color.getRGB());
         }
      }

      for (int i = topLimit; i < bottomLimit; i++)
      {
         for (int j = leftLimit; j < leftLimit + thickness; j++)
         {
            image.setRGB(j, i, color.getRGB());
         }

         for (int j = rightLimit; j > rightLimit - thickness; j--)
         {
            image.setRGB(j, i, color.getRGB());
         }
      }

      if (text != null)
      {
         Graphics graphics = image.getGraphics();
         graphics.setFont(graphics.getFont().deriveFont(fontSize));
         graphics.setColor(color);
         if (rightLimit != width)
            graphics.drawString(text, rightLimit, bottomLimit);
         else
            graphics.drawString(text, leftLimit, bottomLimit);
         graphics.dispose();
      }
   }
}
