package us.ihmc.graphicsDescription.instructions;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.font.FontRenderContext;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.listeners.ExtrusionChangedListener;

public class Graphics3DAddExtrusionInstruction extends Graphics3DInstruction
{
   private BufferedImage bufferedImageToExtrude;
   private double thickness;

   private ExtrusionChangedListener extrusionChangedListener;

   // Text3D stuff
   private final Font font = new Font("Lucida Sans", Font.PLAIN, 40);
   
   private final FontRenderContext fontRenderContext;

   private Graphics3DAddExtrusionInstruction()
   {
      super();
      BufferedImage measurementImage = new BufferedImage(1, 1, BufferedImage.TYPE_3BYTE_BGR);
      Graphics2D measurementGraphics = measurementImage.createGraphics();
      fontRenderContext = measurementGraphics.getFontRenderContext();
      
   }
   
   public Graphics3DAddExtrusionInstruction(String text, double thickness, AppearanceDefinition appearance)
   {
      this();
      this.thickness = thickness;
      this.setText(text);
      this.setAppearance(appearance);
      
   }

   /**
    * Create an extrusion of a BufferedImage. Black pixels of the image are extruded. 
    * A pixel is considered black when (red+green+blue)/3 < 60
    * 
    * @param bufferedImageToExtrude    BufferedImage to extrude
    * @param thickness Thinkness of extrusion
    * @param appearance Appearance
    */
   public Graphics3DAddExtrusionInstruction(BufferedImage bufferedImageToExtrude, double thickness, AppearanceDefinition appearance)
   {
      this();
      this.thickness = thickness;
      this.setBufferedImage(bufferedImageToExtrude);
      this.setAppearance(appearance);
   }

   public BufferedImage getBufferedImage()
   {
      return bufferedImageToExtrude;
   }

   public void setBufferedImage(BufferedImage newImage)
   {
      this.bufferedImageToExtrude = newImage;
      notifyChangedListener();
   }

   public void setHeight(double height)
   {
      this.thickness = height;
      notifyChangedListener();
   }

   public void setText(String text)
   {
      Rectangle2D bounds = font.getStringBounds(text, fontRenderContext);

      int width = (int) bounds.getWidth();
      int height = (int) bounds.getHeight();

      bufferedImageToExtrude = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
      Graphics g = bufferedImageToExtrude.getGraphics();

      g.setColor(Color.white);
      g.fillRect(0, 0, width, height);
      g.setColor(Color.black);
      g.setFont(font);
      g.drawString(text, 0, (int) (height + bounds.getCenterY()));
      g.dispose();

      notifyChangedListener();
   }

   private void notifyChangedListener()
   {
      if (extrusionChangedListener != null)
      {
         extrusionChangedListener.extrusionChanged(bufferedImageToExtrude, thickness);
      }
   }

   public void setTextChangedListener(ExtrusionChangedListener textChangedListener)
   {
      this.extrusionChangedListener = textChangedListener;
   }

   public double getHeight()
   {
      return thickness;
   }

}
