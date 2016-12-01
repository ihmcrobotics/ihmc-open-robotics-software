package us.ihmc.javaFXToolkit.shapes;

import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import us.ihmc.robotics.MathTools;

public class TextureColorPalette1D implements TextureColorPalette
{
   private static final boolean PRINT_PALETTE = false;
   private static final int DEFAULT_RESOLUTION = 256;

   private int hueResolution = -1;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   private double hueConstant = Double.NaN;
   private double saturationConstant = Double.NaN;
   private double brightnessConstant = Double.NaN;

   private Image colorPalette;

   public TextureColorPalette1D()
   {
      setHueBased(1.0, 1.0);
   }

   public void setHueBased(double saturationConstant, double brightnessConstant)
   {
      setHueBased(DEFAULT_RESOLUTION, saturationConstant, brightnessConstant);
   }

   public void setHueBased(int hueResolution, double saturationConstant, double brightnessConstant)
   {
      MathTools.checkIfGreaterOrEqual(hueResolution, 1);
      MathTools.checkIfInRange(saturationConstant, 0.0, 1.0);
      MathTools.checkIfInRange(brightnessConstant, 0.0, 1.0);

      this.hueResolution = hueResolution;
      this.saturationResolution = -1;
      this.brightnessResolution = -1;

      this.hueConstant = Double.NaN;
      this.saturationConstant = saturationConstant;
      this.brightnessConstant = brightnessConstant;

      updateColorPalette();
   }

   public void setSaturationBased(double hueConstant, double brightnessConstant)
   {
      setSaturationBased(DEFAULT_RESOLUTION, hueConstant, brightnessConstant);
   }

   public void setSaturationBased(int saturationResolution, double hueConstant, double brightnessConstant)
   {
      MathTools.checkIfGreaterOrEqual(saturationResolution, 1);
      MathTools.checkIfInRange(brightnessConstant, 0.0, 1.0);

      this.hueResolution = -1;
      this.saturationResolution = saturationResolution;
      this.brightnessResolution = -1;

      this.hueConstant = hueConstant;
      this.saturationConstant = brightnessConstant;
      this.brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   public void setBrightnessBased(double hueConstant, double saturationConstant)
   {
      setBrightnessBased(DEFAULT_RESOLUTION, hueConstant, saturationConstant);
   }

   public void setBrightnessBased(int brightnessResolution, double hueConstant, double saturationConstant)
   {
      MathTools.checkIfGreaterOrEqual(brightnessResolution, 1);
      MathTools.checkIfInRange(saturationConstant, 0.0, 1.0);

      this.hueResolution = -1;
      this.saturationResolution = -1;
      this.brightnessResolution = brightnessResolution;

      this.hueConstant = hueConstant;
      this.saturationConstant = saturationConstant;
      this.brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   private void updateColorPalette()
   {
      int resolution;
      if (hueResolution != -1)
         resolution = hueResolution;
      else if (saturationResolution != -1)
         resolution = saturationResolution;
      else
         resolution = brightnessResolution;

      int width = resolution;
      int height = 50;

      WritableImage image = new WritableImage(width, height);
      PixelWriter pw = image.getPixelWriter();

      for (int x = 0; x < width; x++)
      {
         for (int y = 0; y < height; y++)
         {
            pw.setColor(x, y, getColorAtIndex(x));
         }
      }

      if (PRINT_PALETTE)
      {
         // save for testing purposes
         try
         {
            ImageIO.write(SwingFXUtils.fromFXImage(image, null), "png", new File("palette.png"));
         }
         catch (IOException ex)
         {
         }
      }

      colorPalette = image;
   }

   private Color getColorAtIndex(int x)
   {
      double hue;
      if (hueResolution != -1)
         hue = 360.0 * (double) x / (double) hueResolution;
      else
         hue = 360.0 * hueConstant;

      double saturation;
      if (saturationResolution != -1)
      {
         saturation = (double) x / (double) saturationResolution;
      }
      else
         saturation = saturationConstant;

      double brightness;
      if (brightnessResolution != -1)
         brightness = (double) x / (double) brightnessResolution;
      else
         brightness = brightnessConstant;

      return Color.hsb(hue, saturation, brightness);
   }

   @Override
   public float[] getTextureLocation(Color color)
   {
      float x;
      if (hueResolution != -1)
         x = (float) (color.getHue() / 360.0);
      else if (saturationResolution != -1)
         x = (float) color.getSaturation();
      else
         x = (float) color.getBrightness();

      float y = 0.5f;

      return new float[] {x, y};
   }

   @Override
   public Image getColorPalette()
   {
      return colorPalette;
   }
}
