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

/**
 * Provides a HSB-based color palette.
 * As an image is 2D, one of the three components, i.e. hue/saturation/brightness, has to be constant.
 * This implementation of {@link TextureColorPalette} is affected by a bug in JavaFX:
 * <p> <b>
 * When all the texture coordinates of a mesh have the same y-coordinate, the latter gets ignored and replaced with the value {@code 0.5f} (tested with JDK 1.8.0_112).
 * </b> </p>
 * When affected, the resulting colors of a mesh won't be as accurate as usual.
 * @author Sylvain Bertrand
 *
 */
public class TextureColorPalette2D implements TextureColorPalette
{
   /** Debug variable. When set to true, the {@link Image} used in this color palette is printed as a png file.*/
   private static final boolean PRINT_PALETTE = false;
   private static final int DEFAULT_RESOLUTION = 256;

   private int hueResolution = -1;
   private int saturationResolution = -1;
   private int brightnessResolution = -1;

   private double hueConstant = Double.NaN;
   private double saturationConstant = Double.NaN;
   private double brightnessConstant = Double.NaN;

   private Image colorPalette;

   /**
    * Creates a color palette with the brightness constant and set to {@code 1.0}. The two other components have a resolution of {@value #DEFAULT_RESOLUTION}.
    */
   public TextureColorPalette2D()
   {
      setHueSaturationBased(1.0);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation component.
    * @param brightnessConstant the new constant value for the brightness component.
    */
   public void setHueSaturationBased(double brightnessConstant)
   {
      setHueSaturationBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, brightnessConstant);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation.
    * @param hueResolution the new resolution to use for the hue component.
    * @param saturationResolution the new resolution to use for the saturation component.
    * @param brightnessConstant the new constant value for the brightness component.
    */
   public void setHueSaturationBased(int hueResolution, int saturationResolution, double brightnessConstant)
   {
      MathTools.checkIfGreaterOrEqual(hueResolution, 1);
      MathTools.checkIfGreaterOrEqual(saturationResolution, 1);
      MathTools.checkIntervalContains(brightnessConstant, 0.0, 1.0);

      this.hueResolution = hueResolution;
      this.saturationResolution = saturationResolution;
      this.brightnessResolution = -1;

      this.hueConstant = Double.NaN;
      this.saturationConstant = Double.NaN;
      this.brightnessConstant = brightnessConstant;

      updateColorPalette();
   }

   /**
    * Changes this color palette to allow variation in hue and brightness. The two other components have a resolution of {@value #DEFAULT_RESOLUTION}.
    * @param saturationConstant the new constant value for the saturation component.
    */
   public void setHueBrightnessBased(double saturationConstant)
   {
      setHueBrightnessBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, saturationConstant);
   }

   /**
    * Changes this color palette to allow variation in hue and saturation.
    * @param hueResolution the new resolution to use for the hue component.
    * @param brightnessResolution the new resolution to use for the brightness component.
    * @param saturationConstant the new constant value for the saturation component.
    */
   public void setHueBrightnessBased(int hueResolution, int brightnessResolution, double saturationConstant)
   {
      MathTools.checkIfGreaterOrEqual(hueResolution, 1);
      MathTools.checkIfGreaterOrEqual(brightnessResolution, 1);
      MathTools.checkIntervalContains(saturationConstant, 0.0, 1.0);

      this.hueResolution = hueResolution;
      this.saturationResolution = -1;
      this.brightnessResolution = brightnessResolution;

      this.hueConstant = Double.NaN;
      this.saturationConstant = saturationConstant;
      this.brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   /**
    * Changes this color palette to allow variation in saturation and brightness. The two other components have a resolution of {@value #DEFAULT_RESOLUTION}.
    * @param hueConstant the new constant value for the hue component.
    */
   public void setSaturationBrightnessBased(double hueConstant)
   {
      setSaturationBrightnessBased(DEFAULT_RESOLUTION, DEFAULT_RESOLUTION, hueConstant);
   }

   /**
    * Changes this color palette to allow variation in saturation and brightness.
    * @param saturationResolution the new resolution to use for the saturation component.
    * @param brightnessResolution the new resolution to use for the brightness component.
    * @param hueConstant the new constant value for the hue component.
    */
   public void setSaturationBrightnessBased(int saturationResolution, int brightnessResolution, double hueConstant)
   {
      MathTools.checkIfGreaterOrEqual(saturationResolution, 1);
      MathTools.checkIfGreaterOrEqual(brightnessResolution, 1);
      MathTools.checkIntervalContains(hueConstant, 0.0, 1.0);

      this.hueResolution = -1;
      this.saturationResolution = saturationResolution;
      this.brightnessResolution = brightnessResolution;

      this.hueConstant = hueConstant;
      this.saturationConstant = Double.NaN;
      this.brightnessConstant = Double.NaN;

      updateColorPalette();
   }

   private void updateColorPalette()
   {
      int width = hueResolution != -1 ? hueResolution : saturationResolution;
      int height = brightnessResolution != -1 ? brightnessResolution : saturationResolution;

      WritableImage image = new WritableImage(width, height);
      PixelWriter pw = image.getPixelWriter();

      for (int x = 0; x < width; x++)
      {
         for (int y = 0; y < height; y++)
         {
            pw.setColor(x, y, getColorAtLocation(x, y));
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

   private Color getColorAtLocation(int x, int y)
   {
      double hue;
      if (hueResolution != -1)
         hue = 360.0 * (double) x / (double) hueResolution;
      else
         hue = 360.0 * hueConstant;

      double saturation;
      if (saturationResolution != -1)
      {
         int index = hueResolution == -1 ? x : y;
         saturation = (double) index / (double) saturationResolution;
      }
      else
         saturation = saturationConstant;

      double brightness;
      if (brightnessResolution != -1)
         brightness = (double) y / (double) brightnessResolution;
      else
         brightness = brightnessConstant;

      return Color.hsb(hue, saturation, brightness);
   }

   @Override
   public float[] getTextureLocation(Color color)
   {
      float x = (float) (hueResolution != -1 ? color.getHue() / 360.0 : color.getSaturation());
      float y = (float) (brightnessResolution != -1 ? color.getBrightness() : color.getSaturation());

      return new float[] {x, y};
   }

   @Override
   public Image getColorPalette()
   {
      return colorPalette;
   }
}
