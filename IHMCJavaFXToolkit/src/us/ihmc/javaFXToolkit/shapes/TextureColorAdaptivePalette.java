package us.ihmc.javaFXToolkit.shapes;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
import us.ihmc.tools.io.printing.PrintTools;

public class TextureColorAdaptivePalette implements TextureColorPalette
{
   private static final boolean PRINT_PALETTE = false;
   private static final int DEFAULT_PALETTE_SIZE = 1024;

   private WritableImage colorPalette;
   private int paletteSize;
   private int pixelSize;

   private final List<Color> colorList = new ArrayList<>();
   private final Map<Color, Integer> registeredColors = new HashMap<>();
   private final boolean autoClear;

   public TextureColorAdaptivePalette()
   {
      this(DEFAULT_PALETTE_SIZE, 1, true);
   }

   public TextureColorAdaptivePalette(boolean autoClear)
   {
      this(DEFAULT_PALETTE_SIZE, 1, autoClear);
   }

   public TextureColorAdaptivePalette(int paletteSize)
   {
      this(paletteSize, 1, true);
   }

   public TextureColorAdaptivePalette(int paletteSize, boolean autoClear)
   {
      this(paletteSize, 1, autoClear);
   }

   public TextureColorAdaptivePalette(int paletteSize, int colorPixelSize)
   {
      this(paletteSize, colorPixelSize, true);
   }

   public TextureColorAdaptivePalette(int paletteSize, int colorPixelSize, boolean autoClear)
   {
      if (colorPixelSize >= paletteSize)
         throw new RuntimeException("The size of pixels has to me smaller than the size of the palette.");

      this.autoClear = autoClear;
      this.paletteSize = paletteSize;
      this.pixelSize = colorPixelSize;
   }

   public void clearPalette()
   {
      colorPalette = null;
      colorList.clear();
      registeredColors.clear();
   }

   private void initializePalette()
   {
      colorPalette = new WritableImage(paletteSize, paletteSize);
      colorList.clear();
      registeredColors.clear();
   }

   @Override
   public float[] getTextureLocation(Color color)
   {
      if (colorPalette == null)
         initializePalette();

      Integer pixelIndex = registeredColors.get(color);
      if (pixelIndex != null)
      {
         return getTextureLocation(pixelIndex);
      }
      else
      {
         int newPixelIndex = colorList.size();
         if (newPixelIndex <= paletteSize * paletteSize)
         {
            colorList.add(color);
            registeredColors.put(color, newPixelIndex);
            writePixel(newPixelIndex, color);
            return getTextureLocation(newPixelIndex);
         }
         else
         {
            PrintTools.error(this, "Reached maximum capacity of the palette. Next colors will be wrong.");
            return new float[] {0.0f, 0.0f};
         }
      }
   }

   private float[] getTextureLocation(int pixelIndex)
   {
      int scaledWidth = (int) colorPalette.getWidth() / pixelSize;
      int scaledHeight = (int) colorPalette.getHeight() / pixelSize;
      float x = (float) (Math.floorMod(pixelIndex, scaledWidth)) / (float) scaledWidth;
      x += 0.5f * (float) pixelSize / (float) colorPalette.getWidth();
      float y = (float) (Math.floorDiv(pixelIndex, scaledWidth)) / (float) scaledHeight;
      y += 0.5f * (float) pixelSize / (float) colorPalette.getHeight();
      return new float[] {x, y};
   }

   private void writePixel(int pixelIndex, Color color)
   {
      int width = (int) colorPalette.getRequestedWidth();
      int xStart = Math.floorMod(pixelIndex * pixelSize, width);
      int yStart = Math.floorDiv(pixelIndex * pixelSize, width) * pixelSize;
      for (int x = xStart; x < xStart + pixelSize; x++)
      {
         for (int y = yStart; y < yStart + pixelSize; y++)
         {
            colorPalette.getPixelWriter().setColor(x, y, color);
         }
      }
   }

   @Override
   public Image getColorPalette()
   {
      WritableImage ret = colorPalette;
      if (PRINT_PALETTE)
      {
         // save for testing purposes
         try
         {
            ImageIO.write(SwingFXUtils.fromFXImage(colorPalette, null), "png", new File("palette.png"));
         }
         catch (IOException ex)
         {
            ex.printStackTrace();
         }
      }
      if (autoClear)
         clearPalette();
      return ret;
   }
}
