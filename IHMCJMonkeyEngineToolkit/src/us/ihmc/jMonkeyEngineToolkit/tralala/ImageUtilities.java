package us.ihmc.jMonkeyEngineToolkit.tralala;

import java.awt.Color;
import java.awt.Font;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.image.AffineTransformOp;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetKey;
import com.jme3.asset.AssetManager;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.FastMath;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.plugins.AWTLoader;

public class ImageUtilities
{
   public static void main(String[] args)
   {
      new SimpleApplication()
      {
         @Override
         public void simpleInitApp()
         {
            test(this);
         }
      }.start();
   }

   /** Reads the colors of first column of an image and creates a gradient texture.*/
   public static void test(SimpleApplication scene)
   {
      BufferedImage image = new BufferedImage(512, 512, BufferedImage.TYPE_INT_ARGB);
      Graphics2D g2d = image.createGraphics();
      ImageUtilities.verticalGradient(image, g2d, Color.RED, Color.YELLOW, Color.GREEN, Color.BLUE);
      g2d.setFont(new Font("SansSerif", Font.PLAIN, 70));
      g2d.setColor(Color.BLACK);
      g2d.drawString("Hello World", 70, 70);
      image = ImageUtilities.rotateImage(image, FastMath.HALF_PI);
      scene.getRootNode().attachChild(Utilities.createBillboard(Utilities.getUnshadedMaterial(ImageUtilities.createTexture(image,g2d), null, BlendMode.Off, scene.getAssetManager()), 5f));
   }

   public static BufferedImage symmetrifyX(BufferedImage image, boolean useFirstHalfImage, boolean flipHorizontial)
   {
      int halfWidth = image.getWidth()/2;
      int startReadPosition = 0;
      int startWritePosition = 0;
      int endWritePosition = image.getWidth()-1;

      if (!useFirstHalfImage) startReadPosition = halfWidth;
      if (!useFirstHalfImage^flipHorizontial)//xor 
      {
         startWritePosition = halfWidth;
         endWritePosition = halfWidth;
      }

      BufferedImage returned = new BufferedImage(image.getWidth(), image.getHeight(), image.getType());
      for(int i=0;i<image.getWidth()/2;i++)
      {
         for(int j=0;j<image.getHeight();j++)
         {
            int color = image.getRGB(startReadPosition+i, j);
            returned.setRGB(startWritePosition+i, j, color);
            returned.setRGB(endWritePosition-i, j, color);
         }
      }
      return returned;
   }
   
   public static BufferedImage symmetrifyY(BufferedImage image, boolean useFirstHalfImage, boolean flipVertical)
   {
      int halfWidth = image.getHeight()/2;
      int startReadPosition = 0;
      int startWritePosition = 0;
      int endWritePosition = image.getHeight()-1;

      if (!useFirstHalfImage) startReadPosition = halfWidth;
      if (!useFirstHalfImage^flipVertical)//xor 
      {
         startWritePosition = halfWidth;
         endWritePosition = halfWidth;
      }

      BufferedImage returned = new BufferedImage(image.getWidth(), image.getHeight(), image.getType());
      for(int i=0;i<image.getWidth();i++)
      {
         for(int j=0;j<image.getHeight()/2;j++)
         {
            int color = image.getRGB(i, startReadPosition+j);
            returned.setRGB(i, startWritePosition+j, color);
            returned.setRGB(i, endWritePosition-j, color);
         }
      }
      return returned;
   }

   /** the graphic2d changes, must use the new one for future operations. use image.createGraphics(); */
   public static BufferedImage flip(BufferedImage image, boolean flipHorizontal, boolean flipVertical)
   {
      Point scale = new Point(1,1);
      Point translate = new Point(0,0);
      if (flipHorizontal)
      {
         scale.x = -1;
         translate.x = -image.getWidth();
      }
      if (flipVertical)
      {
         scale.y = -1;
         translate.y = -image.getHeight();
      }
      AffineTransform tx = AffineTransform.getScaleInstance(scale.x, scale.y);
      tx.translate(translate.x, translate.y);
      AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
      return op.filter(image, null);
   }

   public static Color[] getVerticalColors(BufferedImage image, int x, boolean hasAlpha)
   {
      Color[] colors = new Color[image.getHeight()];
      for (int i = 0; i < image.getHeight(); i++)
      {
         colors[i] = getColor(image, x, i, hasAlpha);
      }
      return colors;
   }

   public static Color getColor(BufferedImage image, int x, int y, boolean hasAlpha)
   {
      return new Color(image.getRGB(x, y), hasAlpha);
   }

   public static int getRed(int color)
   {
      return (color & 0x00ff0000) >> 16;
   }

   public static int getGreen(int color)
   {
      return (color & 0x0000ff00) >> 8;
   }

   public static int getBlue(int color)
   {
      return color & 0x000000ff;
   }

   public static int getAlpha(int color)
   {
      return (color >> 24) & 0xff;
   }

   /** the graphic2d changes, must use the new one for future operations. use image.createGraphics(); */
   public static BufferedImage rotateImage(BufferedImage image, float angle)
   {
      AffineTransform tx = new AffineTransform();
      tx.rotate(angle, image.getWidth() / 2, image.getHeight() / 2);

      AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_BILINEAR);
      return op.filter(image, null);
   }

   public static void verticalGradient(BufferedImage image, Graphics2D g2d, Color... colors)
   {
      int miniHeight = image.getHeight() / (colors.length - 1);
      for (int i = 1; i < colors.length; i++)
      {
         Point start = new Point(0, (i - 1) * miniHeight);
         Point end = new Point(0, i * miniHeight);
         gradientPaint(g2d, start, colors[i - 1], end, colors[i], start.x, start.y, image.getWidth(), end.y);
      }
   }

   public static void gradientPaint(Graphics2D g2d, Point2D startPosition, Color startColor, Point2D endPosition, Color endColor, int sx, int sy, int ex, int ey)
   {
      GradientPaint gradient = new GradientPaint(startPosition, startColor, endPosition, endColor, true);
      g2d.setPaint(gradient);
      g2d.fillRect(sx, sy, ex, ey);
   }

   /** the graphic2d changes, must use the new one for future operations.*/
   public static Pair<BufferedImage, Graphics2D> fillBackground(BufferedImage originalImage, Color color)
   {
      BufferedImage modifiedImage = new BufferedImage(originalImage.getWidth(null), originalImage.getHeight(null), BufferedImage.TYPE_INT_RGB);
      Graphics2D g = modifiedImage.createGraphics();
      g.drawImage(originalImage, 0, 0, modifiedImage.getWidth(), modifiedImage.getHeight(), color, null);

      return new Pair<BufferedImage, Graphics2D>(modifiedImage, g);
   }

   public static InputStream loadFile(String filepath, AssetManager assetManager)
   {
      return assetManager.locateAsset(new AssetKey(filepath)).openStream();
   }
   
   public static BufferedImage loadImage(String url, AssetManager assetManager)
   {
      try
      {
         BufferedImage img = ImageIO.read(loadFile(url, assetManager));
         return img;
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
         throw new IllegalArgumentException("Cant find file " + url);
      }
   }

   /** this method calls dispose on Graphics2D g*/
   public static Texture2D createTexture(BufferedImage img, Graphics2D g)
   {
      if (g != null) g.dispose();
      AWTLoader loader = new AWTLoader();
      Texture2D tex = new Texture2D(loader.load(img, true)); //changes to image parameter in buffer, affect the texture.
      tex.setMagFilter(Texture.MagFilter.Nearest);
      tex.setMinFilter(Texture.MinFilter.NearestNearestMipMap);
      return tex;
   }
}