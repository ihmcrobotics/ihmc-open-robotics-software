package us.ihmc.rdx;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import javax.imageio.ImageIO;

import org.apache.commons.math3.util.Pair;
import org.lwjgl.opengl.GL41;

import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;

import us.ihmc.log.LogTools;

public class GDX3DSituatedText implements RenderableProvider
{
   private static final HashMap<Pair<String, String>, Model> MODELS = new HashMap<>();
   private static final HashMap<Model, Pair<Integer, Integer>> MODEL_SIZES = new HashMap<>();
   private static final HashMap<Model, Integer> MODEL_USAGES = new HashMap<>();
   private static final Timer TIMER = new Timer();
   private static final ModelBuilder BUILDER = new ModelBuilder();
   private static final Font DEFAULT_FONT = new Font("Arial", Font.PLAIN, 72);

   //Storing some of the models (such as commonly stored ones, like numbers) helps reduce stuttering when creating GDXTextObjects
   private static boolean enableCaching = true;

   private final Model model;
   private final ModelInstance modelInstance;

   public GDX3DSituatedText(String text)
   {
      this(text, DEFAULT_FONT);
   }

   public GDX3DSituatedText(String text, String font)
   {
      this(text, new Font(font, Font.PLAIN, 72));
   }

   public GDX3DSituatedText(String text, Font font)
   {
      this.model = getModel(text, font);
      this.modelInstance = new ModelInstance(model);
      setSize(0.1f);
   }

   /**
    * Sets the size of the contained ModelInstance in meters
    * Note that this will warp/stretch the text to fill the whole area. Use {@link #setSize(float)} instead to maintain relative size
    */
   public void setSize(float width, float height)
   {
      Pair<Integer, Integer> modelSize = MODEL_SIZES.get(this.model);

      modelInstance.transform.setToScaling(width / modelSize.getKey(), height / modelSize.getValue(), 1);
   }

   public void setSize(float height)
   {
      Pair<Integer, Integer> modelSize = MODEL_SIZES.get(this.model);

      float scale = height / modelSize.getValue();

      modelInstance.transform.setToScaling(scale, scale, scale);
   }

   public void scale(float height)
   {
      Pair<Integer, Integer> modelSize = MODEL_SIZES.get(this.model);

      float scale = height / modelSize.getValue();

      modelInstance.transform.scale(scale, scale, scale);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   @Override
   protected void finalize() throws Throwable
   {
      MODEL_USAGES.computeIfPresent(model, (m, integer) -> integer--);

      //If model is unused after two minutes, remove it from the cache
      if (model != null && MODEL_USAGES.get(model) == 0)
      {
         TIMER.schedule(new TimerTask()
         {
            @Override
            public void run()
            {
               if (MODEL_USAGES.get(model) == 0)
               {
                  Pair<String, String> key = null;
                  for (Map.Entry<Pair<String, String>, Model> e : MODELS.entrySet())
                  {
                     if (e.getValue().equals(model))
                     {
                        key = e.getKey();
                        break;
                     }
                  }

                  if (key != null)
                     MODELS.remove(key);

                  MODEL_SIZES.remove(model);
                  MODEL_USAGES.remove(model);
               }
            }
         }, 2 * 60000); //2 minutes
      }

      super.finalize();
   }

   public static boolean isCachingEnabled()
   {
      return enableCaching;
   }

   /**
    * Disabling cacheing here will not clear existing items in the cache - if this is desired, call clearCache()
    */
   public static void setCachingEnabled(boolean value)
   {
      enableCaching = value;
   }

   public static void clearCache()
   {
      MODELS.clear();
      MODEL_USAGES.clear();
   }

   private static Model createModel(String text, Font font)
   { // Mostly following this method: https://stackoverflow.com/a/18800845/3503725
      //Create temporary image here in order to get Graphics2D instance
      BufferedImage image = new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB);
      Graphics2D g2d = image.createGraphics();
      g2d.setFont(font != null ? font : DEFAULT_FONT);

      FontMetrics fm = g2d.getFontMetrics();
      int width = fm.stringWidth(text);
      int height = fm.getHeight();
      g2d.dispose();

      //Create image for use in texture
      image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
      g2d = image.createGraphics();

      g2d.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION, RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
      g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g2d.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING, RenderingHints.VALUE_COLOR_RENDER_QUALITY);
      g2d.setRenderingHint(RenderingHints.KEY_DITHERING, RenderingHints.VALUE_DITHER_ENABLE);
      g2d.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
      g2d.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
      g2d.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
      g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);

      g2d.setFont(font != null ? font : DEFAULT_FONT);
      fm = g2d.getFontMetrics();
      g2d.setColor(Color.BLACK);
      g2d.drawString(text, 0, fm.getAscent());
      g2d.dispose();

      //Create model
      Texture texture;
      try
      {
         File temp = File.createTempFile("GDXTextObject", ".png");
         ImageIO.write(image, "png", temp);
         texture = new Texture(new FileHandle(temp));
      }
      catch (IOException ex)
      {
         LogTools.error("Could not create model for GDXTextObject");
         LogTools.error(ex);

         return null;
      }
      Material material = new Material(TextureAttribute.createDiffuse(texture),
                                       ColorAttribute.createSpecular(1, 1, 1, 1),
                                       new BlendingAttribute(GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA));
      long attributes = VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal | VertexAttributes.Usage.TextureCoordinates;

      Model model = BUILDER.createRect(0, 0, 0, width, 0, 0, width, height, 0, 0, height, 0, 0, 0, 1, material, attributes);
      MODEL_SIZES.put(model, new Pair<Integer, Integer>(width, height));
      return model;
   }

   private static Model getModel(String text, Font font)
   {
      Pair<String, String> modelPair = new Pair<>(text, font.getName());

      if (MODELS.containsKey(modelPair))
         return MODELS.get(modelPair);
      else
      {
         Model model = createModel(text, font);
         if (enableCaching && model != null)
         {
            MODELS.put(modelPair, model);
            MODEL_USAGES.compute(model, (m, integer) -> integer == null ? integer = 1 : integer++);
         }
         return model;
      }
   }
}
