package us.ihmc.gdx;

import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.log.LogTools;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;

public class GDXTextObject implements RenderableProvider
{
   private static final HashMap<String, Model> MODELS = new HashMap<>();
   private static final HashMap<Model, Integer> MODEL_USAGES = new HashMap<>();
   private static final Timer TIMER = new Timer();

   private static final ModelBuilder BUILDER = new ModelBuilder();
   private static final Font FONT = new Font("Arial", Font.PLAIN, 72);

   //Storing some of the models (such as commonly stored ones, like numbers) helps reduce stuttering when creating GDXTextObjects
   private static boolean enableCacheing = true;

   public static boolean isCacheingEnabled() {
      return enableCacheing;
   }

   /**
    * Disabling cacheing here will not clear existing items in the cache - if this is desired, call clearCache()
    */
   public static void setCacheingEnabled(boolean value) {
      enableCacheing = value;
   }

   public static void clearCache() {
      MODELS.clear();
   }

   private static Model createModel(String text) { // Mostly following this method: https://stackoverflow.com/a/18800845/3503725
      //Create temporary image here in order to get Graphics2D instance
      BufferedImage image = new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB);
      Graphics2D g2d = image.createGraphics();
      g2d.setFont(FONT);

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

      g2d.setFont(FONT);
      fm = g2d.getFontMetrics();
      g2d.setColor(Color.BLACK);
      g2d.drawString(text, 0, fm.getAscent());
      g2d.dispose();

      //Create model
      Texture texture;
      try
      {
         File temp = File.createTempFile("GDXTextObject", "png");
         texture = new Texture(new FileHandle(temp));
      } catch (IOException ex) {
         LogTools.error("Could not create model for GDXTextObject");
         LogTools.error(ex);

         return null;
      }
      Material material = new Material(TextureAttribute.createDiffuse(texture));
      long attributes = VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal | VertexAttributes.Usage.TextureCoordinates;

      return BUILDER.createBox(width, height, 1.0f, material, attributes);
   }

   private static Model getModel(String text) {
      if (MODELS.containsKey(text))
         return MODELS.get(text);
      else
      {
         Model model = createModel(text);
         if (enableCacheing && model != null)
         {
            MODELS.put(text, model);
            MODEL_USAGES.compute(model, (m, integer) -> integer == null ? integer = 1 : integer++);
         }
         return model;
      }
   }

   private Model model;
   public ModelInstance modelInstance;

   public GDXTextObject(String text) {
      this.model = getModel(text);
      this.modelInstance = new ModelInstance(model);
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
      if (model != null && MODEL_USAGES.get(model) == 0) {
         TIMER.schedule(new TimerTask()
         {
            @Override
            public void run()
            {
               if (MODEL_USAGES.get(model) == 0)
                  MODEL_USAGES.remove(model);
            }
         }, 2 * 60000); //2 minutes
      }

      super.finalize();
   }
}
