package us.ihmc.rdx;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.util.HashMap;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.math.Matrix4;
import org.bytedeco.javacpp.BytePointer;
import org.lwjgl.opengl.GL41;

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

/**
 * Provides the ability to put text in the 3D scene.
 * This text is only readable from the front and invisible from the back.
 *
 * TODO: This class uses AWT to draw text.
 *   Instead we should try using ImGui to draw the text, which would allow us
 *   to use a consistent font with the rest of the UI, do FreeType rendering,
 *   and potentially free up resource usage of this class.
 */
public class RDX3DSituatedText implements RenderableProvider
{
   public static final Font DEFAULT_FONT = new Font("Dialog", Font.PLAIN, 72);
   public static final Color DEFAULT_COLOR = Color.BLACK;
   public static final float DEFAULT_HEIGHT = 0.1f;

   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final HashMap<String, RDX3DSituatedTextData> textDataMap = new HashMap<>();
   private RDX3DSituatedTextData textData;
   private String currentText;
   private final Font awtFont;
   private final Color awtColor;
   private final float textHeightMeters;

   public RDX3DSituatedText(String text)
   {
      this(text, DEFAULT_FONT, DEFAULT_COLOR, DEFAULT_HEIGHT);
   }

   public RDX3DSituatedText(String text, float textHeightMeters)
   {
      this(text, DEFAULT_FONT, DEFAULT_COLOR, textHeightMeters);
   }

   public RDX3DSituatedText(String text, Color awtColor, float textHeightMeters)
   {

      this(text, DEFAULT_FONT, awtColor, textHeightMeters);
   }

   public RDX3DSituatedText(String text, Font awtFont, Color awtColor, float textHeightMeters)
   {
      this.awtFont = awtFont;
      this.awtColor = awtColor;
      this.textHeightMeters = textHeightMeters;
      setText(text);
   }

   public void setText(String text)
   {
      this.currentText = text;
      textData = textDataMap.get(text);

      if (textData == null)
      {
         setTextWithoutCache(text);
         textDataMap.put(text, textData);
      }
   }

   public RDX3DSituatedTextData setTextWithoutCache(String text)
   {
      this.currentText = text;

      // Mostly following this method: https://stackoverflow.com/a/18800845/3503725
      // Create temporary image here in order to get Graphics2D instance
      BufferedImage onePixelImage = new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB);
      Graphics2D awtGraphics2D = onePixelImage.createGraphics();
      awtGraphics2D.setFont(awtFont != null ? awtFont : DEFAULT_FONT);
      FontMetrics awtFontMetrics = awtGraphics2D.getFontMetrics();
      int textWidthPixels = text.isEmpty() ? 1 : awtFontMetrics.stringWidth(text);
      int textHeightPixels = awtFontMetrics.getHeight();
      awtGraphics2D.dispose();

      // Create image for use in texture
      BufferedImage bufferedImageRGBA8 = new BufferedImage(textWidthPixels, textHeightPixels, BufferedImage.TYPE_INT_ARGB);
      awtGraphics2D = bufferedImageRGBA8.createGraphics();
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_ALPHA_INTERPOLATION, RenderingHints.VALUE_ALPHA_INTERPOLATION_QUALITY);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_COLOR_RENDERING, RenderingHints.VALUE_COLOR_RENDER_QUALITY);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_DITHERING, RenderingHints.VALUE_DITHER_ENABLE);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
      awtGraphics2D.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
      awtGraphics2D.setFont(awtFont != null ? awtFont : DEFAULT_FONT);
      awtFontMetrics = awtGraphics2D.getFontMetrics();
      awtGraphics2D.setColor(awtColor);
      int x = 0;
      int y = awtFontMetrics.getAscent();
      if (!text.isEmpty())
         awtGraphics2D.drawString(text, x, y);
      awtGraphics2D.dispose();

      Pixmap pixmap = new Pixmap(textWidthPixels, textHeightPixels, Pixmap.Format.RGBA8888);
      BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
      DataBuffer dataBuffer = bufferedImageRGBA8.getRaster().getDataBuffer();
      for (int i = 0; i < dataBuffer.getSize(); i++)
      {
         rgba8888BytePointer.putInt(i * Integer.BYTES, dataBuffer.getElem(i));
      }

      Texture libGDXTexture = new Texture(new PixmapTextureData(pixmap, null, false, false));
      Material material = new Material(TextureAttribute.createDiffuse(libGDXTexture),
                                       ColorAttribute.createSpecular(1, 1, 1, 1),
                                       new BlendingAttribute(GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA));
      long attributes = VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal | VertexAttributes.Usage.TextureCoordinates;

      float textWidthMeters = textHeightMeters * (float) textWidthPixels / (float) textHeightPixels;

      float x00 = 0.0f;
      float y00 = 0.0f;
      float z00 = 0.0f;
      float x10 = textWidthMeters;
      float y10 = 0.0f;
      float z10 = 0.0f;
      float x11 = textWidthMeters;
      float y11 = textHeightMeters;
      float z11 = 0.0f;
      float x01 = 0.0f;
      float y01 = textHeightMeters;
      float z01 = 0.0f;
      float normalX = 0.0f;
      float normalY = 0.0f;
      float normalZ = 1.0f;
      Model model = modelBuilder.createRect(x00, y00, z00, x10, y10, z10, x11, y11, z11, x01, y01, z01, normalX, normalY, normalZ, material, attributes);
      ModelInstance modelInstance = new ModelInstance(model);
      textData = new RDX3DSituatedTextData(pixmap, rgba8888BytePointer, libGDXTexture, model, modelInstance);

      return textData;
   }

   public Matrix4 getModelTransform()
   {
      return textData.getModelInstance().transform;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      textData.getModelInstance().getRenderables(renderables, pool);
   }

   public String getCurrentText()
   {
      return currentText;
   }
}
