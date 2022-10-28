package us.ihmc.rdx.simulation;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.glutils.GLOnlyTextureData;
import org.lwjgl.opengl.GL41;

/**
 * TODO: Work in progress. Develop a general purpose shader for speeding up depth sensor simulation.
 */
public class DepthSensorFrameBuffer
{
   private Texture colorTexture;
   private Pixmap colorPixmap;

   public void create(int width, int height)
   {
      int internalFormat;
      int format;
      int type;
      int levelOfDetail;

      levelOfDetail = 0; // not mattering; for uploading an image
      internalFormat = GL41.GL_RGBA8;
      format = GL41.GL_RGBA;
      type = GL41.GL_UNSIGNED_BYTE;
      GLOnlyTextureData colorTextureData = new GLOnlyTextureData(width, height, levelOfDetail, internalFormat, format, type);
      colorTexture = new Texture(colorTextureData);
      colorTexture.setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
      colorTexture.setWrap(Texture.TextureWrap.ClampToEdge, Texture.TextureWrap.ClampToEdge);



   }
}
