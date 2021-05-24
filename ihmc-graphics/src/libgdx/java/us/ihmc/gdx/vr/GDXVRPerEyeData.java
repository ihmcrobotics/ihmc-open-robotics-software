package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import org.lwjgl.openvr.VR;

/**
 * Keeps track of per eye data such as rendering surface,
 * or {@link GDXVRCamera}.
 */
public class GDXVRPerEyeData
{
   /**
    * the {@link GLFrameBuffer < Texture >} for this eye
    */
   private GLFrameBuffer<Texture> buffer;
   /**
    * a {@link TextureRegion} wrapping the color texture of the framebuffer for 2D rendering
    **/
   private final TextureRegion region;
   /**
    * the {@link GDXVRCamera} for this eye
    **/
   private final GDXVRCamera camera;
   /**
    * used internally to submit the frame buffer to OpenVR
    **/
   private final org.lwjgl.openvr.Texture texture;

   GDXVRPerEyeData(FrameBuffer buffer, TextureRegion region, GDXVRCamera cameras)
   {
      this.buffer = buffer;
      this.region = region;
      this.camera = cameras;
      this.texture = org.lwjgl.openvr.Texture.create();
      this.texture.set(buffer.getColorBufferTexture().getTextureObjectHandle(), VR.ETextureType_TextureType_OpenGL, VR.EColorSpace_ColorSpace_Gamma);
   }

   public void setFrameBuffer(GLFrameBuffer<Texture> fbo, boolean disposeOld)
   {
      if (disposeOld)
      {
         this.buffer.dispose();
      }
      this.buffer = fbo;
      this.region.setTexture(fbo.getColorBufferTexture());
      this.texture.set(buffer.getColorBufferTexture().getTextureObjectHandle(), VR.ETextureType_TextureType_OpenGL, VR.EColorSpace_ColorSpace_Gamma);
   }

   public GLFrameBuffer<Texture> getFrameBuffer()
   {
      return buffer;
   }

   public TextureRegion getRegion()
   {
      return region;
   }

   public org.lwjgl.openvr.Texture getTexture()
   {
      return texture;
   }

   public GDXVRCamera getCamera()
   {
      return camera;
   }
}
