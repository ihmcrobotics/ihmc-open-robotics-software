package com.badlogic.gdx.graphics.glutils;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.TextureData;

public class SensorFrameBuffer extends GLFrameBuffer<Texture>
{
   protected SensorFrameBuffer(GLFrameBufferBuilder<? extends GLFrameBuffer<Texture>> bufferBuilder)
   {
      super(bufferBuilder);
   }

   @Override
   protected Texture createTexture(FrameBufferTextureAttachmentSpec attachmentSpec)
   {
      TextureData data;
      if (attachmentSpec.format == Pixmap.Format.toGlFormat(Pixmap.Format.RGBA8888))
      {
         Pixmap pixmap = new Pixmap(bufferBuilder.width, bufferBuilder.height, Pixmap.Format.RGBA8888);
         data = new PixmapTextureData(pixmap, null, false, false);
      }
      else
      {
         data = new FloatTextureData(bufferBuilder.width,
                                     bufferBuilder.height,
                                     attachmentSpec.internalFormat,
                                     attachmentSpec.format,
                                     attachmentSpec.type,
                                     attachmentSpec.isGpuOnly);
      }
      Texture result = new Texture(data);
      result.setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
      result.setWrap(Texture.TextureWrap.ClampToEdge, Texture.TextureWrap.ClampToEdge);
      return result;
   }

   @Override
   protected void disposeColorTexture(Texture colorTexture)
   {
      colorTexture.dispose();
   }

   @Override
   protected void attachFrameBufferColorTexture(Texture texture)
   {
      Gdx.gl20.glFramebufferTexture2D(GL20.GL_FRAMEBUFFER, GL20.GL_COLOR_ATTACHMENT0, GL20.GL_TEXTURE_2D, texture.getTextureObjectHandle(), 0);
   }

   public static void unbind()
   {
      GLFrameBuffer.unbind();
   }
}
