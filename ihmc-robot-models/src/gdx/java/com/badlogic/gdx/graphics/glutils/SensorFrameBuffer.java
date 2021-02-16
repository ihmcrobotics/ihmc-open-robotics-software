package com.badlogic.gdx.graphics.glutils;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;

public class SensorFrameBuffer extends GLFrameBuffer<Texture>
{
   private Pixmap colorPixmap;
   private FloatTextureData depthTextureData;
   private Texture colorTexture;
   private Texture depthTexture;

   protected SensorFrameBuffer(GLFrameBufferBuilder<? extends GLFrameBuffer<Texture>> bufferBuilder)
   {
      super(bufferBuilder);
   }

   @Override
   protected Texture createTexture(FrameBufferTextureAttachmentSpec attachmentSpec)
   {
      Texture texture;
      if (attachmentSpec.format == Pixmap.Format.toGlFormat(Pixmap.Format.RGBA8888))
      {
         colorPixmap = new Pixmap(bufferBuilder.width, bufferBuilder.height, Pixmap.Format.RGBA8888);
         PixmapTextureData pixmapTextureData = new PixmapTextureData(colorPixmap, null, false, false);
         texture = colorTexture = new Texture(pixmapTextureData);
         texture.setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
         texture.setWrap(Texture.TextureWrap.ClampToEdge, Texture.TextureWrap.ClampToEdge);
      }
      else
      {
         attachmentSpec.isGpuOnly = true;
         depthTextureData = new FloatTextureData(bufferBuilder.width,
                                     bufferBuilder.height,
                                     attachmentSpec.internalFormat,
                                     attachmentSpec.format,
                                     attachmentSpec.type,
                                     attachmentSpec.isGpuOnly);
         texture = depthTexture = new Texture(depthTextureData);
         texture.setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
         texture.setWrap(Texture.TextureWrap.ClampToEdge, Texture.TextureWrap.ClampToEdge);
      }
      return texture;
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

   public Pixmap getColorPixmap()
   {
      return colorPixmap;
   }

   public FloatTextureData getDepthTextureData()
   {
      return depthTextureData;
   }

   public Texture getColorTexture()
   {
      return colorTexture;
   }

   public Texture getDepthTexture()
   {
      return depthTexture;
   }
}
