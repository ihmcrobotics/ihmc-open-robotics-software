package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Cubemap;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.gdx.tools.GDXTools;

public class GDXPointLight extends GDXLight
{
   private AccessibleFrameBufferCubemap framebuffer;
   private Cubemap depthMap;

   public GDXPointLight()
   {
      super(90.0);
      update();
   }

   public GDXPointLight(double x, double y, double z)
   {
      super(90.0);
      getPosition().set(x, y, z);
      update();
   }

   @Override
   protected <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new AccessibleFrameBufferCubemap(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }

      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", getCamera().far);
      shaderProgram.setUniformf("u_lightPosition_x", getPosition().getX32());
      shaderProgram.setUniformf("u_lightPosition_y", getPosition().getY32());
      shaderProgram.setUniformf("u_lightPosition_z", getPosition().getZ32());
      shaderProgram.end();

      framebuffer.begin();

      for (int s = 0; s <= 5; s++)
      {
         final Cubemap.CubemapSide side = Cubemap.CubemapSide.values()[s];
         framebuffer.bindSide(side, getCamera());
         Gdx.gl.glClearColor(0, 0, 0, 1);
         Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

         modelBatch.begin(getCamera());
         modelBatch.render(renderableProviders);
         modelBatch.end();
      }

      framebuffer.end();
      depthMap = framebuffer.getColorBufferTexture();
   }

   @Override
   protected void apply(ShaderProgram shader)
   {
      shader.begin();
      final int textureNum = depthMap.getTextureObjectHandle();
      depthMap.bind(textureNum);
      shader.setUniformf("u_type", 2);
      shader.setUniformi("u_depthMapCube", textureNum);
      //shader.setUniformMatrix("u_lightTrans", camera.combined);
      shader.setUniformf("u_cameraFar", getCamera().far);
      shader.setUniformf("u_lightPosition_x", getPosition().getX32());
      shader.setUniformf("u_lightPosition_y", getPosition().getY32());
      shader.setUniformf("u_lightPosition_z", getPosition().getZ32());

      shader.end();
   }

   @Override
   public void update()
   {
      getCamera().near = CAMERA_NEAR;
      getCamera().far = CAMERA_FAR;
      GDXTools.toGDX(getPosition(), getCamera().position);
      getCamera().update();
   }
}
