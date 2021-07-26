package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Cubemap;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.math.Vector3;

public class GDXPointLight extends GDXLight
{
   private AccessibleFrameBufferCubemap framebuffer;
   private Cubemap depthMap;

   public GDXPointLight(Vector3 position)
   {
      this.position = position;
   }

   @Override
   protected <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new AccessibleFrameBufferCubemap(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }

      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", camera.far);
      shaderProgram.setUniformf("u_lightPosition_x", position.x);
      shaderProgram.setUniformf("u_lightPosition_y", position.y);
      shaderProgram.setUniformf("u_lightPosition_z", position.z);
      shaderProgram.end();

      for (int s = 0; s <= 5; s++)
      {
         final Cubemap.CubemapSide side = Cubemap.CubemapSide.values()[s];
         framebuffer.begin();
         framebuffer.bindSide(side, camera);
         Gdx.gl.glClearColor(0, 0, 0, 1);
         Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

         modelBatch.begin(camera);
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
      shader.setUniformMatrix("u_lightTrans", camera.combined);
      shader.setUniformf("u_cameraFar", camera.far);
      shader.setUniformf("u_lightPosition_x", position.x);
      shader.setUniformf("u_lightPosition_y", position.y);
      shader.setUniformf("u_lightPosition_z", position.z);

      shader.end();
   }

   @Override
   public void init()
   {
      super.init();

      this.camera = new PerspectiveCamera(90f, DEPTHMAP_SIZE, DEPTHMAP_SIZE);
      this.camera.near = CAMERA_NEAR;
      this.camera.far = CAMERA_FAR;
      this.camera.position.set(position);

      this.camera.update();
   }
}
