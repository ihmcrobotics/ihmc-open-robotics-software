package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.math.Vector3;

public class GDXDirectionalLight extends GDXLight
{
   protected Vector3 direction;
   private FrameBuffer framebuffer;
   private Texture depthMap;

   public GDXDirectionalLight(Vector3 position, Vector3 direction)
   {
      this.position = position;
      this.direction = direction;
   }

   @Override
   protected <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new FrameBuffer(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }
      framebuffer.begin();
      Gdx.gl.glClearColor(0, 0, 0, 1);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", camera.far);
      shaderProgram.setUniformf("u_lightPosition", camera.position);
      shaderProgram.end();

      modelBatch.begin(camera);
      modelBatch.render(renderableProviders);
      modelBatch.end();

      //PerryUtilsDONOTCOMMIT.screenshot(framebuffer, "depth");
      framebuffer.end();
      depthMap = framebuffer.getColorBufferTexture();
   }

   @Override
   protected void apply(ShaderProgram shader)
   {
      shader.begin();
      final int textureNum = depthMap.getTextureObjectHandle();
      depthMap.bind(textureNum);
      shader.setUniformf("u_type", 1);
      shader.setUniformi("u_depthMapDir", textureNum);
      shader.setUniformMatrix("u_lightTrans", camera.combined);
      shader.setUniformf("u_cameraFar", camera.far);
      shader.setUniformf("u_lightPosition", camera.position);
      shader.end();
   }

   @Override
   public void init()
   {
      super.init();

      this.camera = new PerspectiveCamera(120f, DEPTHMAP_SIZE, DEPTHMAP_SIZE);
      this.camera.near = CAMERA_NEAR;
      this.camera.far = CAMERA_FAR;
      this.camera.position.set(position);
      this.camera.lookAt(direction);

      this.camera.update();
   }

   public Vector3 getDirection()
   {
      return direction;
   }

   public void setDirection(Vector3 direction)
   {
      this.direction = direction;
   }
}
