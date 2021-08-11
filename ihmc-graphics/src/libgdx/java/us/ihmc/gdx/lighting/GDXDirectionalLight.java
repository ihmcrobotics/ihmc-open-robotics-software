package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.tools.GDXTools;

public class GDXDirectionalLight extends GDXLight
{
   private final Vector3D direction = new Vector3D();
   private FrameBuffer framebuffer;
   private Texture depthMap;

   public GDXDirectionalLight()
   {
      super(150.0);
      update();
   }

   public GDXDirectionalLight(Point3D position, Vector3D direction)
   {
      super(150.0);
      getPosition().set(position);
      this.direction.set(direction);
      update();
   }

   @Override
   protected <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new FrameBuffer(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }

      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", getCamera().far);
      shaderProgram.setUniformf("u_lightPosition_x", getPosition().getX32());
      shaderProgram.setUniformf("u_lightPosition_y", getPosition().getY32());
      shaderProgram.setUniformf("u_lightPosition_z", getPosition().getZ32());
      shaderProgram.end();

      framebuffer.begin();
      Gdx.gl.glClearColor(0, 0, 0, 1);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      modelBatch.begin(getCamera());
      modelBatch.render(renderableProviders);
      modelBatch.end();

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
      shader.setUniformMatrix("u_lightTrans", getCamera().combined);
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
      getCamera().lookAt(direction.getX32(), direction.getY32(), direction.getZ32());
      getCamera().update();
   }

   public Vector3D getDirection()
   {
      return direction;
   }
}
