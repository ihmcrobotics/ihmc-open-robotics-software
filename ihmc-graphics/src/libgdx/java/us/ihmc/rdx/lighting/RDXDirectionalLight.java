package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXDirectionalLight
{
   public static final int DEPTHMAP_SIZE = 4096;
   public static final float CAMERA_NEAR = 0.1f;
   public static final float CAMERA_FAR = 100f;

   protected ModelBatch modelBatch;

   private final Camera camera;
   private final Point3D position = new Point3D();
   private final Vector3D direction = new Vector3D();
   private FrameBuffer framebuffer;
   private Texture depthMap;
   private DirectionalLight attribute;

   public RDXDirectionalLight()
   {
      camera = new PerspectiveCamera(150.0f, DEPTHMAP_SIZE, DEPTHMAP_SIZE);
      modelBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(final Renderable renderable)
         {
            return new RDXDepthMapShader(renderable);
         }
      });
      update();
   }

   public void update()
   {
      camera.near = CAMERA_NEAR;
      camera.far = CAMERA_FAR;
      LibGDXTools.toLibGDX(position, camera.position);
      camera.lookAt(direction.getX32(), direction.getY32(), direction.getZ32());
      camera.update();
      if (attribute != null)
         LibGDXTools.toLibGDX(direction, attribute.direction);
   }

   public <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new FrameBuffer(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }

      ShaderProgram shaderProgram = RDXDepthMapShader.getOrLoadShaderProgram();
      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", camera.far);
      shaderProgram.setUniformf("u_lightPosition_x", getPosition().getX32());
      shaderProgram.setUniformf("u_lightPosition_y", getPosition().getY32());
      shaderProgram.setUniformf("u_lightPosition_z", getPosition().getZ32());
      shaderProgram.end();

      framebuffer.begin();
      GL41.glClearColor(0, 0, 0, 1);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

      modelBatch.begin(camera);
      modelBatch.render(renderableProviders);
      modelBatch.end();

      framebuffer.end();
      depthMap = framebuffer.getColorBufferTexture();
   }

   public void apply(ShaderProgram shader)
   {
      shader.begin();
      final int textureNum = depthMap.getTextureObjectHandle();
      depthMap.bind(textureNum);
      shader.setUniformf("u_type", 1);
      shader.setUniformi("u_depthMapDir", textureNum);
      shader.setUniformMatrix("u_lightTrans", camera.combined);
      shader.setUniformf("u_cameraFar", camera.far);
      shader.setUniformf("u_lightPosition_x", getPosition().getX32());
      shader.setUniformf("u_lightPosition_y", getPosition().getY32());
      shader.setUniformf("u_lightPosition_z", getPosition().getZ32());
      shader.end();
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Vector3D getDirection()
   {
      return direction;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public void setAttribute(DirectionalLight attribute)
   {
      this.attribute = attribute;
   }

   public DirectionalLight getAttribute()
   {
      return attribute;
   }
}
