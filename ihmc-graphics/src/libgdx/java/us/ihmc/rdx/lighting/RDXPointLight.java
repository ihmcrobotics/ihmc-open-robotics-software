package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXPointLight
{
   public static final int DEPTHMAP_SIZE = 4096;
   public static final float CAMERA_NEAR = 0.1f;
   public static final float CAMERA_FAR = 100f;

   protected ModelBatch modelBatch;

   private final Camera camera;
   private final Point3D position = new Point3D();

   private AccessibleFrameBufferCubemap framebuffer;
   private Cubemap depthMap;
   private PointLight attribute;

   public RDXPointLight()
   {
      camera = new PerspectiveCamera(90.0f, DEPTHMAP_SIZE, DEPTHMAP_SIZE);
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
      camera.update();
      if (attribute != null)
         LibGDXTools.toLibGDX(position, attribute.position);
   }

   public <T extends RenderableProvider> void render(Iterable<T> renderableProviders)
   {
      if (framebuffer == null)
      {
         framebuffer = new AccessibleFrameBufferCubemap(Pixmap.Format.RGBA8888, DEPTHMAP_SIZE, DEPTHMAP_SIZE, true);
      }

      ShaderProgram shaderProgram = RDXDepthMapShader.getOrLoadShaderProgram();
      shaderProgram.begin();
      shaderProgram.setUniformf("u_cameraFar", camera.far);
      shaderProgram.setUniformf("u_lightPosition_x", position.getX32());
      shaderProgram.setUniformf("u_lightPosition_y", position.getY32());
      shaderProgram.setUniformf("u_lightPosition_z", position.getZ32());
      shaderProgram.end();

      framebuffer.begin();

      for (int sideIndex = 0; sideIndex < 6; sideIndex++)
      {
         final Cubemap.CubemapSide side = Cubemap.CubemapSide.values()[sideIndex];
         framebuffer.bindSide(side, camera);
         GL41.glClearColor(0, 0, 0, 1);
         GL41.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

         modelBatch.begin(camera);
         modelBatch.render(renderableProviders);
         modelBatch.end();
      }

      framebuffer.end();
      depthMap = framebuffer.getColorBufferTexture();
   }

   public void apply(ShaderProgram shader)
   {
      shader.begin();
      final int textureNum = depthMap.getTextureObjectHandle();
      depthMap.bind(textureNum);
      shader.setUniformf("u_type", 2);
      shader.setUniformi("u_depthMapCube", textureNum);
      //shader.setUniformMatrix("u_lightTrans", camera.combined);
      shader.setUniformf("u_cameraFar", camera.far);
      shader.setUniformf("u_lightPosition_x", position.getX32());
      shader.setUniformf("u_lightPosition_y", position.getY32());
      shader.setUniformf("u_lightPosition_z", position.getZ32());

      shader.end();
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public void setAttribute(PointLight attribute)
   {
      this.attribute = attribute;
   }

   public PointLight getAttribute()
   {
      return attribute;
   }
}
