package us.ihmc.gdx.lighting;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.euclid.tuple3D.Point3D;

public abstract class GDXLight
{
   public static final int DEPTHMAP_SIZE = 4096;
   public static final float CAMERA_NEAR = 0.1f;
   public static final float CAMERA_FAR = 100f;

   protected ShaderProgram shaderProgram = null;
   protected ModelBatch modelBatch;

   private final Camera camera;
   private final Point3D position = new Point3D();

   protected abstract void apply(final ShaderProgram shader);

   protected abstract <T extends RenderableProvider> void render(Iterable<T> renderableProviders);

   protected GDXLight(double fieldOfViewY)
   {
      camera = new PerspectiveCamera((float) fieldOfViewY, DEPTHMAP_SIZE, DEPTHMAP_SIZE);
      shaderProgram = GDXDepthMapShader.buildShaderProgram();
      modelBatch = new ModelBatch(new DefaultShaderProvider()
      {
         @Override
         protected Shader createShader(final Renderable renderable)
         {
            return new GDXDepthMapShader(renderable, shaderProgram);
         }
      });
   }

   public abstract void update();

   public Camera getCamera()
   {
      return camera;
   }

   public Point3D getPosition()
   {
      return position;
   }
}
