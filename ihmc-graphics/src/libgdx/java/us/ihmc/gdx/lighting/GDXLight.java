package us.ihmc.gdx.lighting;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.math.Vector3;

public abstract class GDXLight
{
   public static final int DEPTHMAP_SIZE = 4096;
   public static final float CAMERA_NEAR = 0.1f;
   public static final float CAMERA_FAR = 100f;

   protected ShaderProgram shaderProgram = null;
   protected ModelBatch modelBatch = null;

   protected Camera camera;
   protected Vector3 position = new Vector3();

   protected abstract void apply(final ShaderProgram shader);

   protected abstract <T extends RenderableProvider> void render(Iterable<T> renderableProviders);

   /**
    * init() should be called after the light is created. It initializes the shaders, depthmap, etc.
    */
   public void init()
   {
      if (modelBatch == null)
      {
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
   }

   public abstract void update();

   public boolean isInitialized()
   {
      return modelBatch != null;
   }

   public Camera getCamera()
   {
      return camera;
   }

   public void setCamera(Camera camera)
   {
      this.camera = camera;
   }

   public Vector3 getPosition()
   {
      return position;
   }

   public void setPosition(Vector3 position)
   {
      this.position = position; //changing the position of a light causes rendering problems
   }
}
