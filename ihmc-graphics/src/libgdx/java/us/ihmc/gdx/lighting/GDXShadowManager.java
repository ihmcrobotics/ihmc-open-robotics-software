package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;

public class GDXShadowManager
{
   private final Array<GDXLight> lights = new Array<>();
   private final ShaderProgram shader;

   /**
    * @param shader The ShaderProgram used to create the shader used by the main ModelBatch
    */
   public GDXShadowManager(ShaderProgram shader) {
      this.shader = shader;
   }

   public void addLight(GDXLight light) {
      lights.add(light);
   }

   public <T extends RenderableProvider> void render(Iterable<T> renderableProviders) {
      for (GDXLight light : lights) {
         light.render(renderableProviders);
      }

      for (GDXLight light : lights) {
         light.apply(shader);
      }
   }

   public static String getVertexShader() {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_v.glsl").readString();
   }

   public static String getFragmentShader() {
      return Gdx.files.classpath("us/ihmc/gdx/shadows/scene_f.glsl").readString();
   }
}
