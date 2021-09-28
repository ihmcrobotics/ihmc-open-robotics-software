package us.ihmc.gdx.shader;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.gdx.tools.GDXTools;

public class GDXShader
{
   private final Class<?> clazz;
   private BaseShader baseShader;
   private ShaderProgram shaderProgram;

   public GDXShader(Class<?> clazz)
   {
      this.clazz = clazz;
   }

   public void create()
   {
      String combinedString = Gdx.files.classpath(clazz.getName().replace(".", "/") + ".glsl").readString();

      String vertexMacro = "#type vertex\n";
      int vertexBegin = combinedString.indexOf(vertexMacro);
      String fragmentMacro = "#type fragment\n";
      int fragmentBegin = combinedString.indexOf(fragmentMacro);

      String fragmentShader = combinedString.substring(vertexBegin + vertexMacro.length() - 1, fragmentBegin);
      String vertexShader = combinedString.substring(fragmentBegin + fragmentMacro.length() - 1);
      shaderProgram = new ShaderProgram(vertexShader, fragmentShader);
      GDXTools.printShaderLog(shaderProgram);

      baseShader = new GDXDefaultBaseShader()
      {
         @Override
         public void render(Renderable renderable)
         {
            // TODO: Set these here?
            // - blending
            // - cull face
            // - depth test
            // - depth mask
            context.setBlending(false, GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
            context.setCullFace(GL20.GL_BACK);
            context.setDepthTest(GL20.GL_LEQUAL, 0.0f, 50.0f);
            context.setDepthMask(true);
            super.render(renderable);
         }
      };
   }

   public void init(Renderable renderable)
   {
      baseShader.init(shaderProgram, renderable);
   }

   public BaseShader getBaseShader()
   {
      return baseShader;
   }
}
