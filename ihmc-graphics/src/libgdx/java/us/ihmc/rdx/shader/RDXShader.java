package us.ihmc.rdx.shader;

import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.utils.ShaderProvider;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.log.LogTools;

public class RDXShader implements ShaderProvider
{
   private final Class<?> clazz;
   private BaseShader baseShader;
   private ShaderProgram shaderProgram;

   public RDXShader(Class<?> clazz)
   {
      this.clazz = clazz;
   }

   public void create()
   {
      String path = clazz.getName().replace(".", "/") + ".glsl";
      Pair<String, String> shaderStrings = LibGDXTools.loadCombinedShader(path);
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      shaderProgram = new ShaderProgram(vertexShader, fragmentShader);

      LogTools.info("OpenGL shader compilation output for {}:\n{}", path, shaderProgram.getLog());
//      LibGDXTools.printShaderLog(shaderProgram);

      baseShader = new RDXDefaultBaseShader()
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

   public void registerUniform(RDXUniform uniform)
   {
      baseShader.register(uniform.getUniform(), uniform.getSetter());
   }

   public BaseShader getBaseShader()
   {
      return baseShader;
   }

   @Override
   public Shader getShader(Renderable renderable)
   {
      return baseShader;
   }

   @Override
   public void dispose()
   {
      baseShader.dispose();
   }
}
