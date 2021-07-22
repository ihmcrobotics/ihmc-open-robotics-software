package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.log.LogTools;

public class GDXDepthMapShader extends BaseShader
{
   private Renderable renderable;

   protected GDXDepthMapShader(final Renderable renderable, final ShaderProgram shaderProgram)
   {
      this.renderable = renderable;
      this.program = shaderProgram;
      register(DefaultShader.Inputs.worldTrans, DefaultShader.Setters.worldTrans);
      register(DefaultShader.Inputs.projViewTrans, DefaultShader.Setters.projViewTrans);
      register(DefaultShader.Inputs.normalMatrix, DefaultShader.Setters.normalMatrix);
   }

   protected static ShaderProgram buildShaderProgram()
   {
      ShaderProgram.pedantic = false;
      final String directory = "us/ihmc/gdx/shadows";
      final String prefix = "depthmap";

      final ShaderProgram shaderProgram = new ShaderProgram(Gdx.files.classpath(directory + "/" + prefix + "_v.glsl"),
                                                            Gdx.files.classpath(directory + "/" + prefix + "_f.glsl"));
      if (!shaderProgram.isCompiled())
      {
         LogTools.fatal("Error with shader " + prefix + ": " + shaderProgram.getLog());
         System.exit(1);
      }
      else
      {
         LogTools.info("Shader " + prefix + " compiled");
      }
      return shaderProgram;
   }

   @Override
   public void end()
   {
      super.end();
   }

   @Override
   public void begin(final Camera camera, final RenderContext context)
   {
      super.begin(camera, context);
      context.setDepthTest(GL20.GL_LEQUAL);
      context.setCullFace(GL20.GL_BACK);
   }

   @Override
   public void render(final Renderable renderable)
   {
      context.setBlending(renderable.material.has(BlendingAttribute.Type), GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
      super.render(renderable);
   }

   @Override
   public void init()
   {
      final ShaderProgram program = this.program;
      this.program = null;
      init(program, renderable);
      renderable = null;
   }

   @Override
   public int compareTo(final Shader other)
   {
      return 0;
   }

   @Override
   public boolean canRender(final Renderable instance)
   {
      return true;
   }

   @Override
   public void render(final Renderable renderable, final Attributes combinedAttributes)
   {
      super.render(renderable, combinedAttributes);
   }
}
