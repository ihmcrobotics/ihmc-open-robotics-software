package us.ihmc.gdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader.Inputs;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader.Setters;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import us.ihmc.log.LogTools;

/**
 * Shader used to render multiple shadows on the main scene.
 * This shader will render the scene multiple times, adding shadows for one light at a time
 */
public class GDXShadowMapShader extends BaseShader
{
   private Renderable renderable;
   private final GDXShadowManager manager;

   protected GDXShadowMapShader(final Renderable renderable, final ShaderProgram shader, final GDXShadowManager manager)
   {
      this.renderable = renderable;
      this.program = shader;
      this.manager = manager;
      register(Inputs.worldTrans, Setters.worldTrans);
      register(Inputs.projViewTrans, Setters.projViewTrans);
      register(Inputs.normalMatrix, Setters.normalMatrix);
   }

   protected static ShaderProgram buildShaderProgram()
   {
      ShaderProgram.pedantic = false;
      final String directory = "us/ihmc/gdx/shadows";
      final String prefix = "shadows";

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
      boolean firstCall = true;
      for (final GDXLight light : manager.lights)
      {
         light.apply(program);
         if (firstCall)
         {
            context.setDepthTest(GL20.GL_LEQUAL);
            context.setBlending(false, GL20.GL_ONE, GL20.GL_ONE); //Deactivate blending on first pass
            super.render(renderable, combinedAttributes); //TODO something goes wrong here w/ glDrawElements, only with multiple lights and with the EnvironmentBuilderUI - model issue?
            firstCall = false;
         }
         else
         {
            context.setDepthTest(GL20.GL_EQUAL);
            context.setBlending(true, GL20.GL_ONE, GL20.GL_ONE); //Activate additive blending
            renderable.meshPart.render(program);
         }
      }
   }
}
