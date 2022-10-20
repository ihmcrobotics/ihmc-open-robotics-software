package us.ihmc.rdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader.Inputs;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader.Setters;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL41;
import us.ihmc.log.LogTools;

import java.util.function.Consumer;

/**
 * Shader used to render multiple shadows on the main scene.
 * This shader will render the scene multiple times, adding shadows for one light at a time
 */
public class GDXShadowMapShader extends BaseShader
{
   private Renderable renderable;
   private final GDXShadowManager shadowManager;

   protected GDXShadowMapShader(Renderable renderable, GDXShadowManager shadowManager)
   {
      this.renderable = renderable;
      this.shadowManager = shadowManager;

      register(Inputs.worldTrans, Setters.worldTrans);
      register(Inputs.projViewTrans, Setters.projViewTrans);
      register(Inputs.normalMatrix, Setters.normalMatrix);

      ShaderProgram.pedantic = false;
      final String directory = "us/ihmc/rdx/shadows";
      final String prefix = "shadows";

      program = new ShaderProgram(Gdx.files.classpath(directory + "/" + prefix + "_v.glsl"),
                                                            Gdx.files.classpath(directory + "/" + prefix + "_f.glsl"));
      if (!program.isCompiled())
      {
         LogTools.fatal("Error with shader " + prefix + ": " + program.getLog());
         System.exit(1);
      }
      else
      {
         LogTools.info("Shader " + prefix + " compiled");
      }
   }

   @Override
   public void init()
   {
      final ShaderProgram program = this.program;
      this.program = null;
      super.init(program, renderable);
      renderable = null;
   }

   @Override
   public boolean canRender(final Renderable instance)
   {
      return true;
   }

   @Override
   public int compareTo(final Shader other)
   {
      if (other == null) return -1;
      if (other == this) return 0;
      return 0;
   }

   @Override
   public void begin(Camera camera, RenderContext context)
   {
      super.begin(camera, context);
      context.setDepthTest(GL41.GL_LEQUAL);
      context.setCullFace(GL41.GL_BACK);
   }

   @Override
   public void render(final Renderable renderable, final Attributes combinedAttributes)
   {
      if (!combinedAttributes.has(BlendingAttribute.Type))
         context.setBlending(false, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA);

      context.setDepthTest(GL41.GL_LEQUAL);
      context.setBlending(false, GL41.GL_ONE, GL41.GL_ONE); // Deactivate blending on first pass
      super.render(renderable, combinedAttributes);
      for (GDXPointLight light : shadowManager.getPointLights())
      {
         context.setDepthTest(GL41.GL_LEQUAL);
         context.setBlending(true, GL41.GL_ONE, GL41.GL_ONE); // Activate additive blending
         renderLight(renderable, combinedAttributes, light::apply);
      }
   }

   private void renderLight(Renderable renderable, Attributes combinedAttributes, Consumer<ShaderProgram> light)
   {
      light.accept(program);
      context.setBlending(true, GL41.GL_ONE, GL41.GL_ONE); // Activate additive blending
      renderable.meshPart.render(program);
   }
}
