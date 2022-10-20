package us.ihmc.rdx.lighting;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL41;
import us.ihmc.log.LogTools;

public class GDXDepthMapShader extends BaseShader
{
   private static ShaderProgram shaderProgram = null;
   private Renderable renderable;

   protected GDXDepthMapShader(Renderable renderable)
   {
      this.renderable = renderable;
      register(DefaultShader.Inputs.worldTrans, DefaultShader.Setters.worldTrans);
      register(DefaultShader.Inputs.projViewTrans, DefaultShader.Setters.projViewTrans);
      register(DefaultShader.Inputs.normalMatrix, DefaultShader.Setters.normalMatrix);
   }

   public static ShaderProgram getOrLoadShaderProgram()
   {
      if (shaderProgram == null)
      {
         ShaderProgram.pedantic = false;
         String directory = "us/ihmc/rdx/shadows";
         String prefix = "depthmap";
         shaderProgram = new ShaderProgram(Gdx.files.classpath(directory + "/" + prefix + "_v.glsl"), Gdx.files.classpath(directory + "/" + prefix + "_f.glsl"));
         if (!shaderProgram.isCompiled())
         {
            LogTools.fatal("Error with shader " + prefix + ": " + shaderProgram.getLog());
            System.exit(1);
         }
         else
         {
            LogTools.info("Shader " + prefix + " compiled");
         }
      }
      return shaderProgram;
   }

   @Override
   public void init()
   {
      super.init(getOrLoadShaderProgram(), renderable);
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
      super.render(renderable, combinedAttributes);
   }
}
