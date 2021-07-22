package us.ihmc.gdx.lighting;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;

public class GDXSceneShader extends BaseShader
{
   private Renderable renderable;
   private ShaderProgram program;

   public GDXSceneShader(Renderable renderable, ShaderProgram shaderProgram) {
      this.renderable = renderable;
      this.program = shaderProgram;
      register(DefaultShader.Inputs.worldTrans, DefaultShader.Setters.worldTrans);
      register(DefaultShader.Inputs.projViewTrans, DefaultShader.Setters.projViewTrans);
      register(DefaultShader.Inputs.normalMatrix, DefaultShader.Setters.normalMatrix);
      register(DefaultShader.Inputs.diffuseTexture, DefaultShader.Setters.diffuseTexture);
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
   public void begin(Camera camera, RenderContext context)
   {
      super.begin(camera, context);
      context.setDepthTest(GL20.GL_LEQUAL);
      context.setCullFace(GL20.GL_BACK);
   }

   @Override
   public int compareTo(Shader other)
   {
      return 0; //TODO probably fix me
   }

   @Override
   public boolean canRender(Renderable instance)
   {
      return true; //TODO maybe fix me
   }

   @Override
   public void render(final Renderable renderable)
   {
      if (!renderable.material.has(BlendingAttribute.Type))
      {
         context.setBlending(false, GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
      }
      else
      {
         context.setBlending(true, GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
      }
      super.render(renderable);
   }
}
