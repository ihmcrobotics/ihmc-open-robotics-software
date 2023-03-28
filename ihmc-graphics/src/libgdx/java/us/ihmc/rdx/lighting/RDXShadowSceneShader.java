package us.ihmc.rdx.lighting;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.RenderContext;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.lwjgl.opengl.GL41;

public class RDXShadowSceneShader extends BaseShader
{
   private Renderable renderable;
   private ShaderProgram program;

   public RDXShadowSceneShader(Renderable renderable, ShaderProgram shaderProgram)
   {
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
      context.setDepthTest(GL41.GL_LEQUAL);
      context.setCullFace(GL41.GL_BACK);
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
      Material material = renderable.material;
      context.setBlending(material.has(BlendingAttribute.Type), GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA);
      if (!material.has(TextureAttribute.Diffuse))
      {
         Pixmap map = new Pixmap(100, 100, Pixmap.Format.RGBA8888);
         map.setColor(((ColorAttribute) material.get(ColorAttribute.Diffuse)).color);
         map.drawRectangle(0, 0, 100, 100);

         material.set(TextureAttribute.createDiffuse(new Texture(map)));

         map.dispose();
      }
      super.render(renderable);
   }
}
