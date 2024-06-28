package us.ihmc.rdx.shader;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXOutlineShader
{
   private final ShaderProgram shaderProgram;
   private final SpriteBatch spriteBatch;

   private float outlineWidth = 1.0f;
   private float outlineDepthMin = 0.35f;
   private float outlineDepthMax = 0.7f;
   private Color innerColor = Color.WHITE;
   private Color outerColor = Color.WHITE;

   public RDXOutlineShader()
   {
      String path = RDXOutlineShader.class.getName().replace(".", "/") + ".glsl";
      Pair<String, String> shaderStrings = LibGDXTools.loadCombinedShader(path);
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      shaderProgram = new ShaderProgram(vertexShader, fragmentShader);

      if (!shaderProgram.isCompiled())
         LogTools.error("Shader failed to compile: {}", path);
      LogTools.info("OpenGL shader compilation output for {}:\n{}", path, shaderProgram.getLog());

      spriteBatch = new SpriteBatch();
   }

   public void render(Texture depthTexture)
   {
      shaderProgram.bind();
      float size = 1.0f - outlineWidth;

      float depthMin = (float) Math.pow(outlineDepthMin, 10.0);
      float depthMax = (float) Math.pow(outlineDepthMax, 10.0);

      // TODO use an integer instead and divide w and h
      shaderProgram.setUniformf("u_size", Gdx.graphics.getWidth() * size, Gdx.graphics.getHeight() * size);
      shaderProgram.setUniformf("u_depth_min", depthMin);
      shaderProgram.setUniformf("u_depth_max", depthMax);
      shaderProgram.setUniformf("u_inner_color", innerColor);
      shaderProgram.setUniformf("u_outer_color", outerColor);

      spriteBatch.enableBlending();
      spriteBatch.getProjectionMatrix().setToOrtho2D(0, 0, 1, 1);
      spriteBatch.setShader(shaderProgram);
      spriteBatch.begin();
      spriteBatch.draw(depthTexture, 0, 0, 1, 1, 0f, 0f, 1f, 1f);
      spriteBatch.end();
      spriteBatch.setShader(null);
   }
}
