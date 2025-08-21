package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;

/**
 * Renders a height map as a point cloud. The height map is stored as a 16-bit grayscale image.
 * height for each cell is represented in that buffer as value between 0 and 65536,
 * where the midway point 32,768 is the metric 0.0f height,
 * 0 is the metric -3.2768f height, and 65536 is the 3.2768f height.
 * The height is scaled up by 10,000 for storage as 16-bit value (short)
 * <p>
 * To learn about OpenGL and figure out what this code is doing,
 * Tomasz recommends <a href="https://learnopengl.com/">LearnOpenGL.com</a>
 */
public class RDXChunkRenderer implements RenderableProvider
{
   // The height map renderable
   private final Renderable renderable = new Renderable();

   // Height map data sent to the GPU (as floats)
   private final VertexAttributes vertexAttributes = new VertexAttributes(new VertexAttribute(VertexAttributes.Usage.Generic, 1, "a_height"));

   // Uniforms
   private final Vector2 chunkOrigin = new Vector2();
   private float cellSize;
   private int cellsPerAxis;
   private boolean hasBeenCreated;

   public void create(int maxCells)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      // Create the height map mesh (just a bunch of points)
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      if (renderable.meshPart.mesh != null)
      {
         renderable.meshPart.mesh.dispose();
      }
      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxCells, maxIndices, vertexAttributes);

      // Initialize the shader & assign it to the renderable
      RDXShader shader = new RDXShader(getClass());
      shader.create();
      registerUniforms(shader);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
      hasBeenCreated = true;
   }

   // Registers a bunch of uniforms needed in the shader
   @SuppressWarnings("CodeBlock2Expr")
   private void registerUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);

      RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, shader.camera.viewportWidth);
      });
      rdxShader.registerUniform(screenWidthUniform);

      RDXUniform cellsPerAxisUniform = RDXUniform.createGlobalUniform("u_cellsPerAxis", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, cellsPerAxis);
      });
      rdxShader.registerUniform(cellsPerAxisUniform);

      RDXUniform chunkOriginUniform = RDXUniform.createGlobalUniform("u_chunkOrigin", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, chunkOrigin);
      });
      rdxShader.registerUniform(chunkOriginUniform);

      RDXUniform cellSizeUniform = RDXUniform.createGlobalUniform("u_cellSize", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, cellSize);
      });
      rdxShader.registerUniform(cellSizeUniform);
   }

   public void update(Mat heightMapImage, float chunkOriginX, float chunkOriginY, int cellsPerAxis, float cellSize)
   {
      // Update uniforms
      this.cellsPerAxis = cellsPerAxis;
      this.chunkOrigin.set(chunkOriginX, chunkOriginY);
      this.cellSize = cellSize;

      // Get the vertices buffer (contains the data sent to GPU for the vertex attribute)
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true);

      int totalCells = cellsPerAxis * cellsPerAxis;
      if (renderable.meshPart.size != totalCells)
      {
         renderable.meshPart.size = totalCells;
         verticesBuffer.limit(totalCells);
      }

      /* NOTE:
       * We need to copy the short values from the height map image (Mat) to the vertices buffer (FloatBuffer) as floats.
       * In both objects, the data is in native memory. Copying native -> java -> native is slow,
       * so we use the Mat#convertTo() method which performs the short to float conversion and memory copy natively.
       */

      // Wrap the vertices buffer into a pointer, then into a Mat
      FloatPointer verticesPointer = new FloatPointer(verticesBuffer);
      Mat verticesMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, verticesPointer);

      // Convert the height map data to float, and put it into the vertex buffer
      heightMapImage.copyTo(verticesMat);

      verticesMat.close();
      verticesPointer.close();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }

   public boolean isHasBeenCreated()
   {
      return hasBeenCreated;
   }
}
