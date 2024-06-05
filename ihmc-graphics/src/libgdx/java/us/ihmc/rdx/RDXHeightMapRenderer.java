package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.javacpp.BytePointer;
import org.lwjgl.opengl.GL41;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;

/**
 * Renders a height map as a point cloud. The height map is stored as a 16-bit grayscale image.
 * height for each cell is represented in that buffer as value between 0 and 65536,
 * where the midway point 32,768 is the metric 0.0f height,
 * 0 is the metric -3.2768f height, and 65536 is the 3.2768f height.
 * The height is scaled up by 10,000 for storage as 16-bit value (short)
 */

/**
 * This has been updated to use {@link us.ihmc.rdx.ui.graphics.RDXHeightMapGraphicNew}, please use that going forward, this implementation has bugs with
 * interacting with collisions
 * from the mouse
 */
@Deprecated
public class RDXHeightMapRenderer implements RenderableProvider
{
   private Renderable renderable;

   public static final int FLOATS_PER_CELL = 8;
   private final VertexAttributes vertexAttributes = new VertexAttributes(new VertexAttribute(VertexAttributes.Usage.Position,
                                                                                              3,
                                                                                              ShaderProgram.POSITION_ATTRIBUTE),
                                                                          new VertexAttribute(VertexAttributes.Usage.ColorUnpacked,
                                                                                              4,
                                                                                              ShaderProgram.COLOR_ATTRIBUTE),
                                                                          new VertexAttribute(VertexAttributes.Usage.Generic,
                                                                                              1,
                                                                                              GL41.GL_FLOAT,
                                                                                              false,
                                                                                              "a_size"));
   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth",
                                                                                (shader, inputID, renderable, combinedAttributes) -> shader.set(inputID,
                                                                                                                                                shader.camera.viewportWidth));
   private final RDXUniform multiColorUniform = RDXUniform.createGlobalUniform("u_multiColor", (shader, inputID, renderable, combinedAttributes) ->
   {
      int multiColor = 0;
      shader.set(inputID, multiColor);
   });

   private float[] intermediateVertexBuffer;

   private int totalCells;

   public void create(int numberOfCells)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      totalCells = numberOfCells;
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, totalCells, maxIndices, vertexAttributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create();
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.registerUniform(screenWidthUniform);
      shader.registerUniform(multiColorUniform);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();

      LogTools.info("Vertex Buffer Size: {}", totalCells * FLOATS_PER_CELL);
      intermediateVertexBuffer = new float[totalCells * FLOATS_PER_CELL];
   }

   public void update(RigidBodyTransform zUpFrameToWorld,
                      BytePointer heightMapPointer,
                      float heightOffset,
                      float gridCenterX,
                      float gridCenterY,
                      int centerIndex,
                      float cellSizeXYInMeters,
                      float heightScalingFactor)
   {
      zUpFrameToWorld.getTranslation().setZ(0);

      int cellsPerAxis = 2 * centerIndex + 1;

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            double xPosition = HeightMapTools.indexToCoordinate(xIndex, gridCenterX, cellSizeXYInMeters, centerIndex);
            double yPosition = HeightMapTools.indexToCoordinate(yIndex, gridCenterY, cellSizeXYInMeters, centerIndex);

            /* look at the header docs for decoding the height map values as below */
            int heightIndex = xIndex * cellsPerAxis + yIndex;
            int vertexIndex = heightIndex * FLOATS_PER_CELL;
            int height = heightMapPointer.getShort(heightIndex * 2L) & 0xFFFF;
            float zPosition = ((float) height / heightScalingFactor);
            zPosition -= heightOffset;

            intermediateVertexBuffer[vertexIndex] = (float) xPosition;
            intermediateVertexBuffer[vertexIndex + 1] = (float) yPosition;
            intermediateVertexBuffer[vertexIndex + 2] = zPosition;

            // Color (0.0 to 1.0)
            double[] redGreenBlue = HeightMapTools.getRedGreenBlue(zPosition);
            Color color = new Color((float) redGreenBlue[0], (float) redGreenBlue[1], (float) redGreenBlue[2], 1.0f);
            intermediateVertexBuffer[vertexIndex + 3] = color.r;
            intermediateVertexBuffer[vertexIndex + 4] = color.g;
            intermediateVertexBuffer[vertexIndex + 5] = color.b;
            intermediateVertexBuffer[vertexIndex + 6] = color.a;

            // Size
            intermediateVertexBuffer[vertexIndex + 7] = 0.02f;
         }
      }

      renderable.meshPart.size = totalCells;
      renderable.meshPart.mesh.setVertices(intermediateVertexBuffer, 0, totalCells * FLOATS_PER_CELL);
   }

   public FloatBuffer getVertexBuffer()
   {
      return renderable.meshPart.mesh.getVerticesBuffer();
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
}
