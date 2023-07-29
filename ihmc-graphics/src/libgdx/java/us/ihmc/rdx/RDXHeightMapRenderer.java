package us.ihmc.rdx;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.bytedeco.javacpp.BytePointer;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;

public class RDXHeightMapRenderer implements RenderableProvider
{
   private Renderable renderable;

   public static final int FLOATS_PER_CELL = 8;
   public static final int BYTES_PER_VERTEX = FLOATS_PER_CELL * Float.BYTES;
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
   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, shader.camera.viewportWidth);
   });
   private int multiColor = 0;
   private final RDXUniform multiColorUniform = RDXUniform.createGlobalUniform("u_multiColor", (shader, inputID, renderable, combinedAttributes) ->
   {
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

      intermediateVertexBuffer = new float[totalCells * FLOATS_PER_CELL];
   }

   public void update(BytePointer heightMapPointer, Point2DReadOnly center, int centerIndex, float cellSizeXYInMeters)
   {
      int cellsPerAxis = 2 * centerIndex + 1;
      LogTools.info("Rendering Height Map: {} {} {}", cellsPerAxis, cellsPerAxis, cellSizeXYInMeters);

      float maxHeight = 2.0f;
      float minHeight = 0.0f;

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         double xPosition = indexToCoordinate(xIndex, center.getX(), cellSizeXYInMeters, centerIndex);

         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int heightIndex = xIndex * cellsPerAxis + yIndex;
            int vertexIndex = heightIndex * FLOATS_PER_CELL;
            float cellHeight = (float) (heightMapPointer.getShort(heightIndex * 2L) / 10000.0f);
            cellHeight = (float) MathTools.clamp(cellHeight, minHeight, maxHeight);
            if (cellHeight > maxHeight - 0.01f)
               cellHeight = 0.0f;

            double yPosition = indexToCoordinate(yIndex, center.getY(), cellSizeXYInMeters, centerIndex);

            // Position
            intermediateVertexBuffer[vertexIndex] = (float) xPosition;
            intermediateVertexBuffer[vertexIndex + 1] = (float) yPosition;
            intermediateVertexBuffer[vertexIndex + 2] = cellHeight;

            // Color (0.0 to 1.0)
            float heightRatio = (cellHeight / maxHeight);
            intermediateVertexBuffer[vertexIndex + 3] = Math.abs(1.0f - heightRatio);
            intermediateVertexBuffer[vertexIndex + 4] = Math.abs(1.0f - heightRatio * 0.123f);
            intermediateVertexBuffer[vertexIndex + 5] = Math.abs(1.0f - heightRatio);
            intermediateVertexBuffer[vertexIndex + 6] = Math.abs(0.3f + 0.5f * heightRatio);

            // Size
            intermediateVertexBuffer[vertexIndex + 7] = 0.03f;
         }
      }

      renderable.meshPart.size = totalCells;
      renderable.meshPart.mesh.setVertices(intermediateVertexBuffer, 0, totalCells * FLOATS_PER_CELL);
   }

   public static double indexToCoordinate(int index, double gridCenter, double resolution, int centerIndex)
   {
      return (index - centerIndex) * resolution + gridCenter;
   }

   public FloatBuffer getVertexBuffer()
   {
      return renderable.meshPart.mesh.getVerticesBuffer();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderable != null)
         renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }
}
