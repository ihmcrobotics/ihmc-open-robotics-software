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
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.stream.IntStream;

/**
 * Renders a height map as a point cloud. The height map is stored as a 16-bit grayscale image.
 * height for each cell is represented in that buffer as value between 0 and 65536,
 * where the midway point 32,768 is the metric 0.0f height,
 * 0 is the metric -3.2768f height, and 65536 is the 3.2768f height.
 * The height is scaled up by 10,000 for storage as 16-bit value (short)
 */
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

      Point3D spritePoint = new Point3D();
      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            spritePoint.setToZero();

            double xPosition = indexToCoordinate(xIndex, gridCenterX, cellSizeXYInMeters, centerIndex); // + 1.5f
            double yPosition = indexToCoordinate(yIndex, gridCenterY, cellSizeXYInMeters, centerIndex);

            /* look at the header docs for decoding the height map values as below */
            int heightIndex = xIndex * cellsPerAxis + yIndex;
            int vertexIndex = heightIndex * FLOATS_PER_CELL;
            int height = heightMapPointer.getShort(heightIndex * 2L) & 0xFFFF;
            float zPosition = ((float) height / heightScalingFactor);
            zPosition -= heightOffset;

            spritePoint.set(xPosition, yPosition, zPosition);
            intermediateVertexBuffer[vertexIndex] = (float) spritePoint.getX();
            intermediateVertexBuffer[vertexIndex + 1] = (float) spritePoint.getY();
            intermediateVertexBuffer[vertexIndex + 2] = (float) spritePoint.getZ();

            Color color = computeColorFromHeight(zPosition);

            /* For the brighter ones */
            //float heightRatio = (zPosition / maxHeight);
            //color.set(Math.abs(1.0f - heightRatio), Math.max(100.0f * heightRatio, 1.0f), Math.abs(1.0f - heightRatio), Math.abs(0.3f + 10.0f * heightRatio));

            // Color (0.0 to 1.0)
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

   public static Color computeColorFromHeight(double height)
   {
      // Using interpolation between key color points
      double r = 0, g = 0, b = 0;
      double redR = 1.0, redG = 0.0, redB = 0.0;
      double magentaR = 1.0, magentaG = 0.0, magentaB = 1.0;
      double orangeR = 1.0, orangeG = 200.0 / 255.0, orangeB = 0.0;
      double yellowR = 1.0, yellowG = 1.0, yellowB = 0.0;
      double blueR = 0.0, blueG = 0.0, blueB = 1.0;
      double greenR = 0.0, greenG = 1.0, greenB = 0.0;
      double gradientSize = 0.2;
      double gradientLength = 1.0;
      double alpha = height % gradientLength;
      if (alpha < 0)
         alpha = 1 + alpha;
      while (alpha > 5 * gradientSize)
         alpha -= 5 * gradientSize;

      if (alpha <= gradientSize * 1)
      {
         r = InterpolationTools.linearInterpolate(magentaR, blueR, (alpha) / gradientSize);
         g = InterpolationTools.linearInterpolate(magentaG, blueG, (alpha) / gradientSize);
         b = InterpolationTools.linearInterpolate(magentaB, blueB, (alpha) / gradientSize);
      }
      else if (alpha <= gradientSize * 2)
      {
         r = InterpolationTools.linearInterpolate(blueR, greenR, (alpha - gradientSize * 1) / gradientSize);
         g = InterpolationTools.linearInterpolate(blueG, greenG, (alpha - gradientSize * 1) / gradientSize);
         b = InterpolationTools.linearInterpolate(blueB, greenB, (alpha - gradientSize * 1) / gradientSize);
      }
      else if (alpha <= gradientSize * 3)
      {
         r = InterpolationTools.linearInterpolate(greenR, yellowR, (alpha - gradientSize * 2) / gradientSize);
         g = InterpolationTools.linearInterpolate(greenG, yellowG, (alpha - gradientSize * 2) / gradientSize);
         b = InterpolationTools.linearInterpolate(greenB, yellowB, (alpha - gradientSize * 2) / gradientSize);
      }
      else if (alpha <= gradientSize * 4)
      {
         r = InterpolationTools.linearInterpolate(yellowR, orangeR, (alpha - gradientSize * 3) / gradientSize);
         g = InterpolationTools.linearInterpolate(yellowG, orangeG, (alpha - gradientSize * 3) / gradientSize);
         b = InterpolationTools.linearInterpolate(yellowB, orangeB, (alpha - gradientSize * 3) / gradientSize);
      }
      else if (alpha <= gradientSize * 5)
      {
         r = InterpolationTools.linearInterpolate(orangeR, redR, (alpha - gradientSize * 4) / gradientSize);
         g = InterpolationTools.linearInterpolate(orangeG, redG, (alpha - gradientSize * 4) / gradientSize);
         b = InterpolationTools.linearInterpolate(orangeB, redB, (alpha - gradientSize * 4) / gradientSize);
      }
      else
      {
         throw new RuntimeException("no valid color");
      }

      if (r == 0.0 && g == 0.0 && b == 0.0)
         throw new RuntimeException("Shouldn't return black.)");
      return new Color((float) r, (float) g, (float) b, 1.0f);
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
      renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }
}
