package us.ihmc.rdx;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class RDXPointCloudRenderer implements RenderableProvider
{
   private Renderable renderable;
   private float[] vertices;

   public static final int FLOATS_PER_VERTEX = 8;
   public static final int BYTES_PER_VERTEX = FLOATS_PER_VERTEX * Float.BYTES;
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
   private final int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;
   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, shader.camera.viewportWidth);
   });
   private int multiColor = 0;
   private final RDXUniform multiColorUniform = RDXUniform.createGlobalUniform("u_multiColor", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, multiColor);
   });

   private final Point3D32 nanPoint = new Point3D32();
   {
      nanPoint.setToNaN();
   }
   private List<Point3D32> pointsToRender;
   private ColorProvider colorProvider;
   private float pointScale = 0.01f;
   private int pointsPerSegment;
   private int numberOfSegments;
   private boolean hasTurnedOver = false;
   private int currentSegmentIndex = 0;
   private int maxPoints;

   public interface ColorProvider
   {
      float getNextR();

      float getNextG();

      float getNextB();

      float getNextA();
   }

   public void create(int size)
   {
      create(size, 1);
   }

   public void create(int pointsPerSegment, int numberOfSegments)
   {
      currentSegmentIndex = 0;
      hasTurnedOver = false;
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      maxPoints = pointsPerSegment * numberOfSegments;
      vertices = new float[maxPoints * floatsPerVertex];
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxPoints, maxIndices, vertexAttributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create();
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.registerUniform(screenWidthUniform);
      shader.registerUniform(multiColorUniform);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   public void updateMesh()
   {
      updateMesh(1.0f);
   }

   public void updateMesh(float alpha)
   {
      if (pointsToRender != null)
      {
         if (pointsToRender.isEmpty()) // make sure there's always one point
         {
            if (pointsToRender instanceof RecyclingArrayList<Point3D32> recycling)
               recycling.add().setToNaN();
            else
               pointsToRender.add(nanPoint);
         }

         for (int i = 0; i < pointsToRender.size(); i++)
         {
            int offset = i * floatsPerVertex;

            Point3D32 point = pointsToRender.get(i);
            vertices[offset] = point.getX32();
            vertices[offset + 1] = point.getY32();
            vertices[offset + 2] = point.getZ32();

            // color [0.0f - 1.0f]
            vertices[offset + 3] = colorProvider.getNextR();
            vertices[offset + 4] = colorProvider.getNextG();
            vertices[offset + 5] = colorProvider.getNextB();
            vertices[offset + 6] = alpha; // alpha

            vertices[offset + 7] = pointScale; // size
         }

         renderable.meshPart.size = pointsToRender.size();
         renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * floatsPerVertex);
//         renderable.meshPart.update();
      }
   }

   public void updateMesh(RecyclingArrayList<? extends Tuple3DBasics> pointsToRender, Color color)
   {
      if (pointsToRender != null)
      {
         if (pointsToRender.isEmpty()) // make sure there's always one point
            pointsToRender.add().setToNaN();

         for (int i = 0; i < pointsToRender.size(); i++)
         {
            int offset = i * floatsPerVertex;

            Tuple3DReadOnly point = pointsToRender.get(i);
            vertices[offset] = point.getX32();
            vertices[offset + 1] = point.getY32();
            vertices[offset + 2] = point.getZ32();

            // color [0.0f - 1.0f]
            vertices[offset + 3] = color.r;
            vertices[offset + 4] = color.g;
            vertices[offset + 5] = color.b;
            vertices[offset + 6] = color.a;

            vertices[offset + 7] = pointScale; // size
         }

         renderable.meshPart.size = pointsToRender.size();
         renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * floatsPerVertex);
         //         renderable.meshPart.update();
      }
   }

   public void updateMesh(RecyclingArrayList<Point3D32> pointsToRender, ArrayList<Integer> colors)
   {
      for (int i = 0; i < pointsToRender.size(); i++)
      {
         int offset = i * floatsPerVertex;

         Point3D32 point = pointsToRender.get(i);
         vertices[offset] = point.getX32();
         vertices[offset + 1] = point.getY32();
         vertices[offset + 2] = point.getZ32();

         // color [0.0f - 1.0f]
         if (colors.size() > i)
         {
            vertices[offset + 3] = ((colors.get(i) & 0xff000000) >>> 24) / 255f;
            vertices[offset + 4] = ((colors.get(i) & 0x00ff0000) >>> 16) / 255f;
            vertices[offset + 5] = ((colors.get(i) & 0x0000ff00) >>> 8) / 255f;
            vertices[offset + 6] = ((colors.get(i) & 0x000000ff)) / 255f;
         }

         vertices[offset + 7] = pointScale; // size
      }

      renderable.meshPart.size = pointsToRender.size();
      renderable.meshPart.mesh.setVertices(vertices, 0, pointsToRender.size() * floatsPerVertex);
      if (!pointsToRender.isEmpty())
      {
//         renderable.meshPart.update();
      }
   }

   public void updateMeshFastest(Function<FloatBuffer, Integer> bufferConsumer)
   {
      updateMeshFastest(bufferConsumer, currentSegmentIndex);
   }

   public void updateMeshFastest(Function<FloatBuffer, Integer> bufferConsumer, int segmentToUpdate)
   {
      if (!hasTurnedOver && segmentToUpdate > currentSegmentIndex)
         return;

      FloatBuffer floatBuffer = renderable.meshPart.mesh.getVerticesBuffer();
      floatBuffer.position(segmentToUpdate * pointsPerSegment * floatsPerVertex);
      floatBuffer.limit(floatBuffer.position() + pointsPerSegment * floatsPerVertex);
      int numberOfPoints = bufferConsumer.apply(floatBuffer);
      updateMeshFastest(numberOfPoints, segmentToUpdate);
   }

   public void updateMeshFastest()
   {
      updateMeshFastest(getVertexBuffer().position() / 8);
   }

   public void updateMeshFastest(int numberOfPoints)
   {
      FloatBuffer floatBuffer = renderable.meshPart.mesh.getVerticesBuffer();
      floatBuffer.position(0);
      floatBuffer.limit(numberOfPoints * FLOATS_PER_VERTEX);
      renderable.meshPart.size = numberOfPoints;
   }

   public void updateMeshFastest(int numberOfPoints, int segmentToUpdate)
   {
      FloatBuffer floatBuffer = renderable.meshPart.mesh.getVerticesBuffer();
      if (numberOfPoints < pointsPerSegment) // special use case here
      {
         if (numberOfPoints == 0) // prevents errors when no point are there
         {
            numberOfPoints = 1;
            floatBuffer.put(Float.NaN);
            floatBuffer.put(Float.NaN);
            floatBuffer.put(Float.NaN);
            floatBuffer.put(1.0f);
            floatBuffer.put(1.0f);
            floatBuffer.put(1.0f);
            floatBuffer.put(1.0f);
            floatBuffer.put(1.0f);
         }

         floatBuffer.position(0);
         floatBuffer.limit(numberOfPoints * floatsPerVertex);
         renderable.meshPart.size = numberOfPoints;
      }
      else // numberOfPoints == pointsPerSegment
      {
         floatBuffer.position(0);
         if (hasTurnedOver)
            floatBuffer.limit(maxPoints * floatsPerVertex);
         else
            floatBuffer.limit((segmentToUpdate + 1) * pointsPerSegment * floatsPerVertex);
         renderable.meshPart.size = maxPoints;
      }

      currentSegmentIndex = segmentToUpdate + 1;
      if (currentSegmentIndex == numberOfSegments)
      {
         hasTurnedOver = true;
         currentSegmentIndex = 0;
      }
   }

   /**
    * This method is to be called before packing the vertex buffer,
    * like when running an OpenCL kernel to pack it.
    */
   public void updateMeshFastestBeforeKernel()
   {
      if (renderable.meshPart.mesh != null)
      {
         FloatBuffer floatBuffer = renderable.meshPart.mesh.getVerticesBuffer();
         floatBuffer.position(0);
         int numberOfPointsNow;
         if (!hasTurnedOver)
            numberOfPointsNow = (currentSegmentIndex + 1) * pointsPerSegment;
         else // After one cycle, we are just updating now; if you don't you'll get a progressive flicker
            numberOfPointsNow = maxPoints;
         floatBuffer.limit(numberOfPointsNow * floatsPerVertex);
         renderable.meshPart.size = numberOfPointsNow;
      }
   }

   /**
    * This method is meant to be called after packing the vertex buffer.
    * It is used to update the renderer segment index in the case
    * that feature is being used.
    */
   public void updateMeshFastestAfterKernel()
   {
      ++currentSegmentIndex;
      if (currentSegmentIndex >= numberOfSegments)
      {
         hasTurnedOver = true;
         currentSegmentIndex = 0;
      }
   }

   public void setVertex(int vertexIndex, Tuple3DReadOnly point)
   {
      setVertex(vertexIndex, point.getX32(), point.getY32(), point.getZ32(), 1.0f, 1.0f, 1.0f, 1.0f, 0.01f);
   }

   public void setVertex(int vertexIndex, float x, float y, float z, float r, float g, float b, float a, float pointSize)
   {
      int offset = vertexIndex * 8;
      getVertexBuffer().put(offset++, x);
      getVertexBuffer().put(offset++, y);
      getVertexBuffer().put(offset++, z);
      getVertexBuffer().put(offset++, r);
      getVertexBuffer().put(offset++, g);
      getVertexBuffer().put(offset++, b);
      getVertexBuffer().put(offset++, a);
      getVertexBuffer().put(offset, pointSize);
   }

   public void putVertex(Tuple3DReadOnly point)
   {
      putVertex(point.getX32(), point.getY32(), point.getZ32(), 1.0f, 1.0f, 1.0f, 1.0f, 0.01f);
   }

   public void putVertex(float x, float y, float z, float r, float g, float b, float a, float pointSize)
   {
      getVertexBuffer().put(x);
      getVertexBuffer().put(y);
      getVertexBuffer().put(z);
      getVertexBuffer().put(r);
      getVertexBuffer().put(g);
      getVertexBuffer().put(b);
      getVertexBuffer().put(a);
      getVertexBuffer().put(pointSize);
   }

   public void prepareVertexBufferForAddingPoints()
   {
      FloatBuffer vertexBuffer = getVertexBuffer();
      vertexBuffer.limit(vertexBuffer.capacity());
      vertexBuffer.rewind();
   }

   public FloatBuffer getAndPrepareVertexBuffer()
   {
      prepareVertexBufferForAddingPoints();
      return getVertexBuffer();
   }

   public FloatBuffer getVertexBuffer()
   {
      return renderable.meshPart.mesh.getVerticesBuffer();
   }

   @Deprecated
   public float[] getVerticesArray()
   {
      return vertices;
   }

   public void updateMeshFast(int numberOfPoints)
   {
      renderable.meshPart.size = numberOfPoints;
      renderable.meshPart.mesh.setVertices(vertices, 0, numberOfPoints * floatsPerVertex);
//      renderable.meshPart.update();
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

   public void setPointsToRender(List<Point3D32> pointsToRender)
   {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         @Override
         public float getNextR()
         {
            return Color.WHITE.r;
         }

         @Override
         public float getNextG()
         {
            return Color.WHITE.g;
         }

         @Override
         public float getNextB()
         {
            return Color.WHITE.b;
         }

         @Override
         public float getNextA()
         {
            return Color.WHITE.a;
         }
      });
   }

   public void setPointsToRender(List<Point3D32> pointsToRender, Color color)
   {
      setPointsToRender(pointsToRender, new ColorProvider()
      {
         @Override
         public float getNextR()
         {
            return color.r;
         }

         @Override
         public float getNextG()
         {
            return color.g;
         }

         @Override
         public float getNextB()
         {
            return color.b;
         }

         @Override
         public float getNextA()
         {
            return color.a;
         }
      });
   }

   public void setPointsToRender(List<Point3D32> pointsToRender, ColorProvider provider)
   {
      this.pointsToRender = pointsToRender;
      this.colorProvider = provider;
   }

   public void setPointScale(float size)
   {
      this.pointScale = size;
   }

   public int getFloatsPerVertex()
   {
      return floatsPerVertex;
   }

   public int getMaxPoints()
   {
      return maxPoints;
   }

   public int getCurrentSegmentIndex()
   {
      return currentSegmentIndex;
   }
}
