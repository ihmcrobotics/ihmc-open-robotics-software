package us.ihmc.gdx;

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
import org.lwjgl.opengl.GL41;

import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.function.Function;

public class GDXPointRenderer implements RenderableProvider
{
   private Renderable renderable;
   private float[] vertices;

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

   private float pointScale = 0.01f;
   private int numberOfPoints;
   private int maxPoints;

   public void create(int numberOfPoints)
   {
      this.numberOfPoints = numberOfPoints;
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(ColorAttribute.createDiffuse(Color.WHITE));

      maxPoints = numberOfPoints;
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
      floatBuffer.position(segmentToUpdate * numberOfPoints * floatsPerVertex);
      floatBuffer.limit(floatBuffer.position() + numberOfPoints * floatsPerVertex);
      int numberOfPoints = bufferConsumer.apply(floatBuffer);
      updateMeshFastest(numberOfPoints, segmentToUpdate);
   }

   public void updateMeshFastest()
   {
      updateMeshFastest(getVertexBuffer().position() / 8);
   }

   public void updateMeshFastest(int numberOfPoints)
   {
      updateMeshFastest(numberOfPoints, currentSegmentIndex);
   }

   public void submitPointCloud(PointCloud pointCloud)
   {
      FloatBuffer floatBuffer = renderable.meshPart.mesh.getVerticesBuffer();
      if (numberOfPoints < numberOfPoints) // special use case here
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
      else // numberOfPoints == numberOfPoints
      {
         floatBuffer.position(0);
         if (hasTurnedOver)
            floatBuffer.limit(maxPoints * floatsPerVertex);
         else
            floatBuffer.limit((segmentToUpdate + 1) * numberOfPoints * floatsPerVertex);
         renderable.meshPart.size = maxPoints;
      }

   }

   public FloatBuffer getVertexBuffer()
   {
      return renderable.meshPart.mesh.getVerticesBuffer();
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
      renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender)
   {
      setPointsToRender(pointsToRender, new us.ihmc.rdx.RDXPointCloudRenderer.ColorProvider()
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
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, Color color)
   {
      setPointsToRender(pointsToRender, new us.ihmc.rdx.RDXPointCloudRenderer.ColorProvider()
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
      });
   }

   public void setPointsToRender(RecyclingArrayList<Point3D32> pointsToRender, us.ihmc.rdx.RDXPointCloudRenderer.ColorProvider provider)
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
}
