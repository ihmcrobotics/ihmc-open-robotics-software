package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * Renders a voxel map as a point cloud.
 * Each occupied voxel is sent to the GPU as a single point.
 */
public class RDXVoxelMapRenderer implements RenderableProvider
{
   private static final float[] CUBE_VERTS = {
         // +X face (normal +X), view from +X looking at origin
         // corners on this face:
         // p0 = (0.5,  0.5,  0.5)   // top-front
         // p1 = (0.5, -0.5,  0.5)   // bottom-front
         // p2 = (0.5, -0.5, -0.5)   // bottom-back
         // p3 = (0.5,  0.5, -0.5)   // top-back

         // t0: p0, p1, p2
         0.5f,  0.5f,  0.5f,  // p0
         0.5f, -0.5f,  0.5f,  // p1
         0.5f, -0.5f, -0.5f,  // p2

         // t1: p3, p0, p2
         0.5f,  0.5f, -0.5f,  // p3
         0.5f,  0.5f,  0.5f,  // p0
         0.5f, -0.5f, -0.5f,  // p2


         // -X face (normal -X), view from -X looking at origin
         // p0 = (-0.5, -0.5, -0.5)  // bottom-back
         // p1 = (-0.5,  0.5, -0.5)  // top-back
         // p2 = (-0.5,  0.5,  0.5)  // top-front
         // p3 = (-0.5, -0.5,  0.5)  // bottom-front

         // t0: p0, p1, p2
         -0.5f,  0.5f, -0.5f, // p1
         -0.5f, -0.5f, -0.5f, // p0
         -0.5f,  0.5f,  0.5f, // p2

         // t1: p0, p2, p3
         -0.5f,  0.5f,  0.5f, // p2
         -0.5f, -0.5f, -0.5f, // p0
         -0.5f, -0.5f,  0.5f, // p3


         // +Y face (normal +Y), view from +Y looking at origin
         // p0 = (-0.5,  0.5, -0.5)  // left-back
         // p1 = (-0.5,  0.5,  0.5)  // left-front
         // p2 = ( 0.5,  0.5,  0.5)  // right-front
         // p3 = ( 0.5,  0.5, -0.5)  // right-back

         // t0: p0, p1, p2
         -0.5f,  0.5f, -0.5f, // p0
         -0.5f,  0.5f,  0.5f, // p1
         0.5f,  0.5f,  0.5f, // p2

         // t1: p0, p2, p3
         -0.5f,  0.5f, -0.5f, // p0
         0.5f,  0.5f,  0.5f, // p2
         0.5f,  0.5f, -0.5f, // p3


         // -Y face (normal -Y), view from -Y looking at origin
         // p0 = (-0.5, -0.5, -0.5)  // left-back
         // p1 = ( 0.5, -0.5, -0.5)  // right-back
         // p2 = ( 0.5, -0.5,  0.5)  // right-front
         // p3 = (-0.5, -0.5,  0.5)  // left-front

         // t0: p0, p1, p2
         -0.5f, -0.5f, -0.5f, // p0
         0.5f, -0.5f, -0.5f, // p1
         0.5f, -0.5f,  0.5f, // p2

         // t1: p0, p2, p3
         -0.5f, -0.5f, -0.5f, // p0
         0.5f, -0.5f,  0.5f, // p2
         -0.5f, -0.5f,  0.5f, // p3


         // +Z face (normal +Z), view from +Z looking at origin
         // p0 = (-0.5, -0.5,  0.5)  // bottom-left
         // p1 = ( 0.5, -0.5,  0.5)  // bottom-right
         // p2 = ( 0.5,  0.5,  0.5)  // top-right
         // p3 = (-0.5,  0.5,  0.5)  // top-left

         // t0: p0, p1, p2
         -0.5f, -0.5f,  0.5f, // p0
         0.5f, -0.5f,  0.5f, // p1
         0.5f,  0.5f,  0.5f, // p2

         // t1: p0, p2, p3
         -0.5f, -0.5f,  0.5f, // p0
         0.5f,  0.5f,  0.5f, // p2
         -0.5f,  0.5f,  0.5f, // p3


         // -Z face (normal -Z), view from -Z looking at origin
         // p0 = (-0.5, -0.5, -0.5)  // bottom-left
         // p1 = ( 0.5, -0.5, -0.5)  // bottom-right
         // p2 = ( 0.5,  0.5, -0.5)  // top-right
         // p3 = (-0.5,  0.5, -0.5)  // top-left

         // t0: p0, p3, p2  (this ordering is what you want if you need CCW from -Z)
         -0.5f, -0.5f, -0.5f, // p0
         -0.5f,  0.5f, -0.5f, // p3
         0.5f,  0.5f, -0.5f, // p2

         // t1: p0, p2, p1
         -0.5f, -0.5f, -0.5f, // p0
         0.5f,  0.5f, -0.5f, // p2
         0.5f, -0.5f, -0.5f, // p1
   };
   private static final int CUBE_VERTEX_COUNT = 36;

   private final Renderable renderable = new Renderable();

   // Vertex attribute: position only (x, y, z)
   private final VertexAttributes vertexAttributes = new VertexAttributes(new VertexAttribute(Usage.Position, 3, "a_cubePos"),
                                                                          new VertexAttribute(Usage.Generic, 3, "a_voxelCenter"));

   // Uniforms
   private float voxelSize = 0.05f; // default, set with update()
   private boolean hasBeenCreated = false;

   public void create(int maxVoxels)
   {
      renderable.meshPart.primitiveType = GL41.GL_TRIANGLES;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();

      boolean isStatic = false;
      int maxVertices = maxVoxels * CUBE_VERTEX_COUNT;
      int maxIndices = 0; // unindexed, 36 verts per cube

      renderable.meshPart.mesh = new Mesh(isStatic, maxVertices, maxIndices, vertexAttributes);

      // Shader
      RDXShader shader = new RDXShader(getClass());
      shader.create();
      registerUniforms(shader);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
      hasBeenCreated = true;
   }

   private void registerUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);

      // Voxel size
      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_voxelSize", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, voxelSize);
      }));
   }

   /**
    * Updates the GPU mesh with a list of occupied voxel positions.
    *
    * @param occupiedVoxels    List of Vector3 positions in world coordinates
    * @param voxelSizeInMeters size of a voxel in meters
    */
   public void update(List<Vector3> occupiedVoxels, float voxelSizeInMeters)
   {
      this.voxelSize = voxelSizeInMeters;

      Mesh mesh = renderable.meshPart.mesh;
      FloatBuffer buffer = mesh.getVerticesBuffer(true);

      int voxelCount = occupiedVoxels.size();
      int vertexCount = voxelCount * CUBE_VERTEX_COUNT;
      renderable.meshPart.size = vertexCount;

      int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES; // 6

      buffer.clear();
      buffer.limit(vertexCount * floatsPerVertex);

      for (int i = 0; i < voxelCount; i++)
      {
         Vector3 center = occupiedVoxels.get(i);
         float cx = center.x;
         float cy = center.y;
         float cz = center.z;

         int baseVertexIndex = i * CUBE_VERTEX_COUNT;

         for (int v = 0; v < CUBE_VERTEX_COUNT; v++)
         {
            int vertexIndex = baseVertexIndex + v;
            int floatIndex = vertexIndex * floatsPerVertex;

            int cubeOffset = v * 3;
            float lx = CUBE_VERTS[cubeOffset];
            float ly = CUBE_VERTS[cubeOffset + 1];
            float lz = CUBE_VERTS[cubeOffset + 2];

            // a_cubePos
            buffer.put(floatIndex,     lx);
            buffer.put(floatIndex + 1, ly);
            buffer.put(floatIndex + 2, lz);

            // a_voxelCenter
            buffer.put(floatIndex + 3, cx);
            buffer.put(floatIndex + 4, cy);
            buffer.put(floatIndex + 5, cz);
         }
      }
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

   public boolean hasBeenCreated()
   {
      return hasBeenCreated;
   }

   public List<Vector3> extractOccupiedVoxels(GpuMat voxelMap, int cellsPerAxis, float voxelSize, float gridCenterX, float gridCenterY)
   {
      Mat cpuMat = new Mat();
      voxelMap.download(cpuMat);

      IntPointer dataPtr = new IntPointer(cpuMat.data());

      List<Vector3> occupiedVoxels = new ArrayList<>();

      // 2. Iterate over voxel indices
      for (int iz = 0; iz < cellsPerAxis; iz++)
      {
         for (int iy = 0; iy < cellsPerAxis; iy++)
         {
            for (int ix = 0; ix < cellsPerAxis; ix++)
            {
               int flatIndex = ix + iy * cellsPerAxis + iz * cellsPerAxis * cellsPerAxis;
               if (dataPtr.get(flatIndex) != 0)
               {
                  float half = 0.5f * cellsPerAxis * voxelSize;
                  float x = (ix * voxelSize) - half + gridCenterX;
                  float y = (iy * voxelSize) - half + gridCenterY;
                  float z = (iz * voxelSize) - half; // or + gridCenterZ if you have one

                  occupiedVoxels.add(new Vector3(x, y, z));
               }
            }
         }
      }

      dataPtr.close();
      cpuMat.close();

      return occupiedVoxels;
   }
}
