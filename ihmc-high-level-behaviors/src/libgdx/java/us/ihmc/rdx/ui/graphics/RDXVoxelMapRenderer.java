package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
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
import java.util.stream.IntStream;

/**
 * Renders a voxel map as a point cloud.
 * Each occupied voxel is sent to the GPU as a single point.
 */
public class RDXVoxelMapRenderer implements RenderableProvider
{
   private final Renderable renderable = new Renderable();

   // Vertex attribute: position only (x, y, z)
   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position());

   // Uniforms
   private float voxelSize = 0.05f; // default, set with update()
   private boolean hasBeenCreated = false;

   private final int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;

   public void create(int maxVoxels)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();

      boolean isStatic = false;
      int maxIndices = 0;

      renderable.meshPart.mesh = new Mesh(isStatic, maxVoxels, maxIndices, vertexAttributes);

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

      // Screen width
      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, shader.camera.viewportWidth);
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

      FloatBuffer buffer = renderable.meshPart.mesh.getVerticesBuffer(true);

      int voxelCount = occupiedVoxels.size();
      if (renderable.meshPart.size != voxelCount)
      {
         renderable.meshPart.size = voxelCount;
         buffer.limit(floatsPerVertex * voxelCount);
      }

      IntStream.range(0, voxelCount).parallel().unordered().forEach(i ->
      {
         int offset = i * floatsPerVertex;
         buffer.put(offset, occupiedVoxels.get(i).x);
         buffer.put(offset + 1, occupiedVoxels.get(i).y);
         buffer.put(offset + 2, occupiedVoxels.get(i).z);
      });
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