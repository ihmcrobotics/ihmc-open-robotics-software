package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.bytedeco.javacpp.FloatPointer;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.voxelMap.VoxelMap;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;

import java.nio.FloatBuffer;

/**
 * Renders an occupancy voxel map produced by {@link us.ihmc.perception.voxelMap.CUDAVoxelMapExtractor}
 * as true 3D cubes, one cube per occupied voxel, colored by world-Z height with simple diffuse shading.
 * Cube faces are axis-aligned with the voxel map frame, not world frame.
 */
public class RDXVoxelMapRenderer implements RenderableProvider
{
   private static final int FLOATS_PER_VERTEX = 3;   // world x, y, z of voxel center
   private static final int VERTICES_PER_VOXEL = 36; // 6 faces × 2 triangles × 3 vertices

   private final Renderable renderable = new Renderable();
   private final VertexAttributes vertexAttributes = new VertexAttributes(new VertexAttribute(VertexAttributes.Usage.Generic, 3, "a_position"));

   private float voxelSize;
   private float[] occupancyArray;
   private float[] positionsArray;

   private final RigidBodyTransform mapToWorld = new RigidBodyTransform();
   private final Point3D voxelPos = new Point3D();
   // Rotation part of mapToWorld, sent to the shader so cube corners are aligned with the map frame
   private final Matrix3 mapRotation = new Matrix3();

   /**
    * @param maxVoxels upper bound on total voxel count (mapSizeX * mapSizeY * mapSizeZ);
    *                  used to pre-allocate the GPU mesh and intermediate Java heap arrays.
    */
   public void create(int maxVoxels)
   {
      renderable.meshPart.primitiveType = GL41.GL_TRIANGLES;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      renderable.meshPart.mesh = new Mesh(false, maxVoxels * VERTICES_PER_VOXEL, 0, vertexAttributes);

      occupancyArray = new float[maxVoxels];
      positionsArray = new float[maxVoxels * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX];

      RDXShader shader = new RDXShader(getClass());
      shader.create();
      registerUniforms(shader);
      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   @SuppressWarnings("CodeBlock2Expr")
   private void registerUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);

      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_voxelSize", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, voxelSize);
      }));

      rdxShader.registerUniform(RDXUniform.createGlobalUniform("u_mapRotation", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, mapRotation);
      }));
   }

   /**
    * Extracts occupied voxel centers from the voxel map, transforms them into world coordinates,
    * and uploads positions to the GPU mesh for rendering.
    * <p>
    * The map is center-anchored: voxel {@code i} along an axis of size {@code N} has local offset
    * {@code (i - (N-1)/2.0) * voxelSize}. For even {@code N} the map origin lies exactly on the
    * boundary between the two middle voxels; for odd {@code N} it lies on a voxel center. Flatten
    * order is {@code i*N²+j*N+k} (i outermost/slowest, k innermost/fastest).
    *
    * @param voxelMap voxel map to render; CPU data is fetched lazily via {@link VoxelMap#getCpuData()}
    */
   public void update(VoxelMap voxelMap)
   {
      this.voxelSize = voxelMap.getVoxelSize();
      int mapSizeX = voxelMap.getSizeX();
      int mapSizeY = voxelMap.getSizeY();
      int mapSizeZ = voxelMap.getSizeZ();
      int totalVoxels = voxelMap.getVoxelCount();

      // One JNI call to bulk-copy native host memory into the Java heap array
      FloatPointer cpuData = voxelMap.getCpuData();
      cpuData.get(occupancyArray, 0, totalVoxels);

      voxelMap.getOrigin().get(mapToWorld);

      // Extract rotation for the shader uniform; cube corners are defined in map frame and rotated to world
      RotationMatrixReadOnly r = mapToWorld.getRotation();
      float[] m = mapRotation.val;
      m[Matrix3.M00] = (float) r.getM00(); m[Matrix3.M10] = (float) r.getM10(); m[Matrix3.M20] = (float) r.getM20();
      m[Matrix3.M01] = (float) r.getM01(); m[Matrix3.M11] = (float) r.getM11(); m[Matrix3.M21] = (float) r.getM21();
      m[Matrix3.M02] = (float) r.getM02(); m[Matrix3.M12] = (float) r.getM12(); m[Matrix3.M22] = (float) r.getM22();

      // Pure Java loop — no JNI crossings
      int occupiedCount = 0;
      int flatIndex = 0;
      for (int ix = 0; ix < mapSizeX; ix++)
      {
         for (int iy = 0; iy < mapSizeY; iy++)
         {
            for (int iz = 0; iz < mapSizeZ; iz++, flatIndex++)
            {
               if (occupancyArray[flatIndex] == 0.0f)
                  continue;

               // Center-anchored: offset of voxel i along axis of size N is (i - (N-1)/2.0) * voxelSize
               voxelPos.set((ix - (mapSizeX - 1) / 2.0) * voxelSize,
                            (iy - (mapSizeY - 1) / 2.0) * voxelSize,
                            (iz - (mapSizeZ - 1) / 2.0) * voxelSize);
               mapToWorld.transform(voxelPos);

               // Write the same world-space center for all 36 vertices of this cube.
               // The vertex shader uses gl_VertexID % 36 to pick the cube corner offset.
               float wx = (float) voxelPos.getX();
               float wy = (float) voxelPos.getY();
               float wz = (float) voxelPos.getZ();
               int base = occupiedCount * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX;
               for (int v = 0; v < VERTICES_PER_VOXEL; v++)
               {
                  int offset = base + v * FLOATS_PER_VERTEX;
                  positionsArray[offset]     = wx;
                  positionsArray[offset + 1] = wy;
                  positionsArray[offset + 2] = wz;
               }
               occupiedCount++;
            }
         }
      }

      renderable.meshPart.size = occupiedCount * VERTICES_PER_VOXEL;

      if (occupiedCount == 0)
         return;

      // One JNI call to bulk-copy the Java heap positions into the GPU vertex buffer
      int floatCount = occupiedCount * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX;
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true);
      verticesBuffer.limit(floatCount);
      verticesBuffer.position(0);
      verticesBuffer.put(positionsArray, 0, floatCount);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderable.meshPart.size > 0)
         renderables.add(renderable);
   }

   public void dispose()
   {
      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
   }
}
