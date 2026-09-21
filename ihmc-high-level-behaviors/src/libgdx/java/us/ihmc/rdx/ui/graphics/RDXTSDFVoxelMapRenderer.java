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
 * Renders a TSDF voxel map as transparent cubes.
 *
 * <p>Color converges to white at the isosurface (tsdf ≈ 0) and diverges to blue for positive TSDF
 * (free space) or red for negative (occluded).  Opacity is {@code alpha = 1 - |tsdf|}: fully
 * opaque at the surface, fully transparent at ±1.  Converging to a neutral color at zero prevents
 * blue/red flickering caused by sign noise right at the surface.
 *
 * <p>Voxels with {@code |tsdf| > TSDF_RENDER_CUTOFF} are culled before upload — they would be
 * almost fully transparent and would contribute only noise and vertex bandwidth.
 */
public class RDXTSDFVoxelMapRenderer implements RenderableProvider
{
   /** Voxels with |tsdf| above this threshold are skipped (alpha would be < 0.01). */
   private static final float TSDF_RENDER_CUTOFF = 0.99f;

   private static final int FLOATS_PER_VERTEX = 4;   // world x, y, z of center + tsdf value
   private static final int VERTICES_PER_VOXEL = 36; // 6 faces × 2 triangles × 3 vertices

   private final Renderable renderable = new Renderable();
   private final VertexAttributes vertexAttributes = new VertexAttributes(
         new VertexAttribute(VertexAttributes.Usage.Generic, 4, "a_positionAndTSDF"));

   private float voxelSize;
   private float[] tsdfArray;
   private float[] verticesArray;

   private final RigidBodyTransform mapToWorld = new RigidBodyTransform();
   private final Point3D voxelPos = new Point3D();
   private final Matrix3 mapRotation = new Matrix3();

   /**
    * @param maxVoxels total voxel count (N³); used to pre-allocate vertex buffer and scratch arrays.
    */
   public void create(int maxVoxels)
   {
      renderable.meshPart.primitiveType = GL41.GL_TRIANGLES;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      if (renderable.meshPart.mesh != null)
         renderable.meshPart.mesh.dispose();
      renderable.meshPart.mesh = new Mesh(false, maxVoxels * VERTICES_PER_VOXEL, 0, vertexAttributes);

      tsdfArray = new float[maxVoxels];
      verticesArray = new float[maxVoxels * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX];

      RDXShader shader = new RDXShader(getClass());
      shader.create(true); // enable alpha blending
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
    * Builds the vertex buffer for all renderable voxels.
    *
    * <p>Only voxels with {@code |tsdf| <= TSDF_RENDER_CUTOFF} are emitted; the rest are invisible
    * (alpha < 0.01) and would just waste bandwidth.  The per-vertex layout is a single {@code vec4}
    * {@code [centerX, centerY, centerZ, tsdfValue]} shared by all 36 vertices of a cube; the vertex
    * shader expands each to the correct cube corner.</p>
    *
    * @param voxelMap TSDF voxel map whose {@code voxelMapData} holds float values in [-1, 1]
    */
   public void update(VoxelMap voxelMap)
   {
      this.voxelSize = voxelMap.getVoxelSize();
      int mapSizeX = voxelMap.getSizeX();
      int mapSizeY = voxelMap.getSizeY();
      int mapSizeZ = voxelMap.getSizeZ();
      int totalVoxels = voxelMap.getVoxelCount();

      FloatPointer cpuData = voxelMap.getCpuData();
      cpuData.position(0).get(tsdfArray, 0, totalVoxels);

      mapToWorld.set(voxelMap.getOrigin());

      RotationMatrixReadOnly r = mapToWorld.getRotation();
      float[] m = mapRotation.val;
      m[Matrix3.M00] = (float) r.getM00(); m[Matrix3.M10] = (float) r.getM10(); m[Matrix3.M20] = (float) r.getM20();
      m[Matrix3.M01] = (float) r.getM01(); m[Matrix3.M11] = (float) r.getM11(); m[Matrix3.M21] = (float) r.getM21();
      m[Matrix3.M02] = (float) r.getM02(); m[Matrix3.M12] = (float) r.getM12(); m[Matrix3.M22] = (float) r.getM22();

      int renderedCount = 0;
      int flatIndex = 0;
      for (int ix = 0; ix < mapSizeX; ix++)
      {
         for (int iy = 0; iy < mapSizeY; iy++)
         {
            for (int iz = 0; iz < mapSizeZ; iz++, flatIndex++)
            {
               float tsdf = tsdfArray[flatIndex];
               if (Math.abs(tsdf) > TSDF_RENDER_CUTOFF)
                  continue;

               voxelPos.set((ix - (mapSizeX - 1) / 2.0) * voxelSize,
                            (iy - (mapSizeY - 1) / 2.0) * voxelSize,
                            (iz - (mapSizeZ - 1) / 2.0) * voxelSize);
               mapToWorld.transform(voxelPos);

               float wx = (float) voxelPos.getX();
               float wy = (float) voxelPos.getY();
               float wz = (float) voxelPos.getZ();
               int base = renderedCount * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX;
               for (int v = 0; v < VERTICES_PER_VOXEL; v++)
               {
                  int offset = base + v * FLOATS_PER_VERTEX;
                  verticesArray[offset]     = wx;
                  verticesArray[offset + 1] = wy;
                  verticesArray[offset + 2] = wz;
                  verticesArray[offset + 3] = tsdf;
               }
               renderedCount++;
            }
         }
      }

      renderable.meshPart.size = renderedCount * VERTICES_PER_VOXEL;

      if (renderedCount == 0)
         return;

      int floatCount = renderedCount * VERTICES_PER_VOXEL * FLOATS_PER_VERTEX;
      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true);
      verticesBuffer.limit(floatCount);
      verticesBuffer.position(0);
      verticesBuffer.put(verticesArray, 0, floatCount);
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
