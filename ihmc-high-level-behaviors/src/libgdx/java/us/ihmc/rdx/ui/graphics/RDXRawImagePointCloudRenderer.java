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
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.bytedeco.javacpp.ShortPointer;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXRawImagePointCloudRenderer implements RenderableProvider
{
   private Renderable renderable;

   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position(),
                                                                          VertexAttribute.ColorUnpacked(),
                                                                          new VertexAttribute(Usage.Generic, 1,"a_size"));
   private final int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;
   private float[] vertices;

   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, shader.camera.viewportWidth);
   });

   private CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
   private final RDXUniform cameraIntrinsicsUniform = RDXUniform.createGlobalUniform("u_depthIntrinsics", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, (float) cameraIntrinsics.getFx(), (float) cameraIntrinsics.getFy(), (float) cameraIntrinsics.getCx(), (float) cameraIntrinsics.getCy());
   });

   private float depthDiscretization = 0.0f;
   private final RDXUniform depthDiscretizationUniform = RDXUniform.createGlobalUniform("u_depthDiscretization", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, depthDiscretization);
   });

   private final FixedFramePose3DBasics sensorPose = new FramePose3D();
   private final Matrix4 libGDXMatrix = new Matrix4();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RDXUniform sensorTransformUniform = RDXUniform.createGlobalUniform("u_depthToWorldTransform", (shader, inputID, renderable, combinedAttributes) ->
   {
      LibGDXTools.toLibGDX(sensorPose, tempTransform, libGDXMatrix);
      shader.set(inputID, libGDXMatrix);
   });

   private float pointScale = 0.01f;

   public void create(int maxPoints)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      vertices = new float[floatsPerVertex * maxPoints];

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxPoints, maxIndices, vertexAttributes);

      RDXShader shader = new RDXShader(getClass());
      shader.create();
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.registerUniform(screenWidthUniform);

      shader.registerUniform(cameraIntrinsicsUniform);
      shader.registerUniform(depthDiscretizationUniform);
      shader.registerUniform(sensorTransformUniform);

      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   public void updateMesh(RawImage depthImage)
   {
      if (depthImage.get() == null)
         return;

      cameraIntrinsics = depthImage.getIntrinsicsCopy();
      sensorPose.set(depthImage.getPose());
      depthDiscretization = depthImage.getDepthDiscretization();

      int width = depthImage.getWidth();
      int height = depthImage.getHeight();
      int pixelCount = width * height;
      int x = 0;
      int y = 0;

      // Ensure correct length
      if (vertices.length != floatsPerVertex * pixelCount)
         vertices = new float[floatsPerVertex * pixelCount];

      // Get depth data
      short[] depthData = new short[pixelCount];
      ShortPointer depthPointer = new ShortPointer(depthImage.getCpuImageMat().data());
      depthPointer.get(depthData);
      depthPointer.close();

      for (int i = 0; i < pixelCount; ++i)
      {
         int offset = i * floatsPerVertex;

         // Depth pixel
         vertices[offset + 1] = x;
         vertices[offset + 2] = y;
         vertices[offset + 0] = depthData[i];

         // Color
         vertices[offset + 3] = Color.WHITE.r;
         vertices[offset + 4] = Color.WHITE.g;
         vertices[offset + 5] = Color.WHITE.b;
         vertices[offset + 6] = Color.WHITE.a;

         // Point size
         vertices[offset + 7] = pointScale;

         // Update pixel coordinates
         x = ++x % width;
         if (x == 0)
            y = ++y % height;
      }

      renderable.meshPart.size = pixelCount;
      renderable.meshPart.mesh.setVertices(vertices);

      depthImage.release();
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

   public void setPointScale(float size)
   {
      this.pointScale = size;
   }
}