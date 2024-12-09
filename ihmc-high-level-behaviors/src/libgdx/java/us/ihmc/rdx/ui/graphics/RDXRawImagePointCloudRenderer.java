package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;
import us.ihmc.rdx.tools.LibGDXTools;

import java.util.Collection;
import java.util.Iterator;
import java.util.stream.IntStream;

public class RDXRawImagePointCloudRenderer implements RenderableProvider
{
   public enum InputMethod
   {
      POINT_CLOUD(),
      DEPTH_IMAGE("INPUT_DEPTH_IMAGE"),
      DEPTH_AND_COLOR_IMAGE("INPUT_DEPTH_IMAGE", "INPUT_COLOR_IMAGE");

      private final String[] vertexFlags;
      InputMethod(String... vertexFlags)
      {
         this.vertexFlags = vertexFlags;
      }
   }

   public enum ColoringMethod
   {
      DEFAULT, GRADIENT_WORLD_Z, GRADIENT_SENSOR_X, COLOR_IMAGE
   }

   private Renderable renderable;
   private final InputMethod inputMethod;

   private final VertexAttributes vertexAttributes = new VertexAttributes(VertexAttribute.Position());
   private final int floatsPerVertex = vertexAttributes.vertexSize / Float.BYTES;
   private float[] vertices;

   // GENERALLY NEEDED UNIFORMS
   private final RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, shader.camera.viewportWidth);
   });

   private float pointScale = 0.01f;
   private final RDXUniform pointScaleUniform = RDXUniform.createGlobalUniform("u_pointScale", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, pointScale);
   });

   private final Color defaultPointColor = new Color(Color.WHITE);
   private final RDXUniform defaultPointColorUniform = RDXUniform.createGlobalUniform("u_defaultPointColor", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, defaultPointColor);
   });

   private ColoringMethod coloringMethod = ColoringMethod.DEFAULT;
   private final RDXUniform coloringMethodUniform = RDXUniform.createGlobalUniform("u_coloringMethod", (shader, inputID, renderable, combinedAttributes) ->
   {
      shader.set(inputID, coloringMethod.ordinal());
   });

   // DEPTH IMAGE UNIFORM DATA
   private CameraIntrinsics depthIntrinsics = new CameraIntrinsics();
   private float depthDiscretization = 0.0f;
   private final FixedFramePose3DBasics depthPose = new FramePose3D();
   private final Matrix4 libGDXDepthTransform = new Matrix4();
   private final RigidBodyTransform tempDepthTransform = new RigidBodyTransform();

   // COLOR IMAGE UNIFORM DATA
   private Pixmap colorImagePixmap = new Pixmap(1, 1, Format.RGBA8888);
   private BytePointer pixmapDataPointer = new BytePointer(colorImagePixmap.getPixels());
   private final Texture colorImageTexture = new Texture(colorImagePixmap);
   private CameraIntrinsics colorIntrinsics = new CameraIntrinsics();
   private final RigidBodyTransform depthToColorTransform = new RigidBodyTransform();
   private final Matrix4 libGDXColorTransform = new Matrix4();
   private final RigidBodyTransform tempColorTransform = new RigidBodyTransform();

   public RDXRawImagePointCloudRenderer(InputMethod inputMethod)
   {
      this.inputMethod = inputMethod;
   }

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
      shader.create(inputMethod.vertexFlags, null);
      shader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      shader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);
      shader.registerUniform(screenWidthUniform);
      shader.registerUniform(pointScaleUniform);
      shader.registerUniform(defaultPointColorUniform);
      shader.registerUniform(coloringMethodUniform);

      if (inputMethod == InputMethod.DEPTH_IMAGE || inputMethod == InputMethod.DEPTH_AND_COLOR_IMAGE)
         registerDepthImageUniforms(shader);

      if (inputMethod == InputMethod.DEPTH_AND_COLOR_IMAGE)
         registerColorImageUniforms(shader);

      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   private void registerDepthImageUniforms(RDXShader rdxShader)
   {
      RDXUniform depthIntrinsicsUniform = RDXUniform.createGlobalUniform("u_depthIntrinsics", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, (float) depthIntrinsics.getFx(), (float) depthIntrinsics.getFy(), (float) depthIntrinsics.getCx(), (float) depthIntrinsics.getCy());
      });
      rdxShader.registerUniform(depthIntrinsicsUniform);

      RDXUniform depthDiscretizationUniform = RDXUniform.createGlobalUniform("u_depthDiscretization", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, depthDiscretization);
      });
      rdxShader.registerUniform(depthDiscretizationUniform);

      RDXUniform depthTransformUniform = RDXUniform.createGlobalUniform("u_depthTransform", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, libGDXDepthTransform);
      });
      rdxShader.registerUniform(depthTransformUniform);
   }

   private void registerColorImageUniforms(RDXShader rdxShader)
   {
      RDXUniform colorImageTextureUniform = RDXUniform.createGlobalUniform("u_colorTexture", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, colorImageTexture);
      });
      rdxShader.registerUniform(colorImageTextureUniform);

      RDXUniform colorIntrinsicsUniform = RDXUniform.createGlobalUniform("u_colorIntrinsics", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, (float) colorIntrinsics.getFx(), (float) colorIntrinsics.getFy(), (float) colorIntrinsics.getCx(), (float) colorIntrinsics.getCy());
      });
      rdxShader.registerUniform(colorIntrinsicsUniform);

      RDXUniform depthTransformUniform = RDXUniform.createGlobalUniform("u_depthToColorTransform", (shader, inputID, renderable, combinedAttributes) ->
      {
         LibGDXTools.toLibGDX(depthToColorTransform, tempColorTransform, libGDXColorTransform);
         shader.set(inputID, libGDXColorTransform);
      });
      rdxShader.registerUniform(depthTransformUniform);
   }

   public void updateMesh(RawImage depthImage)
   {
      if (depthImage.get() == null)
         return;

      // Update depth image uniforms
      depthIntrinsics = depthImage.getIntrinsicsCopy();
      depthDiscretization = depthImage.getDepthDiscretization();
      depthPose.set(depthImage.getPose());
      LibGDXTools.toLibGDX(depthPose, tempDepthTransform, libGDXDepthTransform);

      int width = depthImage.getWidth();
      int height = depthImage.getHeight();
      int pixelCount = width * height;

      // Ensure correct length
      if (vertices.length != floatsPerVertex * pixelCount)
         vertices = new float[floatsPerVertex * pixelCount];

      // Get depth data
      short[] depthData = new short[pixelCount];
      ShortPointer depthPointer = new ShortPointer(depthImage.getCpuImageMat().data());
      depthPointer.get(depthData);
      depthPointer.close();

      IntStream.range(0, height * width).parallel().unordered().forEach(i -> {
         int offset = i * floatsPerVertex;
         int x = i % width;
         int y = i / width;
         vertices[offset] = depthData[i];
         vertices[offset + 1] = x;
         vertices[offset + 2] = y;
      });

      renderable.meshPart.size = pixelCount;
      renderable.meshPart.mesh.setVertices(vertices);

      depthImage.release();
   }

   public void updateMesh(RawImage depthImage, RawImage colorImage)
   {
      if (depthImage.get() == null)
         return;

      if (colorImage.get() == null)
         return;

      // Update depth image stuff
      updateMesh(depthImage);

      // Update color image uniforms
      colorIntrinsics = colorImage.getIntrinsicsCopy();
      depthPose.getReferenceFrame().getTransformToDesiredFrame(depthToColorTransform, colorImage.getPose().getReferenceFrame());
      LibGDXTools.toLibGDX(depthToColorTransform, tempColorTransform, libGDXColorTransform);

      // Reallocate pixmap and data pointer if image size changed
      if (colorImagePixmap.getWidth() != colorImage.getWidth() || colorImagePixmap.getHeight() != colorImage.getHeight())
      {
         colorImagePixmap.dispose();
         colorImagePixmap = new Pixmap(colorImage.getWidth(), colorImage.getHeight(), Format.RGBA8888);

         pixmapDataPointer.close();
         pixmapDataPointer = new BytePointer(colorImagePixmap.getPixels());

         colorImageTexture.load(new PixmapTextureData(colorImagePixmap, Format.RGBA8888, false, false));
      }

      // Use a mat to copy RGBA data into the pixmap data
      Mat rgbaMat = new Mat(colorImage.getHeight(), colorImage.getWidth(), opencv_core.CV_8UC4, pixmapDataPointer);
      colorImage.getPixelFormat().convertToRGBA(colorImage.getCpuImageMat(), rgbaMat);

      // Draw the new image to the texture
      colorImageTexture.draw(colorImagePixmap, 0, 0);

      rgbaMat.release();
      depthImage.release();
      colorImage.release();
   }

   public void updateMesh(Collection<? extends Point3DReadOnly> points)
   {
      if (vertices.length != floatsPerVertex * points.size())
         vertices = new float[floatsPerVertex * points.size()];

      // Copy points over
      Iterator<? extends Point3DReadOnly> pointIterator = points.iterator();
      for (int i = 0; i < points.size() && pointIterator.hasNext(); ++i)
      {
         Point3DReadOnly point = pointIterator.next();
         int offset = i * floatsPerVertex;

         vertices[offset] = point.getX32();
         vertices[offset + 1] = point.getY32();
         vertices[offset + 2] = point.getZ32();
      }

      renderable.meshPart.size = points.size();
      renderable.meshPart.mesh.setVertices(vertices);
   }

   public void updateMesh(Point3DReadOnly[] points)
   {
      // Ensure correct length
      if (vertices.length != floatsPerVertex * points.length)
         vertices = new float[floatsPerVertex * points.length];

      // Copy points over
      for (int i = 0; i < points.length; ++i)
      {
         int offset = i * floatsPerVertex;

         vertices[offset] = points[i].getX32();
         vertices[offset + 1] = points[i].getY32();
         vertices[offset + 2] = points[i].getZ32();
      }

      renderable.meshPart.size = points.length;
      renderable.meshPart.mesh.setVertices(vertices);
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
      pointScale = size;
   }

   public void setDefaultPointColor(Color color)
   {
      defaultPointColor.set(color);
   }

   public void setColoringMethod(ColoringMethod method)
   {
      coloringMethod = method;
   }
}