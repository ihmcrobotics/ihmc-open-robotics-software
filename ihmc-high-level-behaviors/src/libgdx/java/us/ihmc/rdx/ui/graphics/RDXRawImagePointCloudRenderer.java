package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
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
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;
import us.ihmc.rdx.tools.LibGDXTools;

import java.nio.FloatBuffer;

public class RDXRawImagePointCloudRenderer implements RenderableProvider
{
   public enum ColoringMethod
   {
      DEFAULT, GRADIENT_WORLD_Z, GRADIENT_SENSOR_X, COLOR_IMAGE
   }

   private Renderable renderable;

   private final VertexAttribute depthDataAttribute = new VertexAttribute(Usage.Generic, 1, "a_depthData");

   // GENERALLY NEEDED UNIFORMS
   private float pointScale = 0.01f;

   private final Color defaultPointColor = new Color(Color.WHITE);
   private ColoringMethod coloringMethod = ColoringMethod.DEFAULT;

   // DEPTH IMAGE UNIFORM DATA
   private CameraIntrinsics depthIntrinsics = new CameraIntrinsics();
   private float depthDiscretization = 0.0f;
   private final FixedFramePose3DBasics depthPose = new FramePose3D();
   private final Matrix4 libGDXDepthTransform = new Matrix4();
   private final RigidBodyTransform tempDepthTransform = new RigidBodyTransform();

   // COLOR IMAGE UNIFORM DATA
   private Pixmap colorImagePixmap;
   private BytePointer pixmapDataPointer;
   private Texture colorImageTexture;
   private CameraIntrinsics colorIntrinsics = new CameraIntrinsics();
   private final RigidBodyTransform depthToColorTransform = new RigidBodyTransform();
   private final Matrix4 libGDXColorTransform = new Matrix4();
   private final RigidBodyTransform tempColorTransform = new RigidBodyTransform();

   private final boolean enableColorImageInput;

   public RDXRawImagePointCloudRenderer(boolean enableColorImageInput)
   {
      this.enableColorImageInput = enableColorImageInput;
   }

   public void create(int maxPoints)
   {
      GL41.glEnable(GL41.GL_VERTEX_PROGRAM_POINT_SIZE);

      renderable = new Renderable();
      renderable.meshPart.primitiveType = GL41.GL_POINTS;
      renderable.meshPart.offset = 0;
      renderable.material = new Material(PBRColorAttribute.createBaseColorFactor(Color.WHITE));

      boolean isStatic = false;
      int maxIndices = 0;
      renderable.meshPart.mesh = new Mesh(isStatic, maxPoints, maxIndices, depthDataAttribute);

      String[] vertexFlags = { "INPUT_COLOR_IMAGE" };
      if (!enableColorImageInput)
         vertexFlags = null;

      RDXShader shader = new RDXShader(getClass());
      shader.create(vertexFlags, null);
      registerGeneralUniforms(shader);
      registerDepthImageUniforms(shader);

      if (enableColorImageInput)
         registerColorImageUniforms(shader);

      shader.init(renderable);
      renderable.shader = shader.getBaseShader();
   }

   private void registerGeneralUniforms(RDXShader rdxShader)
   {
      rdxShader.getBaseShader().register(DefaultShader.Inputs.viewTrans, DefaultShader.Setters.viewTrans);
      rdxShader.getBaseShader().register(DefaultShader.Inputs.projTrans, DefaultShader.Setters.projTrans);

      RDXUniform screenWidthUniform = RDXUniform.createGlobalUniform("u_screenWidth", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, shader.camera.viewportWidth);
      });
      rdxShader.registerUniform(screenWidthUniform);

      RDXUniform pointScaleUniform = RDXUniform.createGlobalUniform("u_pointScale", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, pointScale);
      });
      rdxShader.registerUniform(pointScaleUniform);

      RDXUniform defaultPointColorUniform = RDXUniform.createGlobalUniform("u_defaultPointColor", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, defaultPointColor);
      });
      rdxShader.registerUniform(defaultPointColorUniform);

      RDXUniform coloringMethodUniform = RDXUniform.createGlobalUniform("u_coloringMethod", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, coloringMethod.ordinal());
      });

      rdxShader.registerUniform(coloringMethodUniform);
   }

   private void registerDepthImageUniforms(RDXShader rdxShader)
   {
      RDXUniform depthIntrinsicsUniform = RDXUniform.createGlobalUniform("u_depthIntrinsics", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, (float) depthIntrinsics.getFx(), (float) depthIntrinsics.getFy(), (float) depthIntrinsics.getCx(), (float) depthIntrinsics.getCy());
      });
      rdxShader.registerUniform(depthIntrinsicsUniform);

      RDXUniform depthImageWidthUniform = RDXUniform.createGlobalUniform("u_depthImageWidth", (shader, inputID, renderable, combinedAttributes) ->
      {
         shader.set(inputID, depthIntrinsics.getWidth());
      });
      rdxShader.registerUniform(depthImageWidthUniform);

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
      colorImagePixmap = new Pixmap(1, 1, Format.RGBA8888);
      pixmapDataPointer = new BytePointer(colorImagePixmap.getPixels());
      colorImageTexture = new Texture(colorImagePixmap);

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

      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty

      // Ensure correct length and initialize vertices buffer
      if (renderable.meshPart.size != pixelCount)
      {
         renderable.meshPart.size = pixelCount;
         verticesBuffer.limit(pixelCount);
      }

      // Wrap the vertices buffer into a pointer, than into a Mat
      FloatPointer verticesPointer = new FloatPointer(verticesBuffer);
      Mat verticesMat = new Mat(height, width, opencv_core.CV_32FC1, verticesPointer);

      // Convert the depth data to float, and put it into the vertex buffer
      depthImage.getCpuImageMat().convertTo(verticesMat, opencv_core.CV_32FC1);

      verticesMat.close();
      verticesPointer.close();
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