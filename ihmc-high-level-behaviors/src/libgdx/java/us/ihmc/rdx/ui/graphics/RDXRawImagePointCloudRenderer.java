package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.math.Matrix4;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.Nullable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer;
import us.ihmc.rdx.shader.RDXShader;
import us.ihmc.rdx.shader.RDXUniform;
import us.ihmc.rdx.tools.LibGDXTools;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import static us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod.*;

public class RDXRawImagePointCloudRenderer extends AbstractRDXPointCloudRenderer
{
   private final VertexAttributes vertexAttributes;

   // DEPTH IMAGE UNIFORM DATA
   private CameraIntrinsics depthIntrinsics = new CameraIntrinsics();
   private float depthDiscretization = 0.0f;
   private final RigidBodyTransform depthTransform = new RigidBodyTransform();
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
   private final boolean inputColorizedDepth;

   public RDXRawImagePointCloudRenderer()
   {
      this(true);
   }

   public RDXRawImagePointCloudRenderer(boolean enableColorImageInput)
   {
      this(enableColorImageInput, false);
   }

   public RDXRawImagePointCloudRenderer(boolean enableColorImageInput, boolean inputColorizedDepth)
   {
      this.enableColorImageInput = enableColorImageInput;
      this.inputColorizedDepth = inputColorizedDepth;

      vertexAttributes = new VertexAttributes(new VertexAttribute(Usage.Generic, inputColorizedDepth ? 3 : 1, "a_depthData"));
   }

   public void updateMesh(RawImage depthImage)
   {
      if (depthImage.get() == null)
         return;

      if (inputColorizedDepth && depthImage.getOpenCVType() != opencv_core.CV_8UC3)
         throw new IllegalArgumentException("Depth image's OpenCV type must be 8UC3 for colorized depth input");
      else if (!inputColorizedDepth && depthImage.getOpenCVType() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Depth image's OpenCV type must be 16UC1 for non-colorized input");

      // Update depth image uniforms
      depthIntrinsics = depthImage.getIntrinsicsCopy();
      depthDiscretization = depthImage.getDepthDiscretization();
      depthTransform.set(depthImage.getTransformToWorld());
      LibGDXTools.toLibGDX(depthTransform, tempDepthTransform, libGDXDepthTransform);

      int width = depthImage.getWidth();
      int height = depthImage.getHeight();
      int elementCount = width * height;

      if (inputColorizedDepth)
         elementCount *= 3;

      FloatBuffer verticesBuffer = renderable.meshPart.mesh.getVerticesBuffer(true); // Mark dirty

      // Ensure correct length and initialize vertices buffer
      if (renderable.meshPart.size != elementCount)
      {
         renderable.meshPart.size = elementCount;
         verticesBuffer.limit(elementCount);
      }

      /* NOTE:
       * We need to copy the short values from the depth image (Mat) to the vertices buffer (FloatBuffer) as floats.
       * In both objects, the data is in native memory. Copying native -> java -> native is slow,
       * so we use the Mat#convertTo() method which performs the short to float conversion and memory copy natively.
       */

      // Wrap the vertices buffer into a pointer, than into a Mat
      int matrixType = inputColorizedDepth ? opencv_core.CV_32FC3 : opencv_core.CV_32FC1;
      FloatPointer verticesPointer = new FloatPointer(verticesBuffer);
      Mat verticesMat = new Mat(height, width, matrixType, verticesPointer);

      // Convert the depth data to float, and put it into the vertex buffer
      depthImage.getCpuImageMat().convertTo(verticesMat, matrixType);

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
      depthToColorTransform.setAndInvert(colorImage.getTransformToWorld());
      depthToColorTransform.multiply(depthTransform);
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
   public ColoringMethod[] getAvailableColoringMethods()
   {
      if (enableColorImageInput)
         return new ColoringMethod[] {DEFAULT, GRADIENT_WORLD_Z, GRADIENT_SENSOR_X, COLOR_IMAGE};

      return new ColoringMethod[] {DEFAULT, GRADIENT_WORLD_Z, GRADIENT_SENSOR_X};
   }

   @Nullable
   @Override
   protected String[] getFragmentShaderFlags()
   {
      return null;
   }

   @Nullable
   @Override
   protected String[] getVertexShaderFlags()
   {
      ArrayList<String> flags = new ArrayList<>();
      if (enableColorImageInput)
         flags.add("INPUT_COLOR_IMAGE");
      if (inputColorizedDepth)
         flags.add("INPUT_COLORIZED_DEPTH");

      return flags.toArray(String[]::new);
   }

   @Override
   protected VertexAttributes getVertexAttributes()
   {
      return vertexAttributes;
   }

   @Override
   protected void registerUniforms(RDXShader rdxShader)
   {
      registerGeneralUniforms(rdxShader);
      registerDepthImageUniforms(rdxShader);

      if (enableColorImageInput)
         registerColorImageUniforms(rdxShader);
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
         shader.set(inputID, libGDXColorTransform);
      });
      rdxShader.registerUniform(depthTransformUniform);
   }
}