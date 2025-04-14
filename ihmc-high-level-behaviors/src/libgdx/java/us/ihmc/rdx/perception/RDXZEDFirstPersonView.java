package us.ihmc.rdx.perception;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXZEDFirstPersonView
{
   private final RDX3DScene scene3D;
   private final RDX3DPanel firstPerson3DPanel;
   private Runnable firstPersonBackgroundRenderer;

   public RDXZEDFirstPersonView(RDXBaseUI baseUI)
   {
      scene3D = new RDX3DScene();
      scene3D.create(RDXSceneLevel.values());
      scene3D.addDefaultLighting();

      firstPerson3DPanel = new RDX3DPanel("ZED 1st Person View", false);
      baseUI.add3DPanel(firstPerson3DPanel, scene3D);
      firstPerson3DPanel.getCamera3D().setInputEnabled(false);
   }

   public void update(Mat bgraMat, float vFov, ReferenceFrame zedLeftEyeFrame)
   {
      if (firstPersonBackgroundRenderer == null)
      {
         firstPersonBackgroundRenderer = () ->
         {
            Mat rgbaMat = new Mat(bgraMat.rows(), bgraMat.cols(), opencv_core.CV_8UC4);
            opencv_imgproc.cvtColor(bgraMat, rgbaMat, opencv_imgproc.COLOR_BGRA2RGBA);
            opencv_core.flip(rgbaMat, rgbaMat, OpenCVTools.FLIP_Y);

            int viewportWidth = (int) Math.floor(firstPerson3DPanel.getViewportSizeX()) * firstPerson3DPanel.getAntiAliasing();
            int viewportHeight = (int) Math.floor(firstPerson3DPanel.getViewportSizeY()) * firstPerson3DPanel.getAntiAliasing();

            float cameraAspect = (float) rgbaMat.cols() / rgbaMat.rows();
            float viewportAspect = (float) viewportWidth / viewportHeight;

            int cameraRenderWidth = Math.round(viewportHeight * cameraAspect);
            int offsetX = 0;
            int shiftAmountX = 0;
            if (cameraAspect > viewportAspect) // Camera extends beyond side edges
            {
               shiftAmountX = (cameraRenderWidth - viewportWidth) / 2;
            }
            else // Camera ends within side edges
            {
               offsetX = (viewportWidth - cameraRenderWidth) / 2;
            }

            Size size = new Size(cameraRenderWidth, viewportHeight);
            Mat scaledRgbaMat = new Mat(size, opencv_core.CV_8UC4);
            opencv_imgproc.resize(rgbaMat, scaledRgbaMat, size);
            size.close();
            rgbaMat.close();

            Mat renderMat = scaledRgbaMat;
            Mat shiftedRgbaMat = null;
            if (shiftAmountX > 0) // We need to shift the image to the left in the case the camera extends beyond the side edges
            {
               shiftedRgbaMat = new Mat(scaledRgbaMat.size(), scaledRgbaMat.type());
               Mat rangeToShift = scaledRgbaMat.colRange(shiftAmountX, scaledRgbaMat.cols());
               rangeToShift.copyTo(shiftedRgbaMat);
               renderMat = shiftedRgbaMat;
               rangeToShift.close();
            }

            firstPerson3DPanel.getFrameBuffer().getColorBufferTexture().bind();
            int glTarget = firstPerson3DPanel.getFrameBuffer().getColorBufferTexture().glTarget;
            Gdx.gl.glTexSubImage2D(glTarget, 0, offsetX, 0, renderMat.cols(), renderMat.rows(),
                                   GL20.GL_RGBA, GL20.GL_UNSIGNED_BYTE, renderMat.data().asBuffer());

            if (shiftedRgbaMat != null)
               shiftedRgbaMat.close();
            scaledRgbaMat.close();
         };

         firstPerson3DPanel.getCamera3D().setVerticalFieldOfView(vFov);
      }
      firstPerson3DPanel.setBackgroundRenderer(firstPersonBackgroundRenderer);
      firstPerson3DPanel.getCamera3D().setPose(zedLeftEyeFrame.getTransformToWorldFrame());
   }

   public RDX3DScene getScene3D()
   {
      return scene3D;
   }

   public void destroy()
   {
      scene3D.dispose();
      firstPerson3DPanel.dispose();
   }
}
