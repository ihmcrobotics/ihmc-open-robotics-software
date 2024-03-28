package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.perception.RDXProjectionSphere;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.annotation.Nullable;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXValveIndexStereoPassthroughDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ValveIndexCameraReaderThread cameraReaderThread = new ValveIndexCameraReaderThread();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(/*RDXProjectionSphere::new*/);
   private volatile boolean running;

   private final FramePose3D leftEyePose = new FramePose3D();
   private final FramePose3D rightEyePose = new FramePose3D();
   //   private ReferenceFrame projectionOriginFrame;

   private final ImDouble projectionZOffset = new ImDouble(1.280632);
   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();

   public RDXValveIndexStereoPassthroughDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            super.create();

            running = true;
            cameraReaderThread.start();

            // Setup projection spheres
            projectionSpheres.get(RobotSide.LEFT).create();
            projectionSpheres.get(RobotSide.RIGHT).create();
            baseUI.getPrimaryScene().addRenderableProvider(RDXValveIndexStereoPassthroughDemo.this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection controls", RDXValveIndexStereoPassthroughDemo.this::renderControls);

            baseUI.create();
         }

         @Override
         public void render()
         {
            super.render();

            boolean hadNewFrame = false;

            for (RobotSide side : RobotSide.values)
            {
               Queue<Mat> rects = cameraReaderThread.getRects(side);

               Mat rect = null;
               while (!rects.isEmpty())
               {
                  rect = rects.remove();

                  if (!rects.isEmpty())
                  {
                     rect.close();
                  }
               }

               if (rect != null && rect.address() != 0)
               {
                  Pixmap pixmap = new Pixmap(rect.cols(), rect.rows(), Pixmap.Format.RGBA8888);
                  BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
                  Mat displayMat = new Mat(rect.rows(), rect.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
                  rect.copyTo(displayMat);
                  Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

                  projectionSpheres.get(side).updateTexture(texture, 0.7f);

                  displayMat.close();
                  pixmap.dispose();
                  rect.close();

                  hadNewFrame = true;
               }
            }

            // Sync (or mirror settings)
            double principlePointX = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointX();
            double principlePointY = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointY();
            projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointX(-principlePointX);
            projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointY(principlePointY);

            if (baseUI.getVRManager().isVRReady() && hadNewFrame)
            {
               for (RDXReferenceFrameGraphic eyeFrameGraphic : eyeFrameGraphics)
                  if (eyeFrameGraphic != null)
                     eyeFrameGraphic.updateFromLastGivenFrame();

               leftEyePose.setToZero(baseUI.getVRManager().getContext().getEyes().get(RobotSide.LEFT).getEyeXForwardZUpFrame());
               rightEyePose.setToZero(baseUI.getVRManager().getContext().getEyes().get(RobotSide.RIGHT).getEyeXForwardZUpFrame());

               RigidBodyTransform leftEyePoseWorld = new RigidBodyTransform(leftEyePose);
               leftEyePoseWorld.preMultiply(leftEyePose.getReferenceFrame().getTransformToRoot());
               RigidBodyTransform rightEyePoseWorld = new RigidBodyTransform(rightEyePose);
               rightEyePoseWorld.preMultiply(rightEyePose.getReferenceFrame().getTransformToRoot());
               LibGDXTools.toLibGDX(leftEyePoseWorld, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);
               LibGDXTools.toLibGDX(rightEyePoseWorld, projectionSpheres.get(RobotSide.RIGHT).getModelInstance().transform);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            super.dispose();

            running = false;

            try
            {
               cameraReaderThread.join();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

            baseUI.dispose();
         }
      });
   }

   private class ValveIndexCameraReaderThread extends Thread
   {
      private final ConcurrentLinkedQueue<Mat> leftRects = new ConcurrentLinkedQueue<>();
      private final ConcurrentLinkedQueue<Mat> rightRects = new ConcurrentLinkedQueue<>();
      private int width;
      private int height;

      @Nullable
      public Queue<Mat> getRects(RobotSide side)
      {
         if (side == RobotSide.LEFT)
            return leftRects;
         else
            return rightRects;
      }

      public int getWidth()
      {
         return width;
      }

      public int getHeight()
      {
         return height;
      }

      @Override
      public void run()
      {
         VideoCapture videoCapture = new VideoCapture(0);

         width = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
         height = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

         System.out.println("Index camera resolution " + width + "x" + height);

         videoCapture.set(opencv_videoio.CAP_PROP_BUFFERSIZE, 2);

         while (running)
         {
            boolean readFrame = false;

            Mat bgrImage = new Mat(height, width, opencv_core.CV_8UC3);

            if (videoCapture.read(bgrImage))
            {
               readFrame = true;
            }
            else
            {
               System.out.println("Failed to read frame");
            }

            if (readFrame)
            {
               // Left
               Mat bgrLeftCameraRect = bgrImage.apply(new Rect(0, 0, width / 2, height));
               Mat rgbLeftCameraRect = new Mat();
               opencv_imgproc.cvtColor(bgrLeftCameraRect, rgbLeftCameraRect, opencv_imgproc.COLOR_BGR2RGBA);
               bgrLeftCameraRect.close();
               leftRects.add(rgbLeftCameraRect);

               // Right
               Mat bgrRightCameraRect = bgrImage.apply(new Rect(width / 2, 0, width - width / 2, height));
               Mat rgbRightCameraRect = new Mat();
               opencv_imgproc.cvtColor(bgrRightCameraRect, rgbRightCameraRect, opencv_imgproc.COLOR_BGR2RGBA);
               bgrRightCameraRect.close();
               rightRects.add(rgbRightCameraRect);
            }

            bgrImage.close();
         }

         System.out.println("Closing video capture device...");
         videoCapture.close();
         System.out.println("Closed video capture device");
      }
   }

   private void renderControls()
   {
      ImGuiTools.sliderDouble("Projection Z offset", projectionZOffset, -4, 4);

      for (RDXProjectionSphere projectionSphere : projectionSpheres)
      {
         projectionSphere.renderImGuiWidgets();
      }
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         if (eyeFrameGraphics.get(RobotSide.RIGHT) != null)
         {
            eyeFrameGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         }
      }
      else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT))
      {
         projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
         if (eyeFrameGraphics.get(RobotSide.LEFT) != null)
         {
            eyeFrameGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
         }
      }
      else
      {
         projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);

         if (baseUI.getVRManager().isVRReady())
         {
            for (RobotSide side : RobotSide.values)
            {
               if (eyeFrameGraphics.get(side) != null)
               {
                  eyeFrameGraphics.get(side).getRenderables(renderables, pool);
               }
               else
               {
                  RDXReferenceFrameGraphic eyeFrameGraphic = new RDXReferenceFrameGraphic(0.5);
                  eyeFrameGraphic.setToReferenceFrame(baseUI.getVRManager().getContext().getEyes().get(side).getEyeXForwardZUpFrame());
                  eyeFrameGraphics.put(side, eyeFrameGraphic);
               }
            }
         }
      }
   }

   public static void main(String[] args)
   {
      Mat mat = new Mat(); // Get opencv loaded before things start
      mat.close();

      new RDXValveIndexStereoPassthroughDemo();
   }
}
