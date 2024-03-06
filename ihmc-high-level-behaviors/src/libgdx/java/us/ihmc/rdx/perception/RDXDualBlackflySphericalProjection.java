package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPReceiver;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXDualBlackflySphericalProjection
{
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   // As you approach infinite distance, pupillary distance needs to be 1
   // Close up things look good with a pupillary distance of ~2.0
   private final ImDouble pupillaryDistance = new ImDouble(1.666667);
   private final RigidBodyTransform leftEyePose = new RigidBodyTransform();
   private final RigidBodyTransform rightEyePose = new RigidBodyTransform();
   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private volatile boolean reconnecting = false;
   private Thread reconnectThread;

   public RDXDualBlackflySphericalProjection(RDXBaseUI baseUI)
   {
      baseUI.getImGuiPanelManager().addPanel("Projection", () ->
      {
         if (ImGuiTools.sliderDouble(labels.get("Pupillary distance"), pupillaryDistance, -15, 15))
         {

         }
         ImGui.separator();
         ImGuiTools.textBold("Left:");
         ImGui.indent();
         projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
         ImGui.unindent();
         ImGui.separator();
         ImGuiTools.textBold("Right:");
         ImGui.indent();
         projectionSpheres.get(RobotSide.RIGHT).renderImGuiWidgets();
         ImGui.unindent();
         ImGui.separator();
      });
   }

   private void startReconnectThread()
   {
      stopReconnectThread();

      reconnecting = true;

      reconnectThread = new Thread(() -> {
         do {
            RDXBaseUI.pushNotification("Dual Blackfly stereo client reconnecting...");

            dualBlackflyUDPReceiver.stop();
            dualBlackflyUDPReceiver.start();

            ThreadTools.sleep(5000);
         } while (reconnecting && !dualBlackflyUDPReceiver.connected());
      }, getClass().getName() + "-ReconnectThread");

      reconnectThread.start();
   }

   private void stopReconnectThread()
   {
      if (reconnectThread != null)
      {
         reconnecting = false;

         try
         {
            reconnectThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }
   }

   public void create()
   {
      startReconnectThread();

      projectionSpheres.get(RobotSide.LEFT).create();
      projectionSpheres.get(RobotSide.RIGHT).create();
   }

   public void shutdown()
   {
      stopReconnectThread();

      dualBlackflyUDPReceiver.stop();
   }

   public void render()
   {
      for (RobotSide side : RobotSide.values)
      {
         byte[] imageData = dualBlackflyUDPReceiver.getImageBuffers().get(side);

         if (imageData != null)
         {
            ImageDimensions imageDimensions = dualBlackflyUDPReceiver.getImageDimensions().get(side);
            BytePointer imageDataPointer = new BytePointer(imageData);

            Mat mat = new Mat(imageDimensions.getImageHeight(), imageDimensions.getImageWidth(), opencv_core.CV_8UC1);
            mat.data(imageDataPointer);

            Pixmap pixmap = new Pixmap(mat.cols(), mat.rows(), Pixmap.Format.RGBA8888);
            BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
            Mat rgba8Mat = new Mat(mat.rows(), mat.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
            opencv_imgproc.cvtColor(mat, rgba8Mat, opencv_imgproc.COLOR_BayerBG2RGBA);
            Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

            projectionSpheres.get(side).updateTexture(texture);

            rgba8Mat.close();
            pixmap.dispose();
            mat.close();
            imageDataPointer.close();
         }
      }

      leftEyePose.getTranslation().setY(pupillaryDistance.get() / 2);
      rightEyePose.getTranslation().setY(-pupillaryDistance.get() / 2);
      LibGDXTools.toLibGDX(leftEyePose, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);
      LibGDXTools.toLibGDX(rightEyePose, projectionSpheres.get(RobotSide.RIGHT).getModelInstance().transform);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         // Only show the renderables if there is an image frame in the buffer
         if (dualBlackflyUDPReceiver.getImageBuffers().get(RobotSide.RIGHT) != null)
         {
            projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         }
      }
      else
      {
         // Only show the renderables if there is an image frame in the buffer
         if (dualBlackflyUDPReceiver.getImageBuffers().get(RobotSide.LEFT) != null)
         {
            projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
         }
      }
   }
}
