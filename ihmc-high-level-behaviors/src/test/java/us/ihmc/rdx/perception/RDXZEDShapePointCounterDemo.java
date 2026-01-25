package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

public class RDXZEDShapePointCounterDemo
{
   private static final String SVO_FILE = System.getProperty("user.home") + "/Downloads/20260121_120734_AlexDoorData.svo2";

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ZEDSVOPlaybackSensor zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private RDXPose3DGizmo spherePoseGizmo;
   private ModelInstance sphereModel;
   private final ImBoolean play = new ImBoolean(false);
   private final ImInt requestedPosition = new ImInt();
   private final ImInt currentPosition = new ImInt();
   private final ImInt zedLength = new ImInt();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXZEDShapePointCounterDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            zedSensor.startSensor();
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            sphereModel = RDXModelBuilder.createSphere(0.5f, new Color(0.45f, 0.75f, 1.0f, 1.0f));
            LibGDXTools.setOpacity(sphereModel, 0.5f);
            baseUI.getPrimaryScene().addModelInstance(sphereModel);
            spherePoseGizmo = new RDXPose3DGizmo();
            spherePoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getImGuiPanelManager().addPanel("Shape Point Counter", RDXZEDShapePointCounterDemo.this::renderImGuiWidgets);
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            LibGDXTools.toLibGDX(spherePoseGizmo.getTransformToParent(), sphereModel.transform);
            pointCloudVisualizer.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            zedSensor.close();
            pointCloudVisualizer.destroy();
            spherePoseGizmo.destroyDefault(baseUI.getPrimary3DPanel());
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      pointCloudVisualizer.renderImGuiWidgets();
      ImGui.separator();

      currentPosition.set(zedSensor.getCurrentPosition());
      zedLength.set(zedSensor.getLength());

      ImGui.text("SVO: " + zedSensor.getSVOFileName());
      if (ImGui.checkbox(labels.get("ZED Playback"), play))
      {
         if (play.get())
            zedSensor.play();
         else
            zedSensor.pause();
      }

      ImGui.beginDisabled(play.get());
      if (play.get())
         requestedPosition.set(currentPosition.get());
      if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max(zedLength.get(), 0)))
      {
         zedSensor.setCurrentPosition(requestedPosition.get());
         zedSensor.grabAndNotify();
      }
      ImGui.endDisabled();

      ImGui.text("Frame: " + currentPosition.get() + " / " + Math.max(zedLength.get(), 0));
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedSensor.waitForGrab();

      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      if (colorImage != null)
      {
         pointCloudVisualizer.setColorImage(colorImage);
         colorImage.release();
      }
      if (depthImage != null)
      {
         pointCloudVisualizer.setDepthImage(depthImage);
         depthImage.release();
      }
   }

   public static void main(String[] args)
   {
      new RDXZEDShapePointCounterDemo();
   }
}
