package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import java.util.concurrent.atomic.AtomicInteger;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounter;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
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
   private static final String SVO_FILE = "/opt/ihmc/LogData/UserFolders/TomaszFolder/heightmap_test.svo2";

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ZEDSVOPlaybackSensor zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private RDXPose3DGizmo spherePoseGizmo;
   private ModelInstance sphereModel;
   private final CUDAShapePointCounter shapePointCounter = new CUDAShapePointCounter();
   private final Point3D32 sphereCenter = new Point3D32();
   private final AtomicInteger pointsInSphere = new AtomicInteger();
   private final ImGuiMovingPlot pointsPlot = new ImGuiMovingPlot("Points in Sphere", 1000, 300, 200);
   private final Color sphereColor = new Color(0.0f, 0.0f, 1.0f, 0.5f);
   private final ImBoolean play = new ImBoolean(false);
   private final ImInt requestedPosition = new ImInt();
   private final ImInt currentPosition = new ImInt();
   private final ImInt zedLength = new ImInt();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RawImage latestDepth = null;

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
            updateSphere(pointsInSphere.get());
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
            shapePointCounter.close();
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      pointCloudVisualizer.renderImGuiWidgets();
      ImGui.separator();
      ImGui.pushFont(ImGuiTools.getBigFont());
      ImGui.text("Points in sphere: " + pointsInSphere.get());
      ImGui.popFont();
      pointsPlot.calculate(pointsInSphere.get());
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

   private void updateSphere(int count)
   {
      sphereCenter.set((float) spherePoseGizmo.getTransformToParent().getM03(),
                       (float) spherePoseGizmo.getTransformToParent().getM13(),
                       (float) spherePoseGizmo.getTransformToParent().getM23());

      if (latestDepth != null)
         pointsInSphere.set(shapePointCounter.countPointsInSphere(latestDepth, sphereCenter, 0.5f));

      float t = Math.min(Math.max(count / 20000.0f, 0.0f), 1.0f);
      sphereColor.set(t, 0.0f, 1.0f - t, 0.5f);
      LibGDXTools.setDiffuseColor(sphereModel, sphereColor);
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
         if (latestDepth != null)
            latestDepth.release();
         latestDepth = depthImage.get();

         pointCloudVisualizer.setDepthImage(depthImage);
         depthImage.release();
      }
   }

   public static void main(String[] args)
   {
      new RDXZEDShapePointCounterDemo();
   }
}
