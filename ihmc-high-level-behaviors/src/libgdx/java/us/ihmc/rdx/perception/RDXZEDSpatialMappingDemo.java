package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

import java.util.List;

/**
 * Demo class for ZED spatial mapping.
 */
public class RDXZEDSpatialMappingDemo extends Lwjgl3ApplicationAdapter
{
   private static final String SVO_FILE = "/home/duncan/Downloads/20251020_ZEDXMini_DoorChargeBarrierBottle.svo2";
   private static final RDXBaseUI baseUI = new RDXBaseUI();
   private final ZEDSVOPlaybackSensor zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private RDXPerceptionVisualizersPanel visualizers;
   private final SideDependentList<RDXImageVisualizer> zedPanels = new SideDependentList<>();
   private long lastZEDTimestamp = 1;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt requestedPosition = new ImInt();

   private RDXPointCloudRenderer spatialMapRenderer;
   private static final int MAX_POINTS_TO_RENDER = 100000;

   @Override
   public void create()
   {
      baseUI.create();
      baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
      baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

      visualizers = new RDXPerceptionVisualizersPanel();
      for (RobotSide side : RobotSide.values)
      {
         RDXImageVisualizer zedPanel = new RDXImageVisualizer("%s ZED".formatted(side.getPascalCaseName()), "%s ZED".formatted(side.getPascalCaseName()), false);
         zedPanels.put(side, zedPanel);
         zedPanel.setActive(true);
         visualizers.addVisualizer(zedPanel);
      }
      visualizers.create(baseUI);

      // Create spatial map renderer from spatial mesh
      spatialMapRenderer = new RDXPointCloudRenderer();
      spatialMapRenderer.create(MAX_POINTS_TO_RENDER);
      spatialMapRenderer.setColoringMethod(RDXPointCloudRenderer.ColoringMethod.GRADIENT_WORLD_Z);
      baseUI.getPrimaryScene().addRenderableProvider(spatialMapRenderer, RDXSceneLevel.VIRTUAL);

      baseUI.getImGuiPanelManager().addPanel("Demo", this::renderImGuiWidgets);

      zedSensor.enablePositionalTracking(true);
      zedSensor.enableSpatialMapping(true);
      zedSensor.startSensor();
      zedSensor.getGrabThread().setFrequencyLimit(15.0);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Current SVO:");
      ImGui.sameLine();
      ImGui.textWrapped(zedSensor.getSVOFileName());

      ImGui.text("Current Position: %d".formatted(zedSensor.getCurrentPosition()));
      ImGui.text("Length: %d".formatted(zedSensor.getLength()));

      requestedPosition.set(zedSensor.getCurrentPosition());

      if (ImGui.button(labels.get("Play")))
         zedSensor.run(true);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Pause")))
         zedSensor.run(false);

      if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max(zedSensor.getLength(), 0)))
         zedSensor.setCurrentPosition(requestedPosition.get());
   }

   @Override
   public void render()
   {
      visualizers.update();

      long lastGrabTimestamp = zedSensor.getLastGrabTimestamp();
      if (lastGrabTimestamp > lastZEDTimestamp)
      {
         lastZEDTimestamp = lastGrabTimestamp;

         for (RobotSide side : RobotSide.values)
         {
            RawImage image = zedSensor.getImage(zedSensor.getImageKeys()[side.ordinal()]);
            zedPanels.get(side).setImage(image);
            image.release();
         }
      }
      updateOccupancyPointCloud();

      baseUI.renderBeforeOnScreenUI();
      baseUI.renderEnd();
   }

   private void updateOccupancyPointCloud()
   {
      if (spatialMapRenderer == null)
         return;

      float[] vertices = zedSensor.getMeshVertices();
      int numVertices = zedSensor.getMeshNumVertices();

      if (vertices == null || numVertices == 0)
      {
         spatialMapRenderer.updateMesh(List.of());
         return;
      }

      // CRITICAL: Enforce strict limit - never exceed MAX_POINTS_TO_RENDER
      int pointsToSample = Math.min(numVertices, MAX_POINTS_TO_RENDER);
      // Calculate stride to hit exactly MAX_POINTS_TO_RENDER points
      int stride = numVertices > MAX_POINTS_TO_RENDER ?
            (numVertices + MAX_POINTS_TO_RENDER - 1) / MAX_POINTS_TO_RENDER : 1;
      // Pre-allocate exactly the number we'll use
      Point3D[] sampledPoints = new Point3D[pointsToSample];

      int idx = 0;
      for (int i = 0; i < numVertices && idx < MAX_POINTS_TO_RENDER; i += stride)
      {
         int base = 3 * i;
         if (base + 2 >= vertices.length)
            break;

         sampledPoints[idx++] = new Point3D(vertices[base], vertices[base + 1], vertices[base + 2]);
      }

      // Only send what we actually filled
      if (idx < sampledPoints.length)
      {
         Point3D[] trimmed = new Point3D[idx];
         System.arraycopy(sampledPoints, 0, trimmed, 0, idx);
         sampledPoints = trimmed;
      }

      spatialMapRenderer.updateMesh(sampledPoints);
   }

   @Override
   public void dispose()
   {
      visualizers.destroy();
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      baseUI.launchRDXApplication(new RDXZEDSpatialMappingDemo());
   }
}
