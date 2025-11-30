package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

/**
 * Demo class for ZED occupancy map.
 */
public class RDXZEDOccupancyMapDemo extends Lwjgl3ApplicationAdapter
{
   private static final String SVO_FILE = "/home/duncan/Downloads/20251110_162146_H1ZEDXMiniFirstMustardGrab.svo2";
   private static final RDXBaseUI baseUI = new RDXBaseUI();
   private final ZEDSVOPlaybackSensor zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private RDXPerceptionVisualizersPanel visualizers;
   private final SideDependentList<RDXImageVisualizer> zedPanels = new SideDependentList<>();
   private long lastZEDTimestamp = 1;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt requestedPosition = new ImInt();

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

      baseUI.getImGuiPanelManager().addPanel("Demo", this::renderImGuiWidgets);

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

      baseUI.renderBeforeOnScreenUI();
      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      visualizers.destroy();
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      baseUI.launchRDXApplication(new RDXZEDOccupancyMapDemo());
   }
}
