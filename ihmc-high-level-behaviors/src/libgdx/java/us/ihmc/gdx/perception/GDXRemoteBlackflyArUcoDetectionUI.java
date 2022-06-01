package us.ihmc.gdx.perception;

import std_msgs.msg.dds.Float64;
import us.ihmc.avatar.colorVision.DualBlackflyComms;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.tools.ImPlotDoublePlot;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXRemoteBlackflyArUcoDetectionUI
{
   private final ImGuiPanel panel = new ImGuiPanel("ArUco Marker Detection", this::renderImGuiWidgets);
   private final SideDependentList<IHMCROS2Input<Float64>> publishRateInputs = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlot> publishRatePlots = new SideDependentList<>();
   private final IHMCROS2Input<Float64> getImageDurationInput;
   private final ImPlotDoublePlot getImageDurationPlot;
   private final IHMCROS2Input<Float64> convertColorDurationInput;
   private final ImPlotDoublePlot convertColorDurationPlot;
   private final IHMCROS2Input<Float64> encodingDurationInput;
   private final ImPlotDoublePlot encodingDurationPlot;
   private final IHMCROS2Input<Float64> copyDurationInput;
   private final ImPlotDoublePlot copyDurationPlot;

   public GDXRemoteBlackflyArUcoDetectionUI(ROS2Helper ros2Helper)
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRateInputs.put(side, ros2Helper.subscribe(DualBlackflyComms.PUBLISH_RATE.get(side)));
         publishRatePlots.put(side, new ImPlotDoublePlot(side.getPascalCaseName() + " Publish Rate", 30));
      }
      getImageDurationInput = ros2Helper.subscribe(DualBlackflyComms.GET_IMAGE_DURATION);
      getImageDurationPlot = new ImPlotDoublePlot("Get image duration", 30);
      convertColorDurationInput = ros2Helper.subscribe(DualBlackflyComms.CONVERT_COLOR_DURATION);
      convertColorDurationPlot = new ImPlotDoublePlot("Convert color duration", 30);
      encodingDurationInput = ros2Helper.subscribe(DualBlackflyComms.ENCODING_DURATION);
      encodingDurationPlot = new ImPlotDoublePlot("Encoding duration", 30);
      copyDurationInput = ros2Helper.subscribe(DualBlackflyComms.COPY_DURATION);
      copyDurationPlot = new ImPlotDoublePlot("Copy duration", 30);
   }

   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         if (publishRateInputs.get(side).hasReceivedFirstMessage())
         {
            publishRatePlots.get(side).addValue(publishRateInputs.get(side).getLatest().getData());
         }
         getImageDurationPlot.addValue(getImageDurationInput.getLatest().getData());
         convertColorDurationPlot.addValue(convertColorDurationInput.getLatest().getData());
         encodingDurationPlot.addValue(encodingDurationInput.getLatest().getData());
         copyDurationPlot.addValue(copyDurationInput.getLatest().getData());
      }
   }

   public void renderImGuiWidgets()
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRatePlots.get(side).renderImGuiWidgets();
      }
      getImageDurationPlot.renderImGuiWidgets();
      convertColorDurationPlot.renderImGuiWidgets();
      encodingDurationPlot.renderImGuiWidgets();
      copyDurationPlot.renderImGuiWidgets();
   }

   public void destroy()
   {
      for (RobotSide side : RobotSide.values)
      {
         publishRateInputs.get(side).destroy();
      }
      getImageDurationInput.destroy();
      convertColorDurationInput.destroy();
      encodingDurationInput.destroy();
      copyDurationInput.destroy();
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
