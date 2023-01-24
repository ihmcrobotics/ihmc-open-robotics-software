package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.flag.ImGuiButtonFlags;
import imgui.flag.ImGuiMouseButton;
import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ihmcPerception.heightMap.HeightMapAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImGuiVideoPanel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.visualizers.RDXGridMapGraphic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.atomic.AtomicReference;

public class RDXRemoteHeightMapPanel
{
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private final HeightMapFilterParameters heightMapFilterParameters = new HeightMapFilterParameters();
   private final ImGuiPanel panel = new ImGuiPanel("CPU Height Map", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   private final ROS2Helper ros2Helper;
   private final HeightMapStateRequestMessage pauseMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage resumeMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage clearMessage = new HeightMapStateRequestMessage();
   private final AtomicReference<HeightMapMessage> latestMessage = new AtomicReference<>(null);

   private final OpenCLManager openCLManager;
   private RDXCVImagePanel heightMapPanel;

   public RDXRemoteHeightMapPanel(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(heightMapParameters, HeightMapAPI.PARAMETERS);
      remotePropertySets.registerRemotePropertySet(heightMapFilterParameters, HeightMapAPI.FILTER_PARAMETERS);

      openCLManager = new OpenCLManager();


      pauseMessage.setRequestPause(true);
      resumeMessage.setRequestResume(true);
      clearMessage.setRequestClear(true);
   }

   public void create()
   {
      openCLManager.create();
      heightMapPanel = new RDXCVImagePanel("Height Map Image", 50, 50);
      heightMapPanel.resize(50, 50, openCLManager);
      panel.addChild(heightMapPanel.getVideoPanel());
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      latestMessage.set(heightMapMessage);
   }

   private void drawHeightMapPanel(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null || !heightMapPanel.getVideoPanel().getIsShowing().get())
         return;

      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      int size = heightMapData.getCellsPerAxis();
      if (heightMapPanel.getBytedecoImage().getImageHeight() != size)
      {
         heightMapPanel.resize(size, size, openCLManager);
      }

      for (int x = 0; x < size; x++)
      {
         for (int y = 0; y < size; y++)
         {
            int row = size - x - 1;
            int col = size - y - 1;
            Color color = RDXGridMapGraphic.computeColorFromHeight(heightMapData.getHeightAt(x, y));
            BytePointer pixel = heightMapPanel.getBytedecoImage().getBytedecoOpenCVMat().ptr(row, col);
            int r = (int) (color.r * 255);
            int g = (int) (color.g * 255);
            int b = (int) (color.b * 255);
            pixel.put(0, (byte) r);
            pixel.put(1, (byte) g);
            pixel.put(2, (byte) b);
         }
      }
      //      }
      heightMapPanel.draw();
   }

   public void update()
   {
      drawHeightMapPanel(latestMessage.getAndSet(null));
   }

   public void renderImGuiWidgets()
   {
      remotePropertySets.renderImGuiWidgets();

      if (ImGui.button("Pause"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, pauseMessage);
      if (ImGui.button("Resume"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, resumeMessage);
      if (ImGui.button("Clear"))
         ros2Helper.publish(ROS2Tools.HEIGHT_MAP_STATE_REQUEST, clearMessage);
   }

   public void destroy()
   {
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }
}
