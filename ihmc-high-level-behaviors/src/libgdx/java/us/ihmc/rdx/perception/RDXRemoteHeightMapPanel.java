package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.HeightMapStateRequestMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.perception.heightMap.HeightMapAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.ui.graphics.RDXGridMapGraphic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.atomic.AtomicReference;

public class RDXRemoteHeightMapPanel
{
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private final HeightMapFilterParameters heightMapFilterParameters = new HeightMapFilterParameters();
   private final RDXPanel panel = new RDXPanel("CPU Height Map", this::renderImGuiWidgets);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   private final ROS2Helper ros2Helper;
   private final HeightMapStateRequestMessage pauseMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage resumeMessage = new HeightMapStateRequestMessage();
   private final HeightMapStateRequestMessage clearMessage = new HeightMapStateRequestMessage();
   private final AtomicReference<HeightMapMessage> latestMessage = new AtomicReference<>(null);

   private RDXMatImagePanel heightMapPanel;

   public RDXRemoteHeightMapPanel(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      remotePropertySets.registerRemotePropertySet(heightMapParameters, HeightMapAPI.PARAMETERS);
      remotePropertySets.registerRemotePropertySet(heightMapFilterParameters, HeightMapAPI.FILTER_PARAMETERS);

      pauseMessage.setRequestPause(true);
      resumeMessage.setRequestResume(true);
      clearMessage.setRequestClear(true);
   }

   public void create()
   {
      heightMapPanel = new RDXMatImagePanel("Height Map Image", 50, 50, false);
      heightMapPanel.resize(50, 50);
      panel.addChild(heightMapPanel.getImagePanel());
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      latestMessage.set(heightMapMessage);
   }

   private void drawHeightMapPanel(HeightMapMessage heightMapMessage)
   {
      if (heightMapMessage == null || !heightMapPanel.getImagePanel().getIsShowing().get())
         return;

      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      int size = heightMapData.getCellsPerAxis();
      if (heightMapPanel.getImage().arrayHeight() != size)
      {
         heightMapPanel.resize(size, size);
      }

      for (int x = 0; x < size; x++)
      {
         for (int y = 0; y < size; y++)
         {
            int row = size - x - 1;
            int col = size - y - 1;
            Color color = RDXGridMapGraphic.computeColorFromHeight(heightMapData.getHeightAt(x, y));
            BytePointer pixel = heightMapPanel.getImage().ptr(row, col);
            int r = (int) (color.r * 255);
            int g = (int) (color.g * 255);
            int b = (int) (color.b * 255);
            pixel.put(0, (byte) r);
            pixel.put(1, (byte) g);
            pixel.put(2, (byte) b);
         }
      }
      heightMapPanel.display();
   }

   public void update()
   {
      drawHeightMapPanel(latestMessage.getAndSet(null));
   }

   public void renderImGuiWidgets()
   {
      remotePropertySets.renderImGuiWidgets();

      if (ImGui.button("Pause"))
         ros2Helper.publish(PerceptionAPI.HEIGHT_MAP_STATE_REQUEST, pauseMessage);
      if (ImGui.button("Resume"))
         ros2Helper.publish(PerceptionAPI.HEIGHT_MAP_STATE_REQUEST, resumeMessage);
      if (ImGui.button("Clear"))
         ros2Helper.publish(PerceptionAPI.HEIGHT_MAP_STATE_REQUEST, clearMessage);
   }

   public void destroy()
   {
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
