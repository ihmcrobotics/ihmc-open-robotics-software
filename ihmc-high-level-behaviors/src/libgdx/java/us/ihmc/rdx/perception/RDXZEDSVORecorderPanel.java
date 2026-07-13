package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import perception_msgs.ZEDSVOCurrentFileMessage;
import std_msgs.Int64;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublisherMap;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXZEDSVORecorderPanel
{
   public enum RecordMode
   {
      RECORD((byte) 0), PLAYBACK((byte) 1);

      private final byte byteValue;

      RecordMode(byte byteValue)
      {
         this.byteValue = byteValue;
      }

      public byte toByte()
      {
         return byteValue;
      }

      public static RecordMode fromByte(byte b)
      {
         for (RecordMode value : values())
         {
            if (value.byteValue == b)
               return value;
         }
         return null;
      }
   }

   private static final String PANEL_NAME = "ZED SVO Recorder";

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ROS2Node ros2Node;
   private final ROS2PublisherMap publisherMap;
   private ZEDSVOCurrentFileMessage latestMessage;

   private final ImInt requestedPosition = new ImInt();
   private boolean holdingOnToTheSlider;
   private boolean paused;

   private final Throttler requestThrottler = new Throttler().setFrequency(5.0);

   public RDXZEDSVORecorderPanel(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.publisherMap = new ROS2PublisherMap(ros2Node);
      ros2Node.createSubscriptionSampler(PerceptionAPI.ZED_SVO_CURRENT_FILE, sample ->
      {
         ZEDSVOCurrentFileMessage copy = new ZEDSVOCurrentFileMessage();
         copy.set(sample);
         latestMessage = copy;
      });
   }

   public void update()
   {
      RDXBaseUI baseUI = RDXBaseUI.getInstance();

      boolean overlayPanelExists = baseUI.getPrimary3DPanel().overlayPanelExists(PANEL_NAME);

      if (!overlayPanelExists && latestMessage != null)
      {
         baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::render);
      }
   }

   public void render()
   {
      // Because of threading, it's possible that we haven't received any message so we can't render anything yet
      if (latestMessage == null)
         return;

      ImGuiTools.textBold("Current SVO:");
      ImGui.sameLine();
      ImGui.textWrapped(latestMessage.getCurrentFileName().toString());

      RecordMode recordMode = RecordMode.fromByte(latestMessage.getRecordMode());

      if (recordMode == RecordMode.PLAYBACK)
      {
         if (!holdingOnToTheSlider)
            requestedPosition.set((int) latestMessage.getCurrentPosition());

         if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max((int) latestMessage.getLength(), 0)))
         {
            holdingOnToTheSlider = true;

            if (requestThrottler.run())
            {
               publishPositionRequest();
            }
         }
         // Called once you let go of the slider
         if (ImGui.isItemDeactivatedAfterEdit())
         {
            holdingOnToTheSlider = false;

            publishPositionRequest();
         }

         ImGui.sameLine();

         if (ImGui.button(labels.get(paused ? "Play" : "Pause")))
         {
            publisherMap.publish(paused ? PerceptionAPI.ZED_SVO_PLAY : PerceptionAPI.ZED_SVO_PAUSE);
            paused = !paused;
         }

         ImGui.beginDisabled(!paused);
         if (ImGui.button(labels.get("Previous Frame")))
         {
            requestedPosition.set((int) latestMessage.getCurrentPosition() - 1);
            publishPositionRequest();
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Next Frame")))
         {
            requestedPosition.set((int) latestMessage.getCurrentPosition() + 1);
            publishPositionRequest();
         }
         ImGui.endDisabled();
      }
   }

   private void publishPositionRequest()
   {
      Int64 positionMessage = new Int64();
      positionMessage.setData(requestedPosition.get());
      publisherMap.publish(PerceptionAPI.ZED_SVO_SET_POSITION, positionMessage);
   }
}
