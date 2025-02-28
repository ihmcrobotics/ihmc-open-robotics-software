package us.ihmc.rdx.perception.doors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.perception.detections.doors.DetectedDoor;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;

import java.time.Instant;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.stream.Collectors;

public class RDXROS2DoorDetectionPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String PANEL_NAME = "Door Detection UI";
   private static final int DETECTED_DOORS_TABLE_COLUMNS = 7;

   private final Map<UUID, RDXDetectedDoor> rdxDetectedDoors = new LinkedHashMap<>();
   private final Set<RDXDetectedDoor> stableDetections = new LinkedHashSet<>();
   private final Set<RDXDetectedDoor> unstableDetections = new LinkedHashSet<>();
   private final Map<UUID, ImBoolean> detectionsToRender = new HashMap<>();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean renderStableDetections = new ImBoolean(false);

   public RDXROS2DoorDetectionPanel(ROS2Node ros2Node)
   {
      super(PANEL_NAME);
      setRenderMethod(this::renderImGuiWidgets);

      ros2Node.createSubscription2(PerceptionAPI.DETECTED_DOORS, detectedDoorListMessage ->
      {
         // Remove detections that disappeared
         Set<UUID> currentlyDetectedDoorUUIDs = detectedDoorListMessage.getDetectedDoors()
                                                                       .stream()
                                                                       .map(detectedDoorMessage -> MessageTools.toUUID(detectedDoorMessage.getDetectionUuid()))
                                                                       .collect(Collectors.toSet());
         Iterator<UUID> uuidIterator = rdxDetectedDoors.keySet().iterator();
         while (uuidIterator.hasNext())
         {
            UUID uuid = uuidIterator.next();
            if (!currentlyDetectedDoorUUIDs.contains(uuid))
            {
               RDXDetectedDoor detectedDoor = rdxDetectedDoors.get(uuid);
               stableDetections.remove(detectedDoor);
               unstableDetections.remove(detectedDoor);
               detectionsToRender.remove(uuid);
               detectedDoor.dispose();
               uuidIterator.remove();
            }
         }

         // Update existing detections
         for (DetectedDoorMessage detectedDoorMessage : detectedDoorListMessage.getDetectedDoors())
         {
            UUID detectionUUID = MessageTools.toUUID(detectedDoorMessage.getDetectionUuid());
            RDXDetectedDoor rdxDetectedDoor = rdxDetectedDoors.get(detectionUUID);
            if (rdxDetectedDoor == null) // Add new RDXDetectedDoors if new detections show up
            {
               rdxDetectedDoor = new RDXDetectedDoor();
               rdxDetectedDoors.put(detectionUUID, rdxDetectedDoor);
               detectionsToRender.put(detectionUUID, new ImBoolean(false));
            }
            rdxDetectedDoor.update(detectedDoorMessage); // Update the RDXDetectedDoor

            // Ensure each door is in the correct stable/unstable set
            if (rdxDetectedDoor.getDetection().isDetectionStable())
            {
               if (!stableDetections.contains(rdxDetectedDoor))
               {
                  detectionsToRender.get(detectionUUID).set(renderStableDetections);
                  stableDetections.add(rdxDetectedDoor);
                  unstableDetections.remove(rdxDetectedDoor);
               }
            }
            else
            {
               if (!unstableDetections.contains(rdxDetectedDoor))
               {
                  detectionsToRender.get(detectionUUID).set(false);
                  unstableDetections.add(rdxDetectedDoor);
                  stableDetections.remove(rdxDetectedDoor);
               }
            }

         }
      });
   }

   private void renderImGuiWidgets()
   {
      // Detected door counts
      ImGui.text("Detected Door Count: " + rdxDetectedDoors.size());
      ImGui.text("Stable Detected Door Count: " + stableDetections.size());
      ImGui.text("Unstable Detected Door Count: " + unstableDetections.size());

      // Tables of detected doors
      ImGui.separator();
      ImGui.text("Stable Detected Doors");
      if (ImGui.checkbox(labels.get("Render Stable Detections"), renderStableDetections))
      {
         detectionsToRender.forEach((uuid, render) ->
         {
            if (stableDetections.contains(rdxDetectedDoors.get(uuid)))
               render.set(renderStableDetections);
         });
      }
      renderDetectedDoorsTable("Stable Detected Doors", stableDetections);

      ImGui.separator();
      ImGui.text("Unstable Detected Doors");
      renderDetectedDoorsTable("Unstable Detected Doors", unstableDetections);
   }

   private void renderDetectedDoorsTable(String name, Set<RDXDetectedDoor> doorsToRender)
   {
      int tableFlags = ImGuiTableFlags.ScrollY;
      tableFlags |= ImGuiTableFlags.Borders;
      tableFlags |= ImGuiTableFlags.NoKeepColumnsVisible;
      tableFlags |= ImGuiTableFlags.Hideable;
      tableFlags |= ImGuiTableFlags.Sortable;
      if(ImGui.beginTable(labels.get(name), DETECTED_DOORS_TABLE_COLUMNS, tableFlags, 0.0f, 200.0f))
      {
         float checkboxWidth = ImGui.getFrameHeight();
         ImGui.tableSetupColumn(labels.get("Render##" + name), ImGuiTableColumnFlags.WidthFixed, checkboxWidth);
         ImGui.tableSetupColumn(labels.get("Opening Mechanism##" + name));
         ImGui.tableSetupColumn(labels.get("Door Side##" + name));
         ImGui.tableSetupColumn(labels.get("Time Since Detection##" + name));
         ImGui.tableSetupColumn(labels.get("Opening Mechanism Pose Known##" + name));
         ImGui.tableSetupColumn(labels.get("Panel Pose Known##" + name), ImGuiTableColumnFlags.DefaultHide);
         ImGui.tableSetupColumn(labels.get("Detection UUID##" + name), ImGuiTableColumnFlags.DefaultHide);

         ImGui.tableHeadersRow();

         for (RDXDetectedDoor detectedDoor : doorsToRender)
         {
            DetectedDoor detection = detectedDoor.getDetection();
            UUID detectedDoorID = detection.getDetectionUUID();
            ImBoolean render = detectionsToRender.get(detectedDoorID);

            ImGui.tableNextRow();

            if (ImGui.tableNextColumn()) // Render
            {
               ImGui.checkbox(labels.getHidden("render" + detectedDoorID), render);
            }

            if (ImGui.tableNextColumn()) // Opening Mechanism
            {
               ImGui.text(detection.getOpeningMechanism().getName());
            }

            if (ImGui.tableNextColumn()) // Door Side
            {
               ImGui.text(detection.getOpeningMechanism().getDoorSide().toString());
            }

            if (ImGui.tableNextColumn()) // Time Since Detection
            {
               ImGui.text(String.format("%04d", Instant.now().minusMillis(detection.getLastDetectedTime().toEpochMilli()).toEpochMilli()));
            }

            if (ImGui.tableNextColumn()) // Opening Mechanism Pose Known
            {
               boolean isKnown = detection.getOpeningMechanism().isPoseKnown();
               String text = isKnown ? "Known" : "Unknown";
               int color = isKnown ? Color.GREEN.toIntBits() : Color.RED.toIntBits();
               ImGui.textColored(color, "Pose " + text);
            }

            if (ImGui.tableNextColumn()) // Panel Pose Known
            {
               boolean isKnown = detection.hasPanelPose();
               String text = isKnown ? "Known" : "Unknown";
               int color = isKnown ? Color.GREEN.toIntBits() : Color.RED.toIntBits();
               ImGui.textColored(color, "Pose " + text);
            }

            if (ImGui.tableNextColumn())
            {
               ImGui.text(detectedDoorID.toString());
            }
         }

         ImGui.endTable();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         return;

      for (UUID detectedDoorID : rdxDetectedDoors.keySet())
      {
         if (detectionsToRender.get(detectedDoorID).get())
            rdxDetectedDoors.get(detectedDoorID).getRenderables(renderables, pool, sceneLevels);
      }
   }
}
