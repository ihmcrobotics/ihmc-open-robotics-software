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
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public class RDXROS2DoorDetectionPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String PANEL_NAME = "Door Detection UI";
   private static final int DETECTED_DOORS_TABLE_COLUMNS = 7;

   private final Map<UUID, RDXDetectedDoor> rdxDetectedDoors = new LinkedHashMap<>();
   private final Map<UUID, ImBoolean> renderIndividualDoors = new HashMap<>();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean renderAllStableDetectedDoors = new ImBoolean(false);

   public RDXROS2DoorDetectionPanel(ROS2Node ros2Node)
   {
      super(PANEL_NAME);
      setRenderMethod(this::renderImGuiWidgets);

      ros2Node.createSubscription2(PerceptionAPI.DETECTED_DOORS, detectedDoorListMessage ->
      {
         rdxDetectedDoors.keySet()
                         .removeIf(rdxUUID -> detectedDoorListMessage.getDetectedDoors()
                                                                     .stream()
                                                                     .map(detectionMessage -> MessageTools.toUUID(detectionMessage.getDetectionUuid()))
                                                                     .noneMatch(uuid -> uuid.equals(rdxUUID)));
         renderIndividualDoors.keySet()
                              .removeIf(rdxUUID -> detectedDoorListMessage.getDetectedDoors()
                                                                          .stream()
                                                                          .map(detectionMessage -> MessageTools.toUUID(detectionMessage.getDetectionUuid()))
                                                                          .noneMatch(uuid -> uuid.equals(rdxUUID)));

         for (int i = 0; i < detectedDoorListMessage.getDetectedDoors().size(); ++i)
         {
            DetectedDoorMessage detectedDoorMessage = detectedDoorListMessage.getDetectedDoors().get(i);
            UUID doorDetectionUUID = MessageTools.toUUID(detectedDoorMessage.getDetectionUuid());
            RDXDetectedDoor rdxDetectedDoor = rdxDetectedDoors.get(doorDetectionUUID);
            if (rdxDetectedDoor == null)
            {
               rdxDetectedDoor = new RDXDetectedDoor();
               rdxDetectedDoors.put(doorDetectionUUID, rdxDetectedDoor);
               renderIndividualDoors.put(doorDetectionUUID, new ImBoolean(renderAllStableDetectedDoors.get()));
            }
            rdxDetectedDoor.update(detectedDoorMessage);
         }
      });
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get("Render All Stable Detected Doors"), renderAllStableDetectedDoors))
      {
         renderIndividualDoors.forEach((detectionUUID, render) ->
         {
            if (rdxDetectedDoors.get(detectionUUID).getDetection().isDetectionStable())
               render.set(renderAllStableDetectedDoors);
         });
      }

      ImGui.separator();

      // Stable detected doors
      Set<RDXDetectedDoor> stableDetectedDoors = new LinkedHashSet<>(rdxDetectedDoors.values());
      stableDetectedDoors.removeIf(detectedDoor -> !detectedDoor.getDetection().isDetectionStable());

      // Unstable detected doors
      Set<RDXDetectedDoor> unstableDetectedDoors = new LinkedHashSet<>(rdxDetectedDoors.values());
      unstableDetectedDoors.removeIf(detectedDoor -> detectedDoor.getDetection().isDetectionStable());

      // Detected door counts
      ImGui.text("Detected Door Count: " + rdxDetectedDoors.size());
      ImGui.text("Stable Detected Door Count: " + stableDetectedDoors.size());
      ImGui.text("Unstable Detected Door Count: " + unstableDetectedDoors.size());

      // Tables of detected doors
      renderDetectedDoorsTable("Stable Detected Doors", stableDetectedDoors);
      renderDetectedDoorsTable("Unstable Detected Doors", unstableDetectedDoors);
   }

   private void renderDetectedDoorsTable(String name, Set<RDXDetectedDoor> doorsToRender)
   {
      ImGui.separator();
      ImGui.text(name);

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
            ImBoolean render = renderIndividualDoors.get(detectedDoorID);

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
         if (renderIndividualDoors.get(detectedDoorID).get())
            rdxDetectedDoors.get(detectedDoorID).getRenderables(renderables, pool, sceneLevels);
      }
   }
}
