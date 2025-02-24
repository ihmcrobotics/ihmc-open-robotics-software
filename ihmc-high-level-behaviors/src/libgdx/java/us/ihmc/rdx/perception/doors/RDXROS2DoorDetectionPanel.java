package us.ihmc.rdx.perception.doors;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.DetectedDoorMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public class RDXROS2DoorDetectionPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String PANEL_NAME = "Door Detection UI";

   // RDX
   private final Map<UUID, RDXDetectedDoor> rdxDetectedDoors = new HashMap<>();

   public RDXROS2DoorDetectionPanel(ROS2Node ros2Node)
   {
      super(PANEL_NAME);

      ros2Node.createSubscription2(PerceptionAPI.DETECTED_DOORS, detectedDoorListMessage ->
      {
         rdxDetectedDoors.keySet()
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
            }
            rdxDetectedDoor.update(detectedDoorMessage);
         }
      });
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (!sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         return;

      for (RDXDetectedDoor rdxDetectedDoor : rdxDetectedDoors.values())
         rdxDetectedDoor.getRenderables(renderables, pool, sceneLevels);
   }
}
