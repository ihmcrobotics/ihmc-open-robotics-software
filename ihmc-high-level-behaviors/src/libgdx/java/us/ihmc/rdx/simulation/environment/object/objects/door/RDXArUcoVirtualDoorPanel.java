package us.ihmc.rdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.PerceptionManager;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXArUcoVirtualDoorPanel extends RDXVirtualGhostObject
{

   private final OpenCVArUcoMarker arUcoMarker;
   private final RigidBodyTransform markerToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerToWorld);
   private final RigidBodyTransform transformToMarker = new RigidBodyTransform();
   private final ReferenceFrame virtualFrame;

   public RDXArUcoVirtualDoorPanel(int id, String name)
   {
      super("environmentObjects/door/doorPanel/DoorPanel.g3dj");

      arUcoMarker = new OpenCVArUcoMarker(id, 0.2032);

      if (id == PerceptionManager.PULL_DOOR_MARKER_ID)
      {
         transformToMarker.set(PerceptionManager.PULL_DOOR_PANEL_TRANSFORM_TO_MARKER);
      }
      else if (id == PerceptionManager.PUSH_DOOR_MARKER_ID)
      {
         transformToMarker.set(PerceptionManager.PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER);
      }

      virtualFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(String.format("%s%dPanel", name, id), markerFrame, transformToMarker);
   }

   @Override
   public void update()
   {
      markerFrame.update();

      virtualFrame.getTransformToDesiredFrame(getTransformToParent(), ReferenceFrame.getWorldFrame());

      super.update();
   }

   public RigidBodyTransform getMarkerToWorld()
   {
      return markerToWorld;
   }

   public OpenCVArUcoMarker getArUcoMarker()
   {
      return arUcoMarker;
   }

   public ReferenceFrame getVirtualFrame()
   {
      return virtualFrame;
   }
}
