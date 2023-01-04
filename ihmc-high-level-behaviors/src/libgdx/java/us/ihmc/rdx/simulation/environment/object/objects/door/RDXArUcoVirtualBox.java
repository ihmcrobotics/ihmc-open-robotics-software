package us.ihmc.rdx.simulation.environment.object.objects.door;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.PerceptionManager;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXArUcoVirtualBox extends RDXVirtualGhostObject
{
   private final OpenCVArUcoMarker arUcoMarker;
   private final RigidBodyTransform markerToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerToWorld);
   private final RigidBodyTransform transformToMarker = new RigidBodyTransform();
   private ReferenceFrame virtualFrame;

   public RDXArUcoVirtualBox(int id)
   {
      super("environmentObjects/box/box.g3dj");

      arUcoMarker = new OpenCVArUcoMarker(id, PerceptionManager.BOX_MARKER_WIDTH);
      transformToMarker.set(PerceptionManager.BOX_TRANSFORM_TO_MARKER);
      virtualFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(markerFrame, transformToMarker);
   }

   @Override
   public void update()
   {
      virtualFrame.update();
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
