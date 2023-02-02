package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class ArUcoMarkerObject
{
   private RigidBodyTransform markerToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerToWorld);
   private final RigidBodyTransform objectToMarker = new RigidBodyTransform();
   // object frame might be different than marker location, for example the door handle is not exactly where the marker is on the door
   private ReferenceFrame objectFrame;
   private FramePose3D objectPose = new FramePose3D();

   public ArUcoMarkerObject(int id, ObjectInfo arucoInfo)
   {
      objectToMarker.set(arucoInfo.getMarkerYawPitchRoll(id), arucoInfo.getMarkerTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(markerFrame, objectToMarker);
   }

   public void updateFrame()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void computeObjectPose(FramePose3DBasics markerPose)
   {
      objectPose.set(markerPose);
      objectPose.appendTransform(objectToMarker);
   }

   public RigidBodyTransform getMarkerToWorld()
   {
      return markerToWorld;
   }

   public ReferenceFrame getMarkerFrame()
   {
      return markerFrame;
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectFrame;
   }

   public FramePose3D getObjectPose()
   {
      return objectPose;
   }
}
