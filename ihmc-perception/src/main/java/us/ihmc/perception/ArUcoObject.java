package us.ihmc.perception;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class ArUcoObject
{
   private RigidBodyTransform markerToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerToWorld);
   private final RigidBodyTransform transformToHandle = new RigidBodyTransform();
   // object frame might be different than marker location, for example the door handle is not exactly where the marker is on the door
   private ReferenceFrame objectFrame;
   private FramePose3DBasics objectPose;

   public ArUcoObject(int id, ArUcoObjectInfo arucoInfo)
   {
      transformToHandle.set(arucoInfo.getObjectYawPitchRoll(id), arucoInfo.getObjectTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(markerFrame, transformToHandle);
   }

   public void update()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void packToObjectPose(FramePose3DBasics markerPose)
   {
      objectPose = markerPose;
      objectPose.appendTranslation(transformToHandle.getTranslation());
      objectPose.appendRotation(transformToHandle.getRotation());
   }

   public RigidBodyTransform getMarkerToWorld()
   {
      return markerToWorld;
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectFrame;
   }

   public FramePose3DReadOnly getObjectPose()
   {
      return objectPose;
   }
}
