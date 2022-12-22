package us.ihmc.perception;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
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
   private FramePose3D objectPose = new FramePose3D();

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

   public void computeObjectPose(FramePose3DBasics markerPose)
   {
      objectPose.set(markerPose);
      objectPose.appendRotation(transformToHandle.getRotation());
      objectPose.appendTranslation(transformToHandle.getTranslation());
   }

   public RigidBodyTransform getMarkerToWorld()
   {
      return markerToWorld;
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
