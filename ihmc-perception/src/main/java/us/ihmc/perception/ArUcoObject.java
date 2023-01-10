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
   private final RigidBodyTransform transformToObject = new RigidBodyTransform();
   // object frame might be different than marker location, for example the door handle is not exactly where the marker is on the door
   private ReferenceFrame objectFrame;
   private FramePose3D objectPose = new FramePose3D();

   public ArUcoObject(int id, ArUcoObjectInfo arucoInfo)
   {
      transformToObject.set(arucoInfo.getObjectYawPitchRoll(id), arucoInfo.getObjectTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(markerFrame, transformToObject);
   }

   public void update()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void computeObjectPose(FramePose3DBasics markerPose)
   {
      objectPose.set(markerPose);
      objectPose.appendTransform(transformToObject);
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
