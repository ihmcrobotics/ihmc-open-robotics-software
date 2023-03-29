package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class ArUcoMarkerObject
{
   private final RigidBodyTransform markerTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerTransformToWorld);
   private final RigidBodyTransform markerTransformToObject = new RigidBodyTransform();
   // object frame might be different from marker location, for example the door handle is not exactly where the marker is on the door
   private final ReferenceFrame objectFrame;
   private final FramePose3D objectPose = new FramePose3D();

   public ArUcoMarkerObject(int id, ArUcoMarkerObjectsInfo arucoInfo)
   {
      markerTransformToObject.set(arucoInfo.getMarkerYawPitchRoll(id), arucoInfo.getMarkerTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(markerFrame, markerTransformToObject);
   }

   public void updateMarkerTransform(FrameTuple3DReadOnly markerTranslation, FrameQuaternionReadOnly markerOrientation)
   {
      markerTranslation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      markerOrientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      markerTransformToWorld.set(markerOrientation, markerTranslation);
      markerFrame.update();
   }

   public void updateFrame()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void computeObjectPose(FramePose3DReadOnly markerPose)
   {
      objectPose.set(markerPose);
      objectPose.appendTransform(markerTransformToObject);
   }

   public RigidBodyTransformReadOnly getMarkerTransformToWorld()
   {
      return markerTransformToWorld;
   }

   public ReferenceFrame getMarkerFrame()
   {
      return markerFrame;
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
