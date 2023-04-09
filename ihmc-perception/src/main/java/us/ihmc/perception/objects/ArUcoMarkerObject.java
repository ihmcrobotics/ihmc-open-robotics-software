package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.function.Consumer;

/**
 * Represents an actively tracked detected object based on ArUco marker detection.
 *
 * TODO: Add DetectedObject interface or abstract, like the info classes
 */
public class ArUcoMarkerObject
{
   private final RigidBodyTransform markerTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerTransformToWorld);
   private final RigidBodyTransform markerTransformToObject = new RigidBodyTransform();
   private final long markerID;
   private final RigidBodyTransform objectTransformToMarker = new RigidBodyTransform();
   // object frame might be different from marker location, for example the door handle is not exactly where the marker is on the door
   private final ReferenceFrame objectFrame;

   public ArUcoMarkerObject(int id, ArUcoMarkerObjectsInfo arucoInfo, String name)
   {
      markerTransformToObject.set(arucoInfo.getMarkerYawPitchRoll(id), arucoInfo.getMarkerTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(name, markerFrame, markerTransformToObject);
   }

   public void setObjectTransformToMarker(Consumer<RigidBodyTransform> setter)
   {
      setter.accept(objectTransformToMarker);
      objectFrame.update();
   }

   public void updateMarkerTransform(FrameTuple3DReadOnly markerTranslation, FrameQuaternionReadOnly markerOrientation)
   {
      markerTranslation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      markerOrientation.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      markerTransformToWorld.set(markerOrientation, markerTranslation);

      markerTransformToWorld.getTranslation().set(position);
      markerTransformToWorld.getRotation().set(orientation);
      
      markerFrame.update();
   }

   public void updateFrame()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void updateMarkerPose(FramePose3DReadOnly markerPose)
   {
      updateMarkerTransform(markerPose.getPosition(), markerPose.getOrientation());
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

   public FramePose3DReadOnly getObjectPose(ReferenceFrame desiredFrame)
   {
      FramePose3D objectPose = new FramePose3D(objectFrame);
      objectPose.changeFrame(desiredFrame);
      return objectPose;
   }
}
