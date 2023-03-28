package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.function.Consumer;

public class ArUcoMarkerObject
{
   private RigidBodyTransform markerTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                     markerTransformToWorld);
   private final RigidBodyTransform objectTransformToMarker = new RigidBodyTransform();
   // object frame might be different from marker location, for example the door handle is not exactly where the marker is on the door
   private ReferenceFrame objectFrame;
   private FramePose3D objectPose = new FramePose3D();

   public ArUcoMarkerObject(int id, ArUcoMarkerObjectInfo arucoInfo)
   {
      objectTransformToMarker.set(arucoInfo.getMarkerYawPitchRoll(id), arucoInfo.getMarkerTranslation(id));
      objectFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(markerFrame, objectTransformToMarker);
   }

   public void updateMarkerTransform(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      markerTransformToWorld.getTranslation().set(position);
      markerTransformToWorld.getRotation().set(orientation);
      markerFrame.update();
   }

   public void updateFrame()
   {
      markerFrame.update();
      objectFrame.update();
   }

   public void computeObjectPose(FramePose3DBasics markerPose)
   {
      objectPose.set(markerPose);
      objectPose.appendTransform(objectTransformToMarker);
   }

   public RigidBodyTransform getMarkerTransformToWorld()
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

   public FramePose3D getObjectPose()
   {
      return objectPose;
   }
}
