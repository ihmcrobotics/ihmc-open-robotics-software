package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

import java.util.function.Consumer;

public class ArUcoMarkerObject
{
   private final RigidBodyTransform markerTransformToWorld = new RigidBodyTransform();
   private final ReferenceFrame markerFrame
         = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), markerTransformToWorld);
   private final long markerID;
   private final RigidBodyTransform objectTransformToMarker = new RigidBodyTransform();
   private final ReferenceFrame objectFrame;

   public ArUcoMarkerObject(long markerID, String name)
   {
      this.markerID = markerID;
      objectFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(name, markerFrame, objectTransformToMarker);
   }

   public void setObjectTransformToMarker(Consumer<RigidBodyTransform> setter)
   {
      setter.accept(objectTransformToMarker);
      objectFrame.update();
   }

   public void updateMarkerTransform(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      markerTransformToWorld.getTranslation().set(position);
      markerTransformToWorld.getRotation().set(orientation);
      markerFrame.update();
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectFrame;
   }
}
