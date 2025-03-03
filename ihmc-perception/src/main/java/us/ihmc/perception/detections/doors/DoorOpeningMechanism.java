package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

public class DoorOpeningMechanism
{
   private String name = "unknown";
   private DoorSide side = DoorSide.UNKNOWN;
   private final MutableReferenceFrame frame;

   public DoorOpeningMechanism()
   {
      frame = new MutableReferenceFrame();
      frame.update(RigidBodyTransformBasics::setToNaN);
   }

   /* package-private */ DoorOpeningMechanism(DetectedDoorOpeningMechanismMessage message)
   {
      name = message.getNameAsString();
      side = DoorSide.values()[message.getSide()];
      frame = new MutableReferenceFrame();
      frame.update(transformToWorld -> transformToWorld.set(message.getPose()));
   }

   /* package-private */ void setName(String name)
   {
      this.name = name;
      setDoorSideByName();
   }

   private void setDoorSideByName()
   {
      if (name.contains("pull_handle"))
         setDoorSide(DoorSide.PULL);
      else if (name.contains("push_bar"))
         setDoorSide(DoorSide.PUSH);
   }

   /* package-private */ void updatePosition(Point3DReadOnly newPosition, double alpha)
   {
      frame.update(transformToWorld ->
      {
         if (isPositionKnown())
            transformToWorld.getTranslation().interpolate(newPosition, alpha);
         else
            transformToWorld.getTranslation().set(newPosition);
      });
   }

   /* package-private */ void updateOrientation(RotationMatrixReadOnly orientation, double alpha)
   {
      frame.update(transformToWorld ->
      {
         if (isOrientationKnown())
            transformToWorld.getRotation().interpolate(orientation, alpha);
         else
            transformToWorld.getRotation().set(orientation);
      });
   }

   /* package-private */ void setDoorSide(DoorSide side)
   {
      this.side = side;
   }

   public DoorSide getDoorSide()
   {
      return side;
   }

   public String getName()
   {
      return name;
   }

   public boolean isPositionKnown()
   {
      return !getTransformToWorld().getTranslation().containsNaN();
   }

   public boolean isOrientationKnown()
   {
      return !getTransformToWorld().getRotation().containsNaN();
   }

   public boolean isPoseKnown()
   {
      return !getTransformToWorld().containsNaN();
   }

   public Point3DReadOnly getPosition()
   {
      return getPose().getPosition();
   }

   public QuaternionReadOnly getOrientation()
   {
      return getPose().getOrientation();
   }

   public Pose3DReadOnly getPose()
   {
      return new Pose3D(getTransformToWorld());
   }

   public ReferenceFrame getFrame()
   {
      return frame.getReferenceFrame();
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return frame.getTransformToParent();
   }

   public void toMessage(DetectedDoorOpeningMechanismMessage messageToPack)
   {
      messageToPack.setName(name);
      messageToPack.setSide((byte) side.ordinal());
      messageToPack.getPose().set(getPose());
   }
}
