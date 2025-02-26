package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;

public class DoorOpeningMechanism
{
   private String name = "unknown";
   private DoorSide side = DoorSide.UNKNOWN;
   private final Pose3D pose;

   public DoorOpeningMechanism()
   {
      pose = new Pose3D();
      pose.setToNaN();
   }

   /* package-private */ DoorOpeningMechanism(DetectedDoorOpeningMechanismMessage message)
   {
      name = message.getNameAsString();
      side = DoorSide.values()[message.getSide()];
      pose = new Pose3D(message.getPose());
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
      if (isPositionKnown())
         pose.getPosition().interpolate(newPosition, alpha);
      else
         pose.getPosition().set(newPosition);
   }

   /* package-private */ void updateOrientation(QuaternionReadOnly orientation, double alpha)
   {
      if (isOrientationKnown())
         pose.getRotation().interpolate(orientation, alpha);
      else
         pose.getRotation().set(orientation);
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
      return !pose.getPosition().containsNaN();
   }

   public boolean isOrientationKnown()
   {
      return !pose.getRotation().containsNaN();
   }

   public boolean isPoseKnown()
   {
      return !pose.containsNaN();
   }

   public Point3DReadOnly getPosition()
   {
      return pose.getPosition();
   }

   public QuaternionReadOnly getOrientation()
   {
      return pose.getOrientation();
   }

   public Pose3DReadOnly getPose()
   {
      return pose;
   }

   public void toMessage(DetectedDoorOpeningMechanismMessage messageToPack)
   {
      messageToPack.setName(name);
      messageToPack.setSide((byte) side.ordinal());
      messageToPack.getPose().set(pose);
   }
}
