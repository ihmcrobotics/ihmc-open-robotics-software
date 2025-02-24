package us.ihmc.perception.detections.doors;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNode.DoorSide;

public class DoorOpeningMechanism
{
   private String name = null;
   private DoorSide side = null;
   private final Pose3D pose;

   public DoorOpeningMechanism()
   {
      pose = new Pose3D();
      pose.setToNaN();
   }

   /* package-private */ void setName(String name)
   {
      this.name = name;
      setDoorSideByName();
   }

   private void setDoorSideByName()
   {
      if (name == null)
         return;

      if (name.contains("pull_handle"))
         setDoorSide(DoorSide.PULL);
      else if (name.contains("push_bar"))
         setDoorSide(DoorSide.PUSH);
   }

   /* package-private */ void setPosition(Point3DReadOnly position)
   {
      pose.getPosition().set(position);
   }

   /* package-private */ void setOrientation(Orientation3DReadOnly orientation)
   {
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
}
