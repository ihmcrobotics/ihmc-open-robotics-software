package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorOpeningMechanismMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
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
      frame.update(transformToWorld -> MessageTools.toEuclid(message.getTransformToWorld(), transformToWorld));
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
         if (isTranslationKnown())
            transformToWorld.getTranslation().interpolate(newPosition, alpha);
         else
            transformToWorld.getTranslation().set(newPosition);
      });
   }

   /* package-private */ void updateOrientation(RotationMatrixReadOnly orientation, double alpha)
   {
      frame.update(transformToWorld ->
      {
         if (isRotationKnown())
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

   public boolean isTranslationKnown()
   {
      return !getTransformToWorld().getTranslation().containsNaN();
   }

   public boolean isRotationKnown()
   {
      return !getTransformToWorld().getRotation().containsNaN();
   }

   public boolean isTransformKnown()
   {
      return !getTransformToWorld().containsNaN();
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
      MessageTools.toMessage(frame.getTransformToParent(), messageToPack.getTransformToWorld());
   }
}
