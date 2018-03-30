package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class WalkingControllerFailureStatusMessage extends Packet<WalkingControllerFailureStatusMessage>
{
   public Vector3D fallingDirection = new Vector3D();

   public WalkingControllerFailureStatusMessage()
   {
   }

   public void setFallingDirection(Vector3DReadOnly fallingDirection)
   {
      this.fallingDirection.set(fallingDirection);
   }

   public Vector3D getFallingDirection()
   {
      return fallingDirection;
   }

   @Override
   public void set(WalkingControllerFailureStatusMessage other)
   {
      fallingDirection.set(other.fallingDirection);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      return fallingDirection.epsilonEquals(other.fallingDirection, (float) epsilon);
   }
}
