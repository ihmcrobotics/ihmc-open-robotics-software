package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class WalkingControllerFailureStatusMessage extends Packet<WalkingControllerFailureStatusMessage>
{
   public Vector2D32 fallingDirection = new Vector2D32();

   public WalkingControllerFailureStatusMessage()
   {
   }

   public void setFallingDirection(Vector2DReadOnly fallingDirection)
   {
      this.fallingDirection.set(fallingDirection);
   }

   public Vector2D32 getFallingDirection()
   {
      return fallingDirection;
   }

   @Override
   public void set(WalkingControllerFailureStatusMessage other)
   {
      if (other.fallingDirection != null)
      {
         if (fallingDirection == null)
            fallingDirection = new Vector2D32();
         fallingDirection.set(other.fallingDirection);
      }
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      return fallingDirection.epsilonEquals(other.fallingDirection, (float) epsilon);
   }
}
