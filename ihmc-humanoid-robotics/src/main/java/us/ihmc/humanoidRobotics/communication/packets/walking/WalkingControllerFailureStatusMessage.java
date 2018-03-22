package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;

public class WalkingControllerFailureStatusMessage extends Packet<WalkingControllerFailureStatusMessage>
{
   public Vector2D32 fallingDirection;

   public WalkingControllerFailureStatusMessage()
   {
   }

   public void setFallingDirection(Vector2D32 fallingDirection)
   {
      this.fallingDirection = fallingDirection;
   }

   public void setFallingDirection(Vector2D fallingDirection)
   {
      if (this.fallingDirection == null)
         this.fallingDirection = new Vector2D32();
      this.fallingDirection.set(fallingDirection);
   }
   
   public void setFallingDirection(FrameVector2D fallingDirection)
   {
      fallingDirection.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      if (this.fallingDirection == null)
         this.fallingDirection = new Vector2D32();
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
