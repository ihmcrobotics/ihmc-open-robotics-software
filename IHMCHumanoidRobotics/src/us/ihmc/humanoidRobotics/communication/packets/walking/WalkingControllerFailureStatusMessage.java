package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class WalkingControllerFailureStatusMessage extends StatusPacket<WalkingControllerFailureStatusMessage>
{
   public Vector2D32 fallingDirection;

   public WalkingControllerFailureStatusMessage()
   {
   }

   public WalkingControllerFailureStatusMessage(Vector2D fallingDirection)
   {
      this.fallingDirection = new Vector2D32(fallingDirection);
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
   
   public void setFallingDirection(FrameVector2d fallingDirection)
   {
      fallingDirection.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      if (this.fallingDirection == null)
         this.fallingDirection = new Vector2D32();
      this.fallingDirection.set(fallingDirection.getVector());
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
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      return fallingDirection.epsilonEquals(other.fallingDirection, (float) epsilon);
   }
}
