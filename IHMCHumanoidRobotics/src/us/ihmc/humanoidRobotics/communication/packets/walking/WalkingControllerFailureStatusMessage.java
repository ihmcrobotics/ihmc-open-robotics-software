package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector2f;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class WalkingControllerFailureStatusMessage extends StatusPacket<WalkingControllerFailureStatusMessage>
{
   public Vector2f fallingDirection;

   public WalkingControllerFailureStatusMessage()
   {
   }

   public WalkingControllerFailureStatusMessage(Vector2d fallingDirection)
   {
      this.fallingDirection = new Vector2f(fallingDirection);
   }

   public void setFallingDirection(Vector2f fallingDirection)
   {
      this.fallingDirection = fallingDirection;
   }

   public void setFallingDirection(Vector2d fallingDirection)
   {
      if (this.fallingDirection == null)
         this.fallingDirection = new Vector2f();
      this.fallingDirection.set(fallingDirection);
   }
   
   public void setFallingDirection(FrameVector2d fallingDirection)
   {
      fallingDirection.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      if (this.fallingDirection == null)
         this.fallingDirection = new Vector2f();
      this.fallingDirection.set(fallingDirection.getVector());
   }

   public Vector2f getFallingDirection()
   {
      return fallingDirection;
   }

   @Override
   public void set(WalkingControllerFailureStatusMessage other)
   {
      if (other.fallingDirection != null)
      {
         if (fallingDirection == null)
            fallingDirection = new Vector2f();
         fallingDirection.set(other.fallingDirection);
      }
   }

   @Override
   public boolean epsilonEquals(WalkingControllerFailureStatusMessage other, double epsilon)
   {
      return fallingDirection.epsilonEquals(other.fallingDirection, (float) epsilon);
   }
}
