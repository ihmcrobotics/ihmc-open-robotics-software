package us.ihmc.communication.packets.walking;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.communication.packets.dataobjects.BlindWalkingSpeed;

public class BlindWalkingPacket extends Packet<BlindWalkingPacket>
{
   public Point2d desiredDestination;
   public BlindWalkingDirection blindWalkingDirection;

   // TODO set this based on slider
   public BlindWalkingSpeed blindWalkingSpeed = BlindWalkingSpeed.SLOW;
   public boolean isInMud;

   public BlindWalkingPacket()
   {
      // Empty constructor for deserialization
   }

   public BlindWalkingPacket(Point2d desiredDestination, BlindWalkingDirection blindWalkingDirection, BlindWalkingSpeed blindWalkingSpeed, boolean isInMud)
   {
      this.desiredDestination = desiredDestination;
      this.blindWalkingDirection = blindWalkingDirection;
      this.blindWalkingSpeed = blindWalkingSpeed;
      this.isInMud = isInMud;
   }

   public Point2d getDesiredDestination()
   {
      return desiredDestination;
   }

   public void setDesiredDestination(Point2d finalDesiredPositionInWorld)
   {
      this.desiredDestination = finalDesiredPositionInWorld;
   }

   public BlindWalkingDirection getBlindWalkingDirection()
   {
      return blindWalkingDirection;
   }

   public void setBlindWalkingDirection(BlindWalkingDirection blindWalkingDirection)
   {
      this.blindWalkingDirection = blindWalkingDirection;
   }

   public BlindWalkingSpeed getBlindWalkingSpeed()
   {
      return blindWalkingSpeed;
   }

   public void setBlindWalkingSpeed(BlindWalkingSpeed blindWalkingSpeed)
   {
      this.blindWalkingSpeed = blindWalkingSpeed;
   }

   public boolean getIsInMud()
   {
      return isInMud;
   }

   public void setIsInMud(boolean isInMud)
   {
      this.isInMud = isInMud;
   }

   public BlindWalkingPacket transform(Vector3d offset)
   {
      Point2d newPoint = new Point2d(desiredDestination.getX() + offset.getX(), desiredDestination.getY() + offset.getY());

      return new BlindWalkingPacket(newPoint, getBlindWalkingDirection(), getBlindWalkingSpeed(), getIsInMud());
   }

   public String toString()
   {
      return "Blind: finalDesiredPositionInWorld = " + desiredDestination + ", blindWalkingSpeed = " + blindWalkingSpeed + ", blindWalkingDirection = "
             + blindWalkingDirection + ", isInMud = " + isInMud;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof BlindWalkingPacket) && this.epsilonEquals((BlindWalkingPacket) obj, 0.05));
   }

   @Override
   public boolean epsilonEquals(BlindWalkingPacket other, double epsilon)
   {
      boolean ret = this.getDesiredDestination().epsilonEquals(other.getDesiredDestination(), epsilon);
      ret &= (this.getBlindWalkingDirection().equals(other.getBlindWalkingDirection()));
      ret &= (this.getBlindWalkingSpeed().equals(other.getBlindWalkingSpeed()));

      return ret;
   }

   public BlindWalkingPacket(Random random)
   {
      final double X_MAX = 128.0;
      final double Y_MAX = 128.0;

      Point2d point = new Point2d(0.9 * random.nextDouble() * X_MAX, 0.9 * random.nextDouble() * Y_MAX);
      BlindWalkingDirection direction = BlindWalkingDirection.fromIntegerValue(random.nextInt(4));
      BlindWalkingSpeed speed = BlindWalkingSpeed.fromIntegerValue(random.nextInt(3));
      Boolean isInMud = random.nextBoolean();

      this.desiredDestination = point;
      this.blindWalkingDirection = direction;
      this.blindWalkingSpeed = speed;
      this.isInMud = isInMud;

   }
}
