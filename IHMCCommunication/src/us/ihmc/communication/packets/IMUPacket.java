package us.ihmc.communication.packets;

import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;


public class IMUPacket extends Packet<IMUPacket>
{
   public Vector3f linearAcceleration = new Vector3f();
   public Quat4f orientation = new Quat4f();
   public Vector3f angularVelocity = new Vector3f();
   
   public IMUPacket()
   {
   }

   public void set(Vector3f linearAcceleration, Quat4f orientation, Vector3f angularVelocity)
   {
      this.linearAcceleration.set(linearAcceleration);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
   }

   public Vector3f getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public Quat4f getOrientation()
   {
      return orientation;
   }

   public Vector3f getAngularVelocity()
   {
      return angularVelocity;
   }

   @Override
   public boolean epsilonEquals(IMUPacket other, double epsilon)
   {
      boolean ret = linearAcceleration.epsilonEquals(other.linearAcceleration, (float) epsilon);
      ret &= orientation.epsilonEquals(other.orientation, (float) epsilon);
      ret &= angularVelocity.epsilonEquals(other.angularVelocity, (float) epsilon);
      return ret;
   }
}
