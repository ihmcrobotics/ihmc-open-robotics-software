package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.robotics.MathTools;


public class IMUPacket extends Packet<IMUPacket>
{
   public Vector3D32 linearAcceleration = new Vector3D32();
   public Quaternion32 orientation = new Quaternion32();
   public Vector3D32 angularVelocity = new Vector3D32();

   public double time;

   public IMUPacket()
   {
   }

   public void set(Vector3D32 linearAcceleration, Quaternion32 orientation, Vector3D32 angularVelocity)
   {
      set(linearAcceleration, orientation, angularVelocity, 0.0);
   }

   public void set(Vector3D32 linearAcceleration, Quaternion32 orientation, Vector3D32 angularVelocity, double time)
   {
      this.linearAcceleration.set(linearAcceleration);
      this.orientation.set(orientation);
      this.angularVelocity.set(angularVelocity);
      this.time = time;
   }

   public Vector3D32 getLinearAcceleration()
   {
      return linearAcceleration;
   }

   public Quaternion32 getOrientation()
   {
      return orientation;
   }

   public Vector3D32 getAngularVelocity()
   {
      return angularVelocity;
   }

   public double getTime()
   {
      return time;
   }

   @Override
   public boolean epsilonEquals(IMUPacket other, double epsilon)
   {
      boolean ret = linearAcceleration.epsilonEquals(other.linearAcceleration, (float) epsilon);
      ret &= orientation.epsilonEquals(other.orientation, (float) epsilon);
      ret &= angularVelocity.epsilonEquals(other.angularVelocity, (float) epsilon);
      ret &= MathTools.epsilonEquals(time, other.time, epsilon);
      return ret;
   }
}
