package us.ihmc.aware.packets;

import us.ihmc.communication.packets.Packet;

import javax.vecmath.Vector3d;

public class TwistPacket extends Packet<TwistPacket>
{
   private final Vector3d linearVelocity;
   private final Vector3d angularVelocity;

   public TwistPacket()
   {
      this.linearVelocity = new Vector3d();
      this.angularVelocity = new Vector3d();
   }

   public TwistPacket(Vector3d linearVelocity, Vector3d angularVelocity)
   {
      this.linearVelocity = new Vector3d(linearVelocity);
      this.angularVelocity = new Vector3d(angularVelocity);
   }

   public TwistPacket(double vx, double vy, double vz, double wx, double wy, double wz)
   {
      this.linearVelocity = new Vector3d(vx, vy, vz);
      this.angularVelocity = new Vector3d(wx, wy, wz);
   }

   public void get(Vector3d linearVelocity, Vector3d angularVelocity)
   {
      linearVelocity.set(this.linearVelocity);
      angularVelocity.set(this.angularVelocity);
   }

   public void getLinearVelocity(Vector3d linearVelocity)
   {
      linearVelocity.set(this.linearVelocity);
   }

   public void getAngularVelocity(Vector3d angularVelocity)
   {
      angularVelocity.set(this.angularVelocity);
   }

   public double getLinearVelocityX()
   {
      return linearVelocity.getX();
   }

   public double getLinearVelocityY()
   {
      return linearVelocity.getY();
   }

   public double getLinearVelocityZ()
   {
      return linearVelocity.getZ();
   }

   public double getAngularVelocityX()
   {
      return angularVelocity.getX();
   }

   public double getAngularVelocityY()
   {
      return angularVelocity.getY();
   }

   public double getAngularVelocityZ()
   {
      return angularVelocity.getZ();
   }

   @Override public boolean epsilonEquals(TwistPacket other, double epsilon)
   {
      return (this.linearVelocity.epsilonEquals(other.linearVelocity, epsilon) && this.angularVelocity.epsilonEquals(other.angularVelocity, epsilon));
   }
}
