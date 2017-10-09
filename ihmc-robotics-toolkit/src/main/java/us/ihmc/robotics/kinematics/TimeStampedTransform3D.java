package us.ihmc.robotics.kinematics;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class TimeStampedTransform3D
{
   public RigidBodyTransform transform3D = new RigidBodyTransform();
   public long timeStamp;

   public TimeStampedTransform3D()
   {
   }

   public TimeStampedTransform3D(RigidBodyTransform transform3D, long timeStamp)
   {
      this.transform3D.set(transform3D);
      this.timeStamp = timeStamp;
   }

   public void setTransform3D(RigidBodyTransform transform3D)
   {
      this.transform3D.set(transform3D);
   }

   public void setTimeStamp(long timeStamp)
   {
      this.timeStamp = timeStamp;
   }

   public long getTimeStamp()
   {
      return this.timeStamp;
   }

   public RigidBodyTransform getTransform3D()
   {
      return this.transform3D;
   }

   public void set(TimeStampedTransform3D other)
   {
      setTransform3D(other.getTransform3D());
      setTimeStamp(other.getTimeStamp());
   }

   public boolean epsilonEquals(TimeStampedTransform3D other, double epsilon)
   {
      boolean haveTheSameTransform = transform3D.epsilonEquals(other.transform3D, epsilon);
      boolean haveTheSameTimestamp = timeStamp == other.timeStamp;

      return haveTheSameTransform && haveTheSameTimestamp;
   }

   @Override
   public String toString()
   {
      String ret = "Timestamp: " + timeStamp + "\n";
      ret += "Transform: \n" + transform3D;
      return ret;
   }
}