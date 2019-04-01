package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextRootJointData implements InPlaceCopyable<HumanoidRobotContextRootJointData>
{
   private final Quaternion rootJointOrientation = new Quaternion();
   private double rootJointAngularVelocity;
   private double rootJointAngularAcceleration;

   /**
    * Called by the update methods in the {@link us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.UniBinding} when receiving data from another task.
    *
    * @param src The data being received
    */
   @Override
   public void copyFrom(HumanoidRobotContextRootJointData src)
   {
      this.rootJointOrientation.set(src.rootJointOrientation);
      this.rootJointAngularVelocity = src.rootJointAngularVelocity;
      this.rootJointAngularAcceleration = src.rootJointAngularAcceleration;
   }

   public void setRootJointOrientation(QuaternionReadOnly other)
   {
      this.rootJointOrientation.set(other);
   }

   public void setRootJointOrientation(double x, double y, double z, double s)
   {
      this.rootJointOrientation.set(x, y, z, s);
   }

   public void setRootJointAngularVelocity(double rootJointAngularVelocity)
   {
      this.rootJointAngularVelocity = rootJointAngularVelocity;
   }

   public void setRootJointAngularAcceleration(double rootJointAngularAcceleration)
   {
      this.rootJointAngularAcceleration = rootJointAngularAcceleration;
   }

   public void getRootJointOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(this.rootJointOrientation);
   }

   public QuaternionReadOnly getRootJointOrientation()
   {
      return this.rootJointOrientation;
   }

   public double getRootJointAngularVelocity()
   {
      return rootJointAngularVelocity;
   }

   public double getRootJointAngularAcceleration()
   {
      return rootJointAngularAcceleration;
   }
}
