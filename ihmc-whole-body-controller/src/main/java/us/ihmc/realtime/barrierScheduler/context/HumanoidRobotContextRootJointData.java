package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextRootJointData implements InPlaceCopyable<HumanoidRobotContextRootJointData>
{
   private final Quaternion rootJointOrientation = new Quaternion();
   private final Vector3D rootJointAngularVelocity = new Vector3D();
   private final Vector3D rootJointAngularAcceleration = new Vector3D();

   /**
    * Called by the update methods in the {@link us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.UniBinding} when receiving data from another task.
    *
    * @param src The data being received
    */
   @Override
   public void copyFrom(HumanoidRobotContextRootJointData src)
   {
      this.rootJointOrientation.set(src.rootJointOrientation);
      this.rootJointAngularVelocity.set(src.rootJointAngularVelocity);
      this.rootJointAngularAcceleration.set(src.rootJointAngularAcceleration);
   }

   public void setRootJointOrientation(QuaternionReadOnly other)
   {
      this.rootJointOrientation.set(other);
   }

   public void setRootJointOrientation(double x, double y, double z, double s)
   {
      this.rootJointOrientation.set(x, y, z, s);
   }

   public void setRootJointAngularVelocity(double x, double y, double z)
   {
      this.rootJointAngularVelocity.set(x, y, z);
   }

   public void setRootJointAngularVelocity(Vector3DReadOnly other)
   {
      this.rootJointAngularVelocity.set(other);
   }

   public void setRootJointAngularAcceleration(double x, double y, double z)
   {
      this.rootJointAngularAcceleration.set(x, y, z);
   }

   public void setRootJointAngularAcceleration(Vector3DReadOnly other)
   {
      this.rootJointAngularAcceleration.set(other);
   }

   public void getRootJointOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(this.rootJointOrientation);
   }

   public QuaternionReadOnly getRootJointOrientation()
   {
      return this.rootJointOrientation;
   }

   public Vector3DReadOnly getRootJointAngularVelocity()
   {
      return rootJointAngularVelocity;
   }

   public void getRootJointAngularVelocity(Vector3D angularVelocityToPack)
   {
      angularVelocityToPack.set(this.rootJointAngularVelocity);
   }

   public Vector3DReadOnly getRootJointAngularAcceleration()
   {
      return rootJointAngularAcceleration;
   }

   public void getRootJointAngularAcceleration(Vector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.set(this.rootJointAngularAcceleration);
   }
}
