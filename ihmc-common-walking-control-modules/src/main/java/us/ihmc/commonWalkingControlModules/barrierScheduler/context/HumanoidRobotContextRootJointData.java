package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextRootJointData implements InPlaceCopyable<HumanoidRobotContextRootJointData>
{
   private final Quaternion rootJointOrientation = new Quaternion();
   private final Vector3D rootJointAngularVelocity = new Vector3D();
   private final Vector3D rootJointAngularAcceleration = new Vector3D();

   private final Point3D rootJointLocation = new Point3D();
   private final Vector3D rootJointLinearVelocity = new Vector3D();
   private final Vector3D rootJointLinearAcceleration = new Vector3D();

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
      this.rootJointLocation.set(src.rootJointLocation);
      this.rootJointLinearVelocity.set(src.rootJointLinearVelocity);
      this.rootJointLinearAcceleration.set(src.rootJointLinearAcceleration);
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

   public void setRootJointLocation(Tuple3DReadOnly other)
   {
      this.rootJointLocation.set(other);
   }

   public void setRootJointLocation(double x, double y, double z)
   {
      this.rootJointLocation.set(x, y, z);
   }

   public void setRootJointLinearVelocity(Tuple3DReadOnly other)
   {
      this.rootJointLinearVelocity.set(other);
   }

   public void setRootJointLinearVelocity(double x, double y, double z)
   {
      this.rootJointLinearVelocity.set(x, y, z);
   }

   public void setRootJointLinearAcceleration(Tuple3DReadOnly other)
   {
      this.rootJointLinearAcceleration.set(other);
   }

   public void setRootJointLinearAcceleration(double x, double y, double z)
   {
      this.rootJointLinearAcceleration.set(x, y, z);
   }

   public void getRootJointOrientation(QuaternionBasics orientationToPack)
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

   public void getRootJointAngularVelocity(Tuple3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(this.rootJointAngularVelocity);
   }

   public Vector3DReadOnly getRootJointAngularAcceleration()
   {
      return rootJointAngularAcceleration;
   }

   public void getRootJointAngularAcceleration(Tuple3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.set(this.rootJointAngularAcceleration);
   }

   public Point3DReadOnly getRootJointLocation()
   {
      return rootJointLocation;
   }

   public Vector3DReadOnly getRootJointLinearVelocity()
   {
      return rootJointLinearVelocity;
   }

   public Vector3DReadOnly getRootJointLinearAcceleration()
   {
      return rootJointLinearAcceleration;
   }

   public void getRootJointLocation(Tuple3DBasics locationToPack)
   {
      locationToPack.set(rootJointLocation);
   }

   public void getRootJointLinearVelocity(Tuple3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.set(rootJointLinearVelocity);
   }

   public void getRootJointLinearAcceleration(Tuple3DBasics linearAccelerationToPack)
   {
      linearAccelerationToPack.set(rootJointLinearAcceleration);
   }
}
