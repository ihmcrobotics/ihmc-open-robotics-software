package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextJointData implements InPlaceCopyable<HumanoidRobotContextJointData>
{
   private final HumanoidRobotContextRootJointData rootJointData = new HumanoidRobotContextRootJointData();
   private final TDoubleArrayList jointQ = new TDoubleArrayList(50);
   private final TDoubleArrayList jointQd = new TDoubleArrayList(50);
   private final TDoubleArrayList jointQdd = new TDoubleArrayList(50);
   private final TDoubleArrayList jointTau = new TDoubleArrayList(50);

   public void clear()
   {
      jointQ.resetQuick();
      jointQd.resetQuick();
      jointQdd.resetQuick();
      jointTau.resetQuick();
   }

   public void addJoint(double q, double qd, double qdd, double tau)
   {
      jointQ.add(q);
      jointQd.add(qd);
      jointQdd.add(qdd);
      jointTau.add(tau);
   }

   public void setJointQForIndex(int index, double value)
   {
      this.jointQ.set(index, value);
   }

   public void setJointQdForIndex(int index, double value)
   {
      this.jointQd.set(index, value);
   }

   public void setJointQddForIndex(int index, double value)
   {
      this.jointQdd.set(index, value);
   }

   public void setJointTauForIndex(int index, double value)
   {
      this.jointTau.set(index, value);
   }

   public void setRootJointData(HumanoidRobotContextRootJointData other)
   {
      this.rootJointData.copyFrom(other);
   }

   public void setRootJointAngularData(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, Vector3DReadOnly angularAcceleration)
   {
      this.rootJointData.setRootJointOrientation(orientation);
      this.rootJointData.setRootJointAngularVelocity(angularVelocity);
      this.rootJointData.setRootJointAngularAcceleration(angularAcceleration);
   }

   public void setRootJointLinearData(Point3DReadOnly location, Vector3DReadOnly linearVelocity, Vector3DReadOnly linearAcceleration)
   {
      this.rootJointData.setRootJointLocation(location);
      this.rootJointData.setRootJointLinearVelocity(linearVelocity);
      this.rootJointData.setRootJointLinearAcceleration(linearAcceleration);
   }

   public void setRootJointData(Pose3DReadOnly pose, TwistReadOnly velocity, SpatialAccelerationReadOnly acceleration)
   {
      this.rootJointData.setRootJointOrientation(pose.getOrientation());
      this.rootJointData.setRootJointAngularVelocity(velocity.getAngularPart());
      this.rootJointData.setRootJointAngularAcceleration(acceleration.getAngularPart());
      this.rootJointData.setRootJointLocation(pose.getPosition());
      this.rootJointData.setRootJointLinearVelocity(velocity.getLinearPart());
      this.rootJointData.setRootJointLinearAcceleration(acceleration.getLinearPart());
   }

   public HumanoidRobotContextRootJointData getRootJointData()
   {
      return rootJointData;
   }

   public double getJointQForIndex(int index)
   {
      return this.jointQ.get(index);
   }

   public double getJointQdForIndex(int index)
   {
      return this.jointQd.get(index);
   }

   public double getJointQddForIndex(int index)
   {
      return this.jointQdd.get(index);
   }

   public double getJointTauForIndex(int index)
   {
      return this.jointTau.get(index);
   }

   public void set(HumanoidRobotContextJointData other)
   {
      copyFrom(other);
   }

   @Override
   public void copyFrom(HumanoidRobotContextJointData src)
   {
      copy(src.jointQ, jointQ);
      copy(src.jointQd, jointQd);
      copy(src.jointQdd, jointQdd);
      copy(src.jointTau, jointTau);
      this.rootJointData.copyFrom(src.rootJointData);
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof HumanoidRobotContextJointData)
      {
         HumanoidRobotContextJointData other = (HumanoidRobotContextJointData) obj;
         if (!jointQ.equals(other.jointQ))
            return false;
         if (!jointQd.equals(other.jointQd))
            return false;
         if (!jointQdd.equals(other.jointQdd))
            return false;
         if (!jointTau.equals(other.jointTau))
            return false;
         if (!rootJointData.equals(other.rootJointData))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }

   private static void copy(TDoubleList src, TDoubleArrayList dest)
   {
      dest.resetQuick();
      for (int i = 0; i < src.size(); i++)
         dest.add(src.get(i));
   }
}
