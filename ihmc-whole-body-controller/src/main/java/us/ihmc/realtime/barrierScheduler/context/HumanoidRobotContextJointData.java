package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextJointData implements InPlaceCopyable<HumanoidRobotContextJointData>
{
   private final HumanoidRobotContextRootJointData rootJointData = new HumanoidRobotContextRootJointData();
   private final double[] jointQ;
   private final double[] jointQd;
   private final double[] jointQdd;
   private final double[] jointTau;

   public HumanoidRobotContextJointData(int numberOfJoints)
   {
      jointQ = new double[numberOfJoints];
      jointQd = new double[numberOfJoints];
      jointQdd = new double[numberOfJoints];
      jointTau = new double[numberOfJoints];
   }

   public void setJointQForIndex(int index, double value)
   {
      this.jointQ[index] = value;
   }

   public void setJointQdForIndex(int index, double value)
   {
      this.jointQd[index] = value;
   }

   public void setJointQddForIndex(int index, double value)
   {
      this.jointQdd[index] = value;
   }

   public void setJointTauForIndex(int index, double value)
   {
      this.jointTau[index] = value;
   }

   public void setJointQs(double[] other)
   {
      System.arraycopy(other, 0, this.jointQ, 0, other.length);
   }

   public void setJointQds(double[] other)
   {
      System.arraycopy(other, 0, this.jointQd, 0, other.length);
   }

   public void setJointQdds(double[] other)
   {
      System.arraycopy(other, 0, this.jointQdd, 0, other.length);
   }

   public void setJointTaus(double[] other)
   {
      System.arraycopy(other, 0, this.jointTau, 0, other.length);
   }

   public void setRootJointData(HumanoidRobotContextRootJointData other)
   {
      this.rootJointData.copyFrom(other);
   }

   public void setRootJointData(QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, Vector3DReadOnly angularAcceleration)
   {
      this.rootJointData.setRootJointOrientation(orientation);
      this.rootJointData.setRootJointAngularVelocity(angularVelocity);
      this.rootJointData.setRootJointAngularAcceleration(angularAcceleration);
   }

   public HumanoidRobotContextRootJointData getRootJointData()
   {
      return rootJointData;
   }

   public void getAndPackRootJointData(HumanoidRobotContextRootJointData dataToPack)
   {
      dataToPack.copyFrom(this.rootJointData);
   }

   public void getAndPackJointQ(double[] dataToPack)
   {
      System.arraycopy(this.jointQ, 0, dataToPack, 0, this.jointQ.length);
   }

   public void getAndPackJointQd(double[] dataToPack)
   {
      System.arraycopy(this.jointQd, 0, dataToPack, 0, this.jointQd.length);
   }

   public void getAndPackJointQdd(double[] dataToPack)
   {
      System.arraycopy(this.jointQdd, 0, dataToPack, 0, this.jointQdd.length);
   }

   public void getAndPackJointTau(double[] dataToPack)
   {
      System.arraycopy(this.jointTau, 0, dataToPack, 0, this.jointTau.length);
   }

   @Override
   public void copyFrom(HumanoidRobotContextJointData src)
   {
      System.arraycopy(src.jointQ, 0, this.jointQ, 0, this.jointQ.length);
      System.arraycopy(src.jointQd, 0, this.jointQd, 0, this.jointQd.length);
      System.arraycopy(src.jointQdd, 0, this.jointQdd, 0, this.jointQdd.length);
      System.arraycopy(src.jointTau, 0, this.jointTau, 0, this.jointTau.length);

      this.rootJointData.copyFrom(src.rootJointData);
   }
}
