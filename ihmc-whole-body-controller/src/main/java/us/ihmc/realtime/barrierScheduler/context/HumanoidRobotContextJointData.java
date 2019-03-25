package us.ihmc.realtime.barrierScheduler.context;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class HumanoidRobotContextJointData implements InPlaceCopyable<HumanoidRobotContextJointData>
{
   private final double[] jointQ;
   private final double[] jointQd;
   private final double[] jointQdd;
   private final double[] jointTau;

   HumanoidRobotContextJointData(int numberOfJoints)
   {
      jointQ = new double[numberOfJoints];
      jointQd = new double[numberOfJoints];
      jointQdd = new double[numberOfJoints];
      jointTau = new double[numberOfJoints];
   }

   public void getJointQ(double[] dataToPack)
   {
      System.arraycopy(this.jointQ, 0, dataToPack, 0, this.jointQ.length);
   }

   public void getJointQd(double[] dataToPack)
   {
      System.arraycopy(this.jointQd, 0, dataToPack, 0, this.jointQd.length);
   }

   public void getJointQdd(double[] dataToPack)
   {
      System.arraycopy(this.jointQdd, 0, dataToPack, 0, this.jointQdd.length);
   }

   public void getJointTau(double[] dataToPack)
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
   }
}
