package us.ihmc.robotics.math.trajectories;


import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public abstract class FrameNDoFTrapezoidalVelocityTrajectory extends AbstractReferenceFrameHolder
{
   private final ReferenceFrame referenceFrame;
   private final NDoFTrapezoidalVelocityTrajectory nDoFTrapezoidalVelocityTrajectory;

   public FrameNDoFTrapezoidalVelocityTrajectory(ReferenceFrame referenceFrame, double t0, double[] x0, double[] xF, double[] v0, double[] vF, double[] vMax,
           double[] aMax, AlphaToAlphaType alphaToAlphaType)
   {
      this.referenceFrame = referenceFrame;
      this.nDoFTrapezoidalVelocityTrajectory = new NDoFTrapezoidalVelocityTrajectory(t0, x0, xF, v0, vF, vMax, aMax, alphaToAlphaType);
   }

   public void synchronize(double tFMax)
   {
      nDoFTrapezoidalVelocityTrajectory.synchronize(tFMax);
   }

   public void synchronize()
   {
      nDoFTrapezoidalVelocityTrajectory.synchronize();
   }

   public void synchronizeWith(FrameNDoFTrapezoidalVelocityTrajectory trajectory)
   {
      this.nDoFTrapezoidalVelocityTrajectory.synchronizeWith(trajectory.nDoFTrapezoidalVelocityTrajectory);
   }

   public double getT0()
   {
      return nDoFTrapezoidalVelocityTrajectory.getT0();
   }

   public double[] getX0Array()
   {
      return nDoFTrapezoidalVelocityTrajectory.getX0Array();
   }

   public double[] getV0Array()
   {
      return nDoFTrapezoidalVelocityTrajectory.getV0Array();
   }

   public double[] getTFArray()
   {
      return nDoFTrapezoidalVelocityTrajectory.getTFArray();
   }

   public double getTFMax()
   {
      return nDoFTrapezoidalVelocityTrajectory.getTFMax();
   }

   public double getDTFMax()
   {
      return nDoFTrapezoidalVelocityTrajectory.getDTFMax();
   }

   protected double getPosition(int index, double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getPosition(index, t);
   }

   protected double getVelocity(int index, double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getVelocity(index, t);
   }

   protected double getAcceleration(int index, double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getAcceleration(index, t);
   }

   protected double getMaximumVelocity(int index)
   {
      return nDoFTrapezoidalVelocityTrajectory.getMaximumVelocity(index);
   }

   protected double getMaximumAcceleration(int index)
   {
      return nDoFTrapezoidalVelocityTrajectory.getMaximumAcceleration(index);
   }

   protected double[] getPositionArray(double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getPositionArray(t);
   }

   protected double[] getVelocityArray(double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getVelocityArray(t);
   }

   protected double[] getAccelerationArray(double t)
   {
      return nDoFTrapezoidalVelocityTrajectory.getAccelerationArray(t);
   }

   protected double[] getMaximumVelocityArray()
   {
      return nDoFTrapezoidalVelocityTrajectory.getMaximumVelocityArray();
   }

   protected double[] getMaximumAccelerationArray()
   {
      return nDoFTrapezoidalVelocityTrajectory.getMaximumAccelerationArray();
   }

   protected int size()
   {
      return nDoFTrapezoidalVelocityTrajectory.size();
   }

   public abstract AbstractReferenceFrameHolder getPosition(double t);

   public abstract AbstractReferenceFrameHolder getVelocity(double t);

   public abstract AbstractReferenceFrameHolder getAcceleration(double t);

   public abstract AbstractReferenceFrameHolder getMaximumVelocity();

   public abstract AbstractReferenceFrameHolder getMaximumAcceleration();

   public abstract AbstractReferenceFrameHolder getInitialPosition();

   public abstract AbstractReferenceFrameHolder getInitialVelocity();

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public static void synchronize(FrameNDoFTrapezoidalVelocityTrajectory[] trajectories)
   {
      // does not require reference frame match!
      double tFMaxMax = getTFMaxMax(trajectories);
      for (FrameNDoFTrapezoidalVelocityTrajectory trajectory : trajectories)
      {
         trajectory.synchronize(tFMaxMax);
      }
   }

   protected static void doReferenceFrameChecks(AbstractReferenceFrameHolder x0, AbstractReferenceFrameHolder xF, AbstractReferenceFrameHolder v0, AbstractReferenceFrameHolder vF,
         AbstractReferenceFrameHolder vMax, AbstractReferenceFrameHolder aMax)
   {
      x0.checkReferenceFrameMatch(xF.getReferenceFrame());
      x0.checkReferenceFrameMatch(v0.getReferenceFrame());
      x0.checkReferenceFrameMatch(vF.getReferenceFrame());
      x0.checkReferenceFrameMatch(vMax.getReferenceFrame());
      x0.checkReferenceFrameMatch(aMax.getReferenceFrame());
   }

   private static double getTFMaxMax(FrameNDoFTrapezoidalVelocityTrajectory[] trajectories)
   {
      double tFMaxMax = Double.NEGATIVE_INFINITY;
      for (FrameNDoFTrapezoidalVelocityTrajectory trajectory : trajectories)
      {
         double tFMax = trajectory.getTFMax();
         if (tFMax > tFMaxMax)
         {
            tFMaxMax = tFMax;
         }
      }

      return tFMaxMax;
   }

   @Override
   public String toString()
   {
      return "nDoFTrapezoidalVelocityTrajectory:\n" + nDoFTrapezoidalVelocityTrajectory.toString() + "\n" + "referenceFrame: " + referenceFrame.toString();
   }
}
