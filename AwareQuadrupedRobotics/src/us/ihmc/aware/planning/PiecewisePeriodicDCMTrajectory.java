package us.ihmc.aware.planning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PiecewisePeriodicDCMTrajectory
{
   private boolean initialized;
   private final int numberOfSteps;
   private double gravity;
   private double comHeight;
   private final double[] timeAtSoS;
   private final FramePoint[] dcmPositionAtSoS;
   private final FramePoint[] vrpPositionAtSoS;
   private final FramePoint dcmPosition;
   private final FrameVector dcmVelocity;
   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F x = new DenseMatrix64F(3, 1);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public PiecewisePeriodicDCMTrajectory(int numberOfSteps, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      if (numberOfSteps < 1)
         throw new RuntimeException("numberOfSteps must be greater than 0");

      this.initialized = false;
      this.numberOfSteps = numberOfSteps;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.timeAtSoS = new double[numberOfSteps + 1];
      this.dcmPositionAtSoS = new FramePoint[numberOfSteps + 1];
      this.vrpPositionAtSoS = new FramePoint[numberOfSteps + 1];
      for (int i = 0; i < numberOfSteps + 1; i++)
      {
         this.dcmPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
      }
      this.dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.dcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   /**
    * Computes a piecewise DCM trajectory assuming a constant CMP during each step. Periodicity is enforced by constraining
    * the final DCM position to be equal to the initial DCM position plus the final CMP position minus the initial CMP position.
    *
    * @param timeAtSoS time at the start of each step
    * @param cmpPositionAtSoS centroidal moment pivot position at the start of each step
    * @param relativeYawAtEoS relative yaw angle at end of the final step
    */
   public void initializeTrajectory(double[] timeAtSoS, FramePoint[] cmpPositionAtSoS, double relativeYawAtEoS)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      if ((timeAtSoS.length != numberOfSteps + 1) || (cmpPositionAtSoS.length != numberOfSteps + 1))
      {
         throw new RuntimeException("length of input vector must be equal to the number of steps plus one");
      }

      // compute initial dcm position assuming a periodic gait
      for (int i = 0; i <= numberOfSteps; i++)
      {
         this.timeAtSoS[i] = timeAtSoS[i];
         this.vrpPositionAtSoS[i].setIncludingFrame(cmpPositionAtSoS[i]);
         this.vrpPositionAtSoS[i].changeFrame(worldFrame);
         this.vrpPositionAtSoS[i].add(0, 0, comHeight);
      }

      // A = (R - e^(w(t[n] - t[0])) * I)^-1
      A.zero();
      A.set(0, 0, Math.cos(relativeYawAtEoS));
      A.set(0, 1, -Math.sin(relativeYawAtEoS));
      A.set(1, 0, Math.sin(relativeYawAtEoS));
      A.set(1, 1, Math.cos(relativeYawAtEoS));
      A.set(2, 2, 1);
      for (int i = 0; i < 3; i++)
      {
         A.add(i, i, -Math.exp(naturalFrequency * (timeAtSoS[numberOfSteps] - timeAtSoS[0])));
      }
      CommonOps.invert(A);

      x.zero();
      for (int i = 0; i < numberOfSteps; i++)
      {
         // x = e^(w(t[i + 1] - t[i])) * x + vrp[i] - vrp[i + 1]
         CommonOps.scale(Math.exp(naturalFrequency * (timeAtSoS[i + 1] - timeAtSoS[i])), x);
         x.add(0, 0, this.vrpPositionAtSoS[i].getX());
         x.add(1, 0, this.vrpPositionAtSoS[i].getY());
         x.add(2, 0, this.vrpPositionAtSoS[i].getZ());
         x.add(0, 0, -this.vrpPositionAtSoS[i + 1].getX());
         x.add(1, 0, -this.vrpPositionAtSoS[i + 1].getY());
         x.add(2, 0, -this.vrpPositionAtSoS[i + 1].getZ());
      }

      // vrp[0] = A * x
      CommonOps.mult(A, x, x);
      x.add(0, 0, this.vrpPositionAtSoS[0].getX());
      x.add(1, 0, this.vrpPositionAtSoS[0].getY());
      x.add(2, 0, this.vrpPositionAtSoS[0].getZ());

      this.dcmPositionAtSoS[0].setX(x.get(0, 0));
      this.dcmPositionAtSoS[0].setY(x.get(1, 0));
      this.dcmPositionAtSoS[0].setZ(x.get(2, 0));

      for (int i = 0; i < numberOfSteps; i++)
      {
         this.dcmPositionAtSoS[i + 1].set(this.dcmPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].sub(this.vrpPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].scale(Math.exp(naturalFrequency * (timeAtSoS[i + 1] - timeAtSoS[i])));
         this.dcmPositionAtSoS[i + 1].add(this.vrpPositionAtSoS[i]);
      }
      this.initialized = true;
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between steps
      currentTime = Math.min(Math.max(currentTime, timeAtSoS[0]), timeAtSoS[numberOfSteps]);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      for (int i = 0; i < numberOfSteps; i++)
      {
         if (currentTime <= timeAtSoS[i + 1])
         {
            dcmPosition.set(dcmPositionAtSoS[i]);
            dcmPosition.sub(vrpPositionAtSoS[i]);
            dcmPosition.scale(Math.exp(naturalFrequency * (currentTime - timeAtSoS[i])));
            dcmPosition.add(vrpPositionAtSoS[i]);
            dcmVelocity.set(dcmPosition);
            if (currentTime == timeAtSoS[i + 1])
               dcmVelocity.sub(vrpPositionAtSoS[i + 1]);
            else
               dcmVelocity.sub(vrpPositionAtSoS[i]);
            dcmVelocity.scale(naturalFrequency);
            break;
         }
      }
   }

   public void getPosition(FramePoint dcmPosition)
   {
      dcmPosition.setIncludingFrame(this.dcmPosition);
   }

   public void getVelocity(FrameVector dcmVelocity)
   {
      dcmVelocity.setIncludingFrame(this.dcmVelocity);
   }

   public static void main(String args[])
   {
      double comHeight = 1.0;
      double gravity = 9.81;
      PiecewisePeriodicDCMTrajectory dcmTrajectory = new PiecewisePeriodicDCMTrajectory(2, gravity, comHeight, null);

      FramePoint[] cmpPosition = new FramePoint[3];
      cmpPosition[0] = new FramePoint(ReferenceFrame.getWorldFrame());
      cmpPosition[1] = new FramePoint(ReferenceFrame.getWorldFrame());
      cmpPosition[2] = new FramePoint(ReferenceFrame.getWorldFrame());
      cmpPosition[0].set(0, 0, 0);
      cmpPosition[1].set(0, -0.4, 0);
      cmpPosition[2].set(0, -0.2, 0);
      double[] time = new double[] {0.0, 0.4, 0.8};
      double relativeYaw = 0.0;

      dcmTrajectory.initializeTrajectory(time, cmpPosition, relativeYaw);

      FramePoint dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      FrameVector dcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < time.length; i++)
      {
         dcmTrajectory.computeTrajectory(time[i]);
         dcmTrajectory.getPosition(dcmPosition);
         dcmTrajectory.getVelocity(dcmVelocity);
         dcmPosition.sub(cmpPosition[i]);
         System.out.println("dcm-cmp offset at step " + i + " : " + dcmPosition);
      }
   }
}

