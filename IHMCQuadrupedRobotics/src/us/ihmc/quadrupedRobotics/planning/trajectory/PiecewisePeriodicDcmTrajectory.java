package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class PiecewisePeriodicDcmTrajectory
{
   private boolean initialized;
   private final int maxSteps;
   private int numSteps;
   private double gravity;
   private double comHeight;
   private final double[] timeAtSoS;
   private final FramePoint3D[] dcmPositionAtSoS;
   private final FramePoint3D[] vrpPositionAtSoS;
   private final FramePoint3D dcmPosition;
   private final FrameVector3D dcmVelocity;
   private final List<MutableDouble> temporaryDouble;
   private final List<FramePoint3D> temporaryFramePoint;
   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F x = new DenseMatrix64F(3, 1);

   public PiecewisePeriodicDcmTrajectory(int maxSteps, double gravity, double comHeight)
   {
      if (maxSteps < 1)
         throw new RuntimeException("maxSteps must be greater than 0");

      this.initialized = false;
      this.maxSteps = maxSteps;
      this.gravity = gravity;
      this.comHeight = Math.max(comHeight, 0.001);
      this.timeAtSoS = new double[maxSteps + 1];
      this.dcmPositionAtSoS = new FramePoint3D[maxSteps + 1];
      this.vrpPositionAtSoS = new FramePoint3D[maxSteps + 1];
      for (int i = 0; i < maxSteps + 1; i++)
      {
         this.dcmPositionAtSoS[i] = new FramePoint3D(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i] = new FramePoint3D(ReferenceFrame.getWorldFrame());
      }
      this.dcmPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
      this.dcmVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
      this.temporaryDouble = new ArrayList<>();
      this.temporaryDouble.add(new MutableDouble(0));
      this.temporaryFramePoint = new ArrayList<>();
      this.temporaryFramePoint.add(new FramePoint3D());
   }

   /**
    * Computes a piecewise DCM trajectory assuming a constant CMP during each step. Periodicity is enforced by constraining
    * the final DCM position to be equal to the initial DCM position plus the final CMP position minus the initial CMP position.
    *
    * @param numSteps number of steps
    * @param timeAtSoS time at the start of each step
    * @param cmpPositionAtSoS centroidal moment pivot position at the start of each step
    * @param timeAtEoS time at the end of the final step
    * @param cmpPositionAtEoS centroidal moment pivot position at the end of the final step
    * @param relativeYawAtEoS relative yaw angle at end of the final step
    */
   public void initializeTrajectory(int numSteps, List<MutableDouble> timeAtSoS, List<FramePoint3D> cmpPositionAtSoS, double timeAtEoS, FramePoint3D cmpPositionAtEoS,
         double relativeYawAtEoS)
   {
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      if ((maxSteps < numSteps) || (timeAtSoS.size() < numSteps) || (cmpPositionAtSoS.size() < numSteps))
      {
         throw new RuntimeException("number of steps exceeds the maximum buffer size");
      }
      this.numSteps = numSteps;

      // compute initial dcm position assuming a periodic gait
      for (int i = 0; i < numSteps; i++)
      {
         this.timeAtSoS[i] = timeAtSoS.get(i).doubleValue();
         this.vrpPositionAtSoS[i].setIncludingFrame(cmpPositionAtSoS.get(i));
         this.vrpPositionAtSoS[i].changeFrame(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i].add(0, 0, comHeight);
      }
      this.timeAtSoS[numSteps] = timeAtEoS;
      this.vrpPositionAtSoS[numSteps].setIncludingFrame(cmpPositionAtEoS);
      this.vrpPositionAtSoS[numSteps].changeFrame(ReferenceFrame.getWorldFrame());
      this.vrpPositionAtSoS[numSteps].add(0, 0, comHeight);

      // A = (R - e^(w(t[n] - t[0])) * I)^-1
      A.zero();
      A.set(0, 0, Math.cos(relativeYawAtEoS));
      A.set(0, 1, -Math.sin(relativeYawAtEoS));
      A.set(1, 0, Math.sin(relativeYawAtEoS));
      A.set(1, 1, Math.cos(relativeYawAtEoS));
      A.set(2, 2, 1);
      for (int i = 0; i < 3; i++)
      {
         A.add(i, i, -Math.exp(naturalFrequency * (this.timeAtSoS[numSteps] - this.timeAtSoS[0])));
      }
      CommonOps.invert(A);

      x.zero();
      for (int i = 0; i < numSteps; i++)
      {
         // x = e^(w(t[i + 1] - t[i])) * x + vrp[i] - vrp[i + 1]
         CommonOps.scale(Math.exp(naturalFrequency * (this.timeAtSoS[i + 1] - this.timeAtSoS[i])), x);
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

      for (int i = 0; i < numSteps; i++)
      {
         this.dcmPositionAtSoS[i + 1].set(this.dcmPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].sub(this.vrpPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].scale(Math.exp(naturalFrequency * (this.timeAtSoS[i + 1] - this.timeAtSoS[i])));
         this.dcmPositionAtSoS[i + 1].add(this.vrpPositionAtSoS[i]);
      }
      this.initialized = true;
      computeTrajectory(this.timeAtSoS[0]);
   }

   public void initializeTrajectory(double timeAtSoS, FramePoint3D cmpPositionAtSoS, double timeAtEoS, FramePoint3D cmpPositionAtEoS, double relativeYawAtEoS)
   {
      this.temporaryDouble.get(0).setValue(timeAtSoS);
      this.temporaryFramePoint.get(0).setIncludingFrame(cmpPositionAtSoS);
      this.initializeTrajectory(1, temporaryDouble, temporaryFramePoint, timeAtEoS, cmpPositionAtEoS, relativeYawAtEoS);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between steps
      currentTime = Math.min(Math.max(currentTime, timeAtSoS[0]), timeAtSoS[numSteps]);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      for (int i = 0; i < numSteps; i++)
      {
         if (currentTime <= timeAtSoS[i + 1])
         {
            dcmPosition.set(dcmPositionAtSoS[i]);
            dcmPosition.sub(vrpPositionAtSoS[i]);
            dcmPosition.scale(Math.exp(naturalFrequency * (currentTime - timeAtSoS[i])));
            dcmPosition.add(vrpPositionAtSoS[i]);
            dcmVelocity.set(dcmPosition);
            dcmVelocity.sub(vrpPositionAtSoS[i]);
            dcmVelocity.scale(naturalFrequency);
            break;
         }
      }
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = Math.max(comHeight, 0.001);
   }

   public double getStartTime()
   {
      return timeAtSoS[0];
   }

   public void getPosition(FramePoint3D dcmPosition)
   {
      dcmPosition.setIncludingFrame(this.dcmPosition);
   }

   public void getVelocity(FrameVector3D dcmVelocity)
   {
      dcmVelocity.setIncludingFrame(this.dcmVelocity);
   }

   public static void main(String args[])
   {
      double comHeight = 1.0;
      double gravity = 9.81;
      PiecewisePeriodicDcmTrajectory dcmTrajectory = new PiecewisePeriodicDcmTrajectory(10, gravity, comHeight);

      List<MutableDouble> timeAtSoS = new ArrayList<>(2);
      timeAtSoS.add(0, new MutableDouble(0.0));
      timeAtSoS.add(1, new MutableDouble(0.4));
      List<FramePoint3D> cmpPositionAtSoS = new ArrayList<>(2);
      cmpPositionAtSoS.add(0, new FramePoint3D(ReferenceFrame.getWorldFrame()));
      cmpPositionAtSoS.add(1, new FramePoint3D(ReferenceFrame.getWorldFrame()));
      cmpPositionAtSoS.get(0).set(0.0, 0.0, 0.0);
      cmpPositionAtSoS.get(1).set(0.0, -0.4, 0.0);

      double timeAtEoS = 0.8;
      FramePoint3D cmpPositionAtEoS = new FramePoint3D(ReferenceFrame.getWorldFrame());
      cmpPositionAtEoS.set(0.0, -0.2, 0.0);

      dcmTrajectory.initializeTrajectory(2, timeAtSoS, cmpPositionAtSoS, timeAtEoS, cmpPositionAtEoS, 0.0);

      FramePoint3D dcmPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
      dcmTrajectory.computeTrajectory(timeAtSoS.get(0).doubleValue());
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(cmpPositionAtSoS.get(0));
      System.out.println("dcm-cmp offset at start of first step : " + dcmPosition);
      dcmTrajectory.computeTrajectory(timeAtEoS);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(cmpPositionAtEoS);
      System.out.println("dcm-cmp offset at end of final step   : " + dcmPosition);
   }
}

