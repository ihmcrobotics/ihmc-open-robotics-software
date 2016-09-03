package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class PiecewiseReverseDcmTrajectory
{
   private boolean initialized;
   private final int maxSteps;
   private int numSteps;
   private double gravity;
   private double comHeight;
   private final double[] timeAtSoS;
   private final FramePoint[] dcmPositionAtSoS;
   private final FramePoint[] vrpPositionAtSoS;
   private final FramePoint dcmPosition;
   private final FrameVector dcmVelocity;
   private final List<MutableDouble> temporaryDouble;
   private final List<FramePoint> temporaryFramePoint;

   public PiecewiseReverseDcmTrajectory(int maxSteps, double gravity, double comHeight)
   {
      if (maxSteps < 1)
         throw new RuntimeException("maxSteps must be greater than 0");

      this.initialized = false;
      this.maxSteps = maxSteps;
      this.numSteps = maxSteps;
      this.gravity = gravity;
      this.comHeight = Math.max(comHeight, 0.001);
      this.timeAtSoS = new double[maxSteps + 1];
      this.dcmPositionAtSoS = new FramePoint[maxSteps + 1];
      this.vrpPositionAtSoS = new FramePoint[maxSteps + 1];
      for (int i = 0; i < maxSteps + 1; i++)
      {
         this.dcmPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
      }
      this.dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.dcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      this.temporaryDouble = new ArrayList<>();
      this.temporaryDouble.add(new MutableDouble(0));
      this.temporaryFramePoint = new ArrayList<>();
      this.temporaryFramePoint.add(new FramePoint());
   }

   /**
    * Computes a piecewise DCM trajectory assuming a constant CMP during each step. The DCM dynamics
    * are integrated in reverse time given a desired final DCM position at the end of the final step.
    *
    * @param numSteps number of steps
    * @param timeAtSoS time at the start of each step
    * @param cmpPositionAtSoS centroidal moment pivot position at the start of each step
    * @param timeAtEoS time at the end of the final step
    * @param dcmPositionAtEoS divergent component of motion position at the end of the final step
    */
   public void initializeTrajectory(int numSteps, List<MutableDouble> timeAtSoS, List<FramePoint> cmpPositionAtSoS, double timeAtEoS, FramePoint dcmPositionAtEoS)
   {
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      if ((maxSteps < numSteps) || (timeAtSoS.size() < numSteps) || (cmpPositionAtSoS.size() < numSteps))
      {
         throw new RuntimeException("number of steps exceeds the maximum buffer size");
      }
      this.numSteps = numSteps;

      // compute dcm position at start of each step assuming a piecewise constant vrp trajectory
      for (int i = 0; i < numSteps; i++)
      {
         this.timeAtSoS[i] = timeAtSoS.get(i).getValue();
         this.vrpPositionAtSoS[i].setIncludingFrame(cmpPositionAtSoS.get(i));
         this.vrpPositionAtSoS[i].changeFrame(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i].add(0, 0, comHeight);
      }
      this.timeAtSoS[numSteps] = timeAtEoS;

      this.dcmPositionAtSoS[numSteps].setIncludingFrame(dcmPositionAtEoS);
      this.dcmPositionAtSoS[numSteps].changeFrame(ReferenceFrame.getWorldFrame());
      for (int i = numSteps - 1; i >= 0; i--)
      {
         this.dcmPositionAtSoS[i].set(this.dcmPositionAtSoS[i + 1]);
         this.dcmPositionAtSoS[i].sub(this.vrpPositionAtSoS[i]);
         this.dcmPositionAtSoS[i].scale(Math.exp(-naturalFrequency * (this.timeAtSoS[i + 1] - this.timeAtSoS[i])));
         this.dcmPositionAtSoS[i].add(this.vrpPositionAtSoS[i]);
      }
      this.initialized = true;
      computeTrajectory(this.timeAtSoS[0]);
   }

   public void initializeTrajectory(double timeAtSoS, FramePoint cmpPositionAtSoS, double timeAtEoS, FramePoint dcmPositionAtEoS)
   {
      this.temporaryDouble.get(0).setValue(timeAtSoS);
      this.temporaryFramePoint.get(0).setIncludingFrame(cmpPositionAtSoS);
      this.initializeTrajectory(1, temporaryDouble, temporaryFramePoint, timeAtEoS, dcmPositionAtEoS);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between steps
      currentTime = Math.min(Math.max(currentTime, timeAtSoS[0]), timeAtSoS[numSteps]);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      for (int i = numSteps - 1; i >= 0; i--)
      {
         if (currentTime >= timeAtSoS[i])
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
      PiecewiseReverseDcmTrajectory dcmTrajectory = new PiecewiseReverseDcmTrajectory(10, gravity, comHeight);

      List<MutableDouble> timeAtSoS = new ArrayList(2);
      timeAtSoS.add(0, new MutableDouble(0.0));
      timeAtSoS.add(1, new MutableDouble(0.4));
      List<FramePoint> cmpPositionAtSoS = new ArrayList<>(2);
      cmpPositionAtSoS.add(0, new FramePoint());
      cmpPositionAtSoS.add(1, new FramePoint());
      cmpPositionAtSoS.get(0).set(0.0, 0.0, 0.0);
      cmpPositionAtSoS.get(1).set(0.0, -0.4, 0.0);

      double timeAtEoS = 0.8;
      FramePoint dcmPositionAtEoS = new FramePoint(ReferenceFrame.getWorldFrame());
      dcmPositionAtEoS.set(0.0, -0.2, comHeight);
      dcmTrajectory.initializeTrajectory(2, timeAtSoS, cmpPositionAtSoS, timeAtEoS, dcmPositionAtEoS);

      FramePoint dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < timeAtSoS.size(); i++)
      {
         dcmTrajectory.computeTrajectory(timeAtSoS.get(i).getValue());
         dcmTrajectory.getPosition(dcmPosition);
         System.out.println("dcm position at start of step " + i + " : " + dcmPosition);
      }
   }
}

