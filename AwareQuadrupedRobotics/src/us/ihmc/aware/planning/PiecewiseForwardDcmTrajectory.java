package us.ihmc.aware.planning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PiecewiseForwardDcmTrajectory
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
   private final double[] temporaryDouble;
   private final FramePoint[] temporaryFramePoint;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public PiecewiseForwardDcmTrajectory(int numberOfSteps, double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      if (numberOfSteps < 1)
         throw new RuntimeException("numberOfSteps must be greater than 0");

      this.initialized = false;
      this.numberOfSteps = numberOfSteps;
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.timeAtSoS = new double[numberOfSteps];
      this.dcmPositionAtSoS = new FramePoint[numberOfSteps];
      this.vrpPositionAtSoS = new FramePoint[numberOfSteps];
      for (int i = 0; i < numberOfSteps; i++)
      {
         this.dcmPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         this.vrpPositionAtSoS[i] = new FramePoint(ReferenceFrame.getWorldFrame());
      }
      this.dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.dcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      this.temporaryDouble = new double[] {0.0};
      this.temporaryFramePoint = new FramePoint[] {new FramePoint(ReferenceFrame.getWorldFrame())};

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
    * @param dcmPositionAtSoS divergent component of motion position at the start of the first step
    */
   public void initializeTrajectory(double[] timeAtSoS, FramePoint[] cmpPositionAtSoS, FramePoint dcmPositionAtSoS)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      if ((timeAtSoS.length != numberOfSteps) || (cmpPositionAtSoS.length != numberOfSteps))
      {
         throw new RuntimeException("length of input vector must be equal to the number of steps");
      }

      // compute dcm position at start of each step assuming a piecewise constant vrp trajectory
      for (int i = 0; i < numberOfSteps; i++)
      {
         this.timeAtSoS[i] = timeAtSoS[i];
         this.vrpPositionAtSoS[i].setIncludingFrame(cmpPositionAtSoS[i]);
         this.vrpPositionAtSoS[i].changeFrame(worldFrame);
         this.vrpPositionAtSoS[i].add(0, 0, comHeight);
      }

      this.dcmPositionAtSoS[0].setIncludingFrame(dcmPositionAtSoS);
      this.dcmPositionAtSoS[0].changeFrame(worldFrame);
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         this.dcmPositionAtSoS[i + 1].set(this.dcmPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].sub(this.vrpPositionAtSoS[i]);
         this.dcmPositionAtSoS[i + 1].scale(Math.exp(naturalFrequency * (this.timeAtSoS[i + 1] - this.timeAtSoS[i])));
         this.dcmPositionAtSoS[i + 1].add(this.vrpPositionAtSoS[i]);
      }
      this.initialized = true;
   }

   public void initializeTrajectory(double timeAtSoS, FramePoint cmpPositionAtSoS, FramePoint dcmPositionAtSoS)
   {
      this.temporaryDouble[0] = timeAtSoS;
      this.temporaryFramePoint[0].setIncludingFrame(cmpPositionAtSoS);
      this.initializeTrajectory(temporaryDouble, temporaryFramePoint, dcmPositionAtSoS);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between steps
      currentTime = Math.max(currentTime, timeAtSoS[0]);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      for (int i = numberOfSteps - 1; i >= 0; i--)
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
      this.comHeight = comHeight;
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
      PiecewiseForwardDcmTrajectory dcmTrajectory = new PiecewiseForwardDcmTrajectory(2, gravity, comHeight, null);

      double[] timeAtSoS = new double[] {0.0, 0.4};
      FramePoint[] cmpPositionAtSoS = new FramePoint[2];
      cmpPositionAtSoS[0] = new FramePoint(ReferenceFrame.getWorldFrame());
      cmpPositionAtSoS[1] = new FramePoint(ReferenceFrame.getWorldFrame());
      cmpPositionAtSoS[0].set(0.0, 0.0, 0.0);
      cmpPositionAtSoS[1].set(0.0,-0.4, 0.0);

      FramePoint dcmPositionAtSoS = new FramePoint(ReferenceFrame.getWorldFrame());
      dcmPositionAtSoS.set(0.0, -0.05, comHeight);
      dcmTrajectory.initializeTrajectory(timeAtSoS, cmpPositionAtSoS, dcmPositionAtSoS);

      FramePoint dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < timeAtSoS.length; i++)
      {
         dcmTrajectory.computeTrajectory(timeAtSoS[i]);
         dcmTrajectory.getPosition(dcmPosition);
         System.out.println("dcm position at start of step " + i + " : " + dcmPosition);
      }
   }
}

