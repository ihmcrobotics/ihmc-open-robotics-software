package us.ihmc.aware.planning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TwoBeatPeriodicDcmTrajectory
{
   private double gravity;
   private double comHeight;
   private boolean initialized;
   private double beat0Time;
   private double beat1Time;
   private double beat2Time;
   private final FramePoint beat0DcmPosition;
   private final FramePoint beat1DcmPosition;
   private final FramePoint beat2DcmPosition;
   private final FramePoint beat0VrpPosition;
   private final FramePoint beat1VrpPosition;
   private final FramePoint beat2VrpPosition;
   private final FramePoint currentDcmPosition;
   private final FrameVector currentDcmVelocity;
   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F x = new DenseMatrix64F(3, 1);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public TwoBeatPeriodicDcmTrajectory(double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.initialized = false;
      this.beat0Time = 0.0;
      this.beat1Time = 0.0;
      this.beat2Time = 0.0;
      this.beat0DcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.beat1DcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.beat2DcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.beat0VrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.beat1VrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.beat2VrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.currentDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.currentDcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void initializeTrajectory(double beat0Time, double beat1Time, double beat2Time, FramePoint beat0VrpPosition, FramePoint beat1VrpPosition, FramePoint beat2VrpPosition, double relativeYaw)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      // compute initial dcm position assuming a periodic one-beat gait
      this.beat0Time = beat0Time;
      this.beat0VrpPosition.setIncludingFrame(beat0VrpPosition);
      this.beat0VrpPosition.changeFrame(worldFrame);
      this.beat1Time = beat1Time;
      this.beat1VrpPosition.setIncludingFrame(beat1VrpPosition);
      this.beat1VrpPosition.changeFrame(worldFrame);
      this.beat2Time = beat2Time;
      this.beat2VrpPosition.setIncludingFrame(beat2VrpPosition);
      this.beat2VrpPosition.changeFrame(worldFrame);

      // A = (R - e^(w(t2 - t0)) * I)^-1
      A.zero();
      A.set(0, 0, Math.cos(relativeYaw)); A.set(0, 1,-Math.sin(relativeYaw));
      A.set(1, 0, Math.sin(relativeYaw)); A.set(1, 1, Math.cos(relativeYaw));
      A.set(2, 2, 1);
      for (int i = 0; i < 3; i++)
      {
         A.add(i, i, -Math.exp(naturalFrequency * (beat2Time - beat0Time)));
      }
      CommonOps.invert(A);

      // dcm0 = vrp0 + A * (vrp1 - vrp2 + e^(w(t1 - t0)) * (vrp0 - vrp1))
      x.set(0, 0, this.beat0VrpPosition.getX());
      x.set(1, 0, this.beat0VrpPosition.getY());
      x.set(2, 0, this.beat0VrpPosition.getZ());
      x.add(0, 0, -this.beat1VrpPosition.getX());
      x.add(1, 0, -this.beat1VrpPosition.getY());
      x.add(2, 0, -this.beat1VrpPosition.getZ());
      CommonOps.scale(Math.exp(naturalFrequency * (beat1Time - beat0Time)), x);
      x.add(0, 0, this.beat1VrpPosition.getX());
      x.add(1, 0, this.beat1VrpPosition.getY());
      x.add(2, 0, this.beat1VrpPosition.getZ());
      x.add(0, 0, -this.beat2VrpPosition.getX());
      x.add(1, 0, -this.beat2VrpPosition.getY());
      x.add(2, 0, -this.beat2VrpPosition.getZ());
      CommonOps.mult(A, x, x);
      x.add(0, 0, this.beat0VrpPosition.getX());
      x.add(1, 0, this.beat0VrpPosition.getY());
      x.add(2, 0, this.beat0VrpPosition.getZ());
      this.beat0DcmPosition.setX(x.get(0, 0));
      this.beat0DcmPosition.setY(x.get(1, 0));
      this.beat0DcmPosition.setZ(x.get(2, 0));

      this.beat1DcmPosition.set(beat0DcmPosition);
      this.beat1DcmPosition.sub(beat0VrpPosition);
      this.beat1DcmPosition.scale(Math.exp(naturalFrequency * (beat1Time - beat0Time)));
      this.beat1DcmPosition.add(beat0VrpPosition);

      this.beat2DcmPosition.set(beat1DcmPosition);
      this.beat2DcmPosition.sub(beat1VrpPosition);
      this.beat2DcmPosition.scale(Math.exp(naturalFrequency * (beat2Time - beat1Time)));
      this.beat2DcmPosition.add(beat1VrpPosition);
      this.initialized = true;
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between beats
      currentTime = Math.min(Math.max(currentTime, beat0Time), beat2Time);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      if (currentTime < beat1Time)
      {
         currentDcmPosition.set(beat0DcmPosition);
         currentDcmPosition.sub(beat0VrpPosition);
         currentDcmPosition.scale(Math.exp(naturalFrequency * (currentTime - beat0Time)));
         currentDcmPosition.add(beat0VrpPosition);
         currentDcmVelocity.set(currentDcmPosition);
         currentDcmVelocity.sub(beat0VrpPosition);
         currentDcmVelocity.scale(naturalFrequency);
      }
      else
      {
         currentDcmPosition.set(beat1DcmPosition);
         currentDcmPosition.sub(beat1VrpPosition);
         currentDcmPosition.scale(Math.exp(naturalFrequency * (currentTime - beat1Time)));
         currentDcmPosition.add(beat1VrpPosition);
         currentDcmVelocity.set(currentDcmPosition);
         currentDcmVelocity.sub(beat1VrpPosition);
         currentDcmVelocity.scale(naturalFrequency);
      }
   }

   public void getPosition(FramePoint currentDcmPosition)
   {
      currentDcmPosition.setIncludingFrame(this.currentDcmPosition);
   }

   public void getVelocity(FrameVector currentDcmVelocity)
   {
      currentDcmVelocity.setIncludingFrame(this.currentDcmVelocity);
   }

   public static void main(String args[])
   {
      double comHeight = 1.0;
      double gravity = 9.81;
      TwoBeatPeriodicDcmTrajectory dcmTrajectory = new TwoBeatPeriodicDcmTrajectory(gravity, comHeight, null);

      FramePoint vrpPosition0 = new FramePoint(ReferenceFrame.getWorldFrame());
      FramePoint vrpPosition1 = new FramePoint(ReferenceFrame.getWorldFrame());
      FramePoint vrpPosition2 = new FramePoint(ReferenceFrame.getWorldFrame());
      vrpPosition0.set(0,  0, comHeight);
      vrpPosition1.set(0, -0.4, comHeight);
      vrpPosition2.set(0, -0.2, comHeight);
      double time0 = 0.0;
      double time1 = 0.4;
      double time2 = 0.8;
      double relativeYaw = 0.0;

      dcmTrajectory.initializeTrajectory(time0, time1, time2, vrpPosition0, vrpPosition1, vrpPosition2, relativeYaw);

      FramePoint dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      dcmTrajectory.computeTrajectory(time0);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(vrpPosition0);
      System.out.println("initial dcm offset      : " + dcmPosition);

      dcmTrajectory.computeTrajectory(time1);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(vrpPosition1);
      System.out.println("intermediate dcm offset : " + dcmPosition);

      dcmTrajectory.computeTrajectory(time2);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(vrpPosition2);
      System.out.println("final dcm offset        : " + dcmPosition);
   }
}

