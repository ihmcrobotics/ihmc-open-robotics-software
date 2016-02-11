package us.ihmc.aware.planning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class OneBeatPeriodicDcmTrajectory
{
   private double gravity;
   private double comHeight;
   private boolean initialized;
   private double initialTime;
   private double finalTime;
   private final FramePoint initialVrpPosition;
   private final FramePoint initialDcmPosition;
   private final FramePoint currentDcmPosition;
   private final FrameVector currentDcmVelocity;
   private final FramePoint finalVrpPosition;
   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F x = new DenseMatrix64F(3, 1);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public OneBeatPeriodicDcmTrajectory(double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.initialized = false;
      this.initialTime = 0.0;
      this.finalTime = 0.0;
      this.initialVrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.initialDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.currentDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      this.currentDcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      this.finalVrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void initializeTrajectory(double initialTime, double finalTime, FramePoint initialVrpPosition, FramePoint finalVrpPosition, double relativeYaw)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      // compute initial dcm position assuming a periodic one-beat gait
      this.initialTime = initialTime;
      this.initialVrpPosition.setIncludingFrame(initialVrpPosition);
      this.initialVrpPosition.changeFrame(worldFrame);
      this.finalTime = finalTime;
      this.finalVrpPosition.setIncludingFrame(finalVrpPosition);
      this.finalVrpPosition.changeFrame(worldFrame);

      // A = (R - e^wt * I)^-1
      A.zero();
      A.set(0, 0, Math.cos(relativeYaw)); A.set(0, 1,-Math.sin(relativeYaw));
      A.set(1, 0, Math.sin(relativeYaw)); A.set(1, 1, Math.cos(relativeYaw));
      A.set(2, 2, 1);
      for (int i = 0; i < 3; i++)
      {
         A.add(i, i, -Math.exp(naturalFrequency * (finalTime - initialTime)));
      }
      CommonOps.invert(A);

      // dcm_i = vrp_i + A * (vrp_i - vrp_f)
      x.set(0, 0, this.initialVrpPosition.getX());
      x.set(1, 0, this.initialVrpPosition.getY());
      x.set(2, 0, this.initialVrpPosition.getZ());
      x.add(0, 0, -this.finalVrpPosition.getX());
      x.add(1, 0, -this.finalVrpPosition.getY());
      x.add(2, 0, -this.finalVrpPosition.getZ());
      CommonOps.mult(A, x, x);
      x.add(0, 0, this.initialVrpPosition.getX());
      x.add(1, 0, this.initialVrpPosition.getY());
      x.add(2, 0, this.initialVrpPosition.getZ());
      this.initialDcmPosition.setX(x.get(0, 0));
      this.initialDcmPosition.setY(x.get(1, 0));
      this.initialDcmPosition.setZ(x.get(2, 0));
      this.initialized = true;
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute constant virtual repellent point trajectory between beats
      currentTime = Math.min(Math.max(currentTime, initialTime), finalTime);
      double naturalFrequency = Math.sqrt(gravity / comHeight);
      currentDcmPosition.set(initialDcmPosition);
      currentDcmPosition.sub(initialVrpPosition);
      currentDcmPosition.scale(Math.exp(naturalFrequency * (currentTime - initialTime)));
      currentDcmPosition.add(initialVrpPosition);
      currentDcmVelocity.set(currentDcmPosition);
      currentDcmVelocity.sub(initialVrpPosition);
      currentDcmVelocity.scale(naturalFrequency);
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
      OneBeatPeriodicDcmTrajectory dcmTrajectory = new OneBeatPeriodicDcmTrajectory(gravity, comHeight, null);

      FramePoint vrpPosition1 = new FramePoint(ReferenceFrame.getWorldFrame());
      FramePoint vrpPosition2 = new FramePoint(ReferenceFrame.getWorldFrame());
      vrpPosition1.set(0, 0, comHeight);
      vrpPosition2.set(0.3, 0.1, comHeight);
      double time1 = 0.0;
      double time2 = 0.4;
      double relativeYaw = 0.0;

      dcmTrajectory.initializeTrajectory(time1, time2, vrpPosition1, vrpPosition2, relativeYaw);

      FramePoint dcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      dcmTrajectory.computeTrajectory(time1);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(vrpPosition1);
      System.out.println("initial dcm offset : " + dcmPosition);

      dcmTrajectory.computeTrajectory(time2);
      dcmTrajectory.getPosition(dcmPosition);
      dcmPosition.sub(vrpPosition2);
      System.out.println("final dcm offset   : " + dcmPosition);
   }
}

