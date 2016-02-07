package us.ihmc.aware.planning;

import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SingleStepDCMTrajectory
{
   private double gravity;
   private double comHeight;
   private boolean stepInitialized;
   private final QuadrupedSupportPolygon supportPolygon;

   private double initialTime;
   private double liftOffTime;
   private double finalTime;
   private final FramePoint initialDCMPosition;
   private final FrameVector initialDCMVelocity;
   private final FramePoint liftOffVRPPosition;
   private final FramePoint liftOffDCMPosition;
   private final FramePoint finalDCMPosition;
   private final FramePoint currentDCMPosition;
   private final FrameVector currentDCMVelocity;

   private final YoPolynomial xTrajectory;
   private final YoPolynomial yTrajectory;
   private final YoPolynomial zTrajectory;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SingleStepDCMTrajectory(double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.stepInitialized = false;
      this.supportPolygon = new QuadrupedSupportPolygon();

      initialTime = 0.0;
      liftOffTime = 0.0;
      finalTime = 0.0;
      initialDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialDCMVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      liftOffVRPPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      liftOffDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      finalDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDCMVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

      xTrajectory = new YoPolynomial("xTrajectory", 4, registry);
      yTrajectory = new YoPolynomial("yTrajectory", 4, registry);
      zTrajectory = new YoPolynomial("zTrajectory", 4, registry);

      parentRegistry.addChild(registry);
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }

   public void setStepPameters(double initialTime, FramePoint initialDCMPosition, FrameVector initialDCMVelocity, QuadrupedSupportPolygon supportPolygon, QuadrupedTimedStep quadrupedStep)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      this.stepInitialized = true;
      this.supportPolygon.set(supportPolygon);
      this.supportPolygon.changeFrame(worldFrame);

      this.initialTime = initialTime;
      this.liftOffTime = quadrupedStep.getTimeInterval().getStartTime();
      this.finalTime = quadrupedStep.getTimeInterval().getEndTime();

      this.initialDCMPosition.setIncludingFrame(initialDCMPosition);
      this.initialDCMVelocity.setIncludingFrame(initialDCMVelocity);
      this.initialDCMPosition.changeFrame(worldFrame);
      this.initialDCMVelocity.changeFrame(worldFrame);

      // compute CMP position during swing phase
      FramePoint acrossBodyFootPosition = supportPolygon.getFootstep(quadrupedStep.getRobotQuadrant().getAcrossBodyQuadrant());
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), acrossBodyFootPosition);
      this.supportPolygon.getCentroid2d(liftOffVRPPosition);
      this.liftOffVRPPosition.add(0, 0, comHeight);

      // compute DCM position at time of touch down
      quadrupedStep.getGoalPosition().changeFrame(worldFrame);
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), quadrupedStep.getGoalPosition());
      this.supportPolygon.getCentroid2d(finalDCMPosition);
      this.finalDCMPosition.add(0, 0, comHeight);

      // compute DCM position at time of heel strike
      this.liftOffDCMPosition.set(finalDCMPosition);
      this.liftOffDCMPosition.sub(liftOffVRPPosition);
      this.liftOffDCMPosition.scale(Math.exp(-Math.sqrt(gravity / comHeight) * (finalTime - liftOffTime)));
      this.liftOffDCMPosition.add(liftOffVRPPosition);

      // compute polynomial trajectory to interpolate between initial and liftOff DCM state
      this.xTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getX(), initialDCMVelocity.getX(), liftOffDCMPosition.getX(), 0.0);
      this.yTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getY(), initialDCMVelocity.getY(), liftOffDCMPosition.getY(), 0.0);
      this.zTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getZ(), initialDCMVelocity.getZ(), liftOffDCMPosition.getZ(), 0.0);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!stepInitialized)
         throw new RuntimeException("step must be initialized before computing trajectory");

      // compute trajectory
      currentTime = Math.min(Math.max(currentTime, initialTime), finalTime);
      if (currentTime < liftOffTime)
      {
         // cubic spline trajectory during support phase
         xTrajectory.compute(currentTime);
         yTrajectory.compute(currentTime);
         zTrajectory.compute(currentTime);
         currentDCMPosition.set(xTrajectory.getPosition(), yTrajectory.getPosition(), zTrajectory.getPosition());
         currentDCMVelocity.set(xTrajectory.getVelocity(), yTrajectory.getVelocity(), zTrajectory.getVelocity());
      }
      else
      {
         // constant virtual repellent point trajectory during swing phase
         double naturalFrequency = Math.sqrt(gravity / comHeight);
         currentDCMPosition.set(liftOffDCMPosition);
         currentDCMPosition.sub(liftOffVRPPosition);
         currentDCMPosition.scale(Math.exp(naturalFrequency * (currentTime - liftOffTime)));
         currentDCMPosition.add(liftOffVRPPosition);
         currentDCMVelocity.set(currentDCMPosition);
         currentDCMVelocity.sub(liftOffVRPPosition);
         currentDCMVelocity.scale(naturalFrequency);
      }
   }

   public void getPosition(FramePoint currentDCMPosition)
   {
      currentDCMPosition.setIncludingFrame(this.currentDCMPosition);
   }

   public void getVelocity(FrameVector currentDCMVelocity)
   {
      currentDCMVelocity.setIncludingFrame(this.currentDCMVelocity);
   }
}
