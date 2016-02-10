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
   /* the time it takes for the DCM to converge to the center of the final support polygon following the swing phase */
   private final static double LOAD_DURATION = 0.5;

   /* the normalized distance the DCM diverges from the center of the initial support polygon during the swing phase (0, 1) */
   private final static double SWING_PHASE_DIVERGENCE = 0.33;

   private double gravity;
   private double comHeight;
   private boolean stepInitialized;
   private final QuadrupedSupportPolygon supportPolygon;

   private double initialTime;
   private double liftOffTime;
   private double touchDownTime;
   private double finalTime;
   private final FramePoint swingVRPPosition;
   private final FramePoint initialDCMPosition;
   private final FrameVector initialDCMVelocity;
   private final FramePoint liftOffDCMPosition;
   private final FramePoint touchDownDCMPosition;
   private final FrameVector touchDownDCMVelocity;
   private final FramePoint finalDCMPosition;
   private final FramePoint currentDCMPosition;
   private final FrameVector currentDCMVelocity;

   private final YoPolynomial xUnloadTrajectory;
   private final YoPolynomial yUnloadTrajectory;
   private final YoPolynomial zUnloadTrajectory;
   private final YoPolynomial xLoadTrajectory;
   private final YoPolynomial yLoadTrajectory;
   private final YoPolynomial zLoadTrajectory;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SingleStepDCMTrajectory(double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.stepInitialized = false;
      this.supportPolygon = new QuadrupedSupportPolygon();

      initialTime = 0.0;
      liftOffTime = 0.0;
      touchDownTime = 0.0;
      finalTime = 0.0;
      swingVRPPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialDCMVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      liftOffDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      touchDownDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      touchDownDCMVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      finalDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDCMPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDCMVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

      xUnloadTrajectory = new YoPolynomial("xUnloadTrajectory", 4, registry);
      yUnloadTrajectory = new YoPolynomial("yUnloadTrajectory", 4, registry);
      zUnloadTrajectory = new YoPolynomial("zUnloadTrajectory", 4, registry);
      xLoadTrajectory = new YoPolynomial("xLoadTrajectory", 4, registry);
      yLoadTrajectory = new YoPolynomial("yLoadTrajectory", 4, registry);
      zLoadTrajectory = new YoPolynomial("zLoadTrajectory", 4, registry);

      parentRegistry.addChild(registry);
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }

   public void setStepPameters(double initialTime, FramePoint initialDCMPosition, FrameVector initialDCMVelocity, QuadrupedSupportPolygon supportPolygon, QuadrupedTimedStep quadrupedStep)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      this.stepInitialized = true;
      this.supportPolygon.set(supportPolygon);
      this.supportPolygon.changeFrame(worldFrame);

      this.initialTime = initialTime;
      this.liftOffTime = quadrupedStep.getTimeInterval().getStartTime();
      this.touchDownTime = quadrupedStep.getTimeInterval().getEndTime();
      this.finalTime = this.touchDownTime + LOAD_DURATION;

      // compute VRP position in swing phase
      FramePoint acrossBodyFootPosition = supportPolygon.getFootstep(quadrupedStep.getRobotQuadrant().getAcrossBodyQuadrant());
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), acrossBodyFootPosition);
      this.supportPolygon.getCentroid2d(swingVRPPosition);
      this.swingVRPPosition.add(0, 0, comHeight);

      // compute DCM position in final support phase
      quadrupedStep.getGoalPosition().changeFrame(worldFrame);
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), quadrupedStep.getGoalPosition());
      this.supportPolygon.getCentroid2d(finalDCMPosition);
      this.finalDCMPosition.add(0, 0, comHeight);

      // compute DCM position and velocity at touch down
      this.touchDownDCMPosition.set(this.finalDCMPosition);
      this.touchDownDCMPosition.sub(this.swingVRPPosition);
      this.touchDownDCMPosition.scale(SWING_PHASE_DIVERGENCE);
      this.touchDownDCMPosition.add(this.swingVRPPosition);
      this.touchDownDCMVelocity.set(this.touchDownDCMPosition);
      this.touchDownDCMVelocity.sub(this.swingVRPPosition);
      this.touchDownDCMVelocity.scale(naturalFrequency);

      // compute DCM position at lift off
      this.liftOffDCMPosition.set(touchDownDCMPosition);
      this.liftOffDCMPosition.sub(swingVRPPosition);
      this.liftOffDCMPosition.scale(Math.exp(-naturalFrequency * (touchDownTime - liftOffTime)));
      this.liftOffDCMPosition.add(swingVRPPosition);

      // compute DCM position in initial support phase
      this.initialDCMPosition.setIncludingFrame(initialDCMPosition);
      this.initialDCMVelocity.setIncludingFrame(initialDCMVelocity);
      this.initialDCMPosition.changeFrame(worldFrame);
      this.initialDCMVelocity.changeFrame(worldFrame);

      // compute polynomial trajectory to interpolate between initial and liftOff DCM state
      this.xUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getX(), initialDCMVelocity.getX(), liftOffDCMPosition.getX(), 0.0);
      this.yUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getY(), initialDCMVelocity.getY(), liftOffDCMPosition.getY(), 0.0);
      this.zUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDCMPosition.getZ(), initialDCMVelocity.getZ(), liftOffDCMPosition.getZ(), 0.0);

      // compute polynomial trajectory to interpolate between touchdown and final DCM state
      this.xLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDCMPosition.getX(), touchDownDCMVelocity.getX(), finalDCMPosition.getX(), 0.0);
      this.yLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDCMPosition.getY(), touchDownDCMVelocity.getY(), finalDCMPosition.getY(), 0.0);
      this.zLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDCMPosition.getZ(), touchDownDCMVelocity.getZ(), finalDCMPosition.getZ(), 0.0);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!stepInitialized)
         throw new RuntimeException("step must be initialized before computing trajectory");

      // compute trajectory
      currentTime = Math.min(Math.max(currentTime, initialTime), finalTime);
      if (currentTime < liftOffTime)
      {
         // cubic spline trajectory prior to lift off
         xUnloadTrajectory.compute(currentTime);
         yUnloadTrajectory.compute(currentTime);
         zUnloadTrajectory.compute(currentTime);
         currentDCMPosition.set(xUnloadTrajectory.getPosition(), yUnloadTrajectory.getPosition(), zUnloadTrajectory.getPosition());
         currentDCMVelocity.set(xUnloadTrajectory.getVelocity(), yUnloadTrajectory.getVelocity(), zUnloadTrajectory.getVelocity());
      }
      else if (currentTime < touchDownTime)
      {
         // constant virtual repellent point trajectory during swing phase
         double naturalFrequency = Math.sqrt(gravity / comHeight);
         currentDCMPosition.set(liftOffDCMPosition);
         currentDCMPosition.sub(swingVRPPosition);
         currentDCMPosition.scale(Math.exp(naturalFrequency * (currentTime - liftOffTime)));
         currentDCMPosition.add(swingVRPPosition);
         currentDCMVelocity.set(currentDCMPosition);
         currentDCMVelocity.sub(swingVRPPosition);
         currentDCMVelocity.scale(naturalFrequency);
      }
      else
      {
         // cubic spline trajectory following touch down
         xLoadTrajectory.compute(currentTime);
         yLoadTrajectory.compute(currentTime);
         zLoadTrajectory.compute(currentTime);
         currentDCMPosition.set(xLoadTrajectory.getPosition(), yLoadTrajectory.getPosition(), zLoadTrajectory.getPosition());
         currentDCMVelocity.set(xLoadTrajectory.getVelocity(), yLoadTrajectory.getVelocity(), zLoadTrajectory.getVelocity());
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
