package us.ihmc.aware.planning;

import us.ihmc.aware.util.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SingleStepDcmTrajectory
{
   /* the time it takes for the DCM to converge to the center of the final support polygon following the swing phase */
   private final static double LOAD_DURATION = 0.5;

   /* the normalized distance the DCM diverges from the center of the initial support polygon during the swing phase (0, 1) */
   private final static double SWING_PHASE_DIVERGENCE = 0.33;

   private double gravity;
   private double comHeight;
   private boolean initialized;
   private final QuadrupedSupportPolygon supportPolygon;

   private double initialTime;
   private double liftOffTime;
   private double touchDownTime;
   private double finalTime;
   private final FramePoint swingVrpPosition;
   private final FramePoint initialDcmPosition;
   private final FrameVector initialDcmVelocity;
   private final FramePoint liftOffDcmPosition;
   private final FramePoint touchDownDcmPosition;
   private final FrameVector touchDownDcmVelocity;
   private final FramePoint finalDcmPosition;
   private final FramePoint currentDcmPosition;
   private final FrameVector currentDcmVelocity;

   private final YoPolynomial xUnloadTrajectory;
   private final YoPolynomial yUnloadTrajectory;
   private final YoPolynomial zUnloadTrajectory;
   private final YoPolynomial xLoadTrajectory;
   private final YoPolynomial yLoadTrajectory;
   private final YoPolynomial zLoadTrajectory;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SingleStepDcmTrajectory(double gravity, double comHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      this.comHeight = comHeight;
      this.initialized = false;
      this.supportPolygon = new QuadrupedSupportPolygon();

      initialTime = 0.0;
      liftOffTime = 0.0;
      touchDownTime = 0.0;
      finalTime = 0.0;
      swingVrpPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      initialDcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      liftOffDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      touchDownDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      touchDownDcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      finalDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDcmPosition = new FramePoint(ReferenceFrame.getWorldFrame());
      currentDcmVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

      xUnloadTrajectory = new YoPolynomial("xUnloadTrajectory", 4, registry);
      yUnloadTrajectory = new YoPolynomial("yUnloadTrajectory", 4, registry);
      zUnloadTrajectory = new YoPolynomial("zUnloadTrajectory", 4, registry);
      xLoadTrajectory = new YoPolynomial("xLoadTrajectory", 4, registry);
      yLoadTrajectory = new YoPolynomial("yLoadTrajectory", 4, registry);
      zLoadTrajectory = new YoPolynomial("zLoadTrajectory", 4, registry);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }

   public void initializeTrajectory(double initialTime, FramePoint initialDcmPosition, FrameVector initialDcmVelocity, QuadrupedSupportPolygon supportPolygon, QuadrupedTimedStep quadrupedStep)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      double naturalFrequency = Math.sqrt(gravity / comHeight);

      this.initialized = true;
      this.supportPolygon.set(supportPolygon);
      this.supportPolygon.changeFrame(worldFrame);

      this.initialTime = initialTime;
      this.liftOffTime = quadrupedStep.getTimeInterval().getStartTime();
      this.touchDownTime = quadrupedStep.getTimeInterval().getEndTime();
      this.finalTime = this.touchDownTime + LOAD_DURATION;

      // compute VRP position in swing phase
      FramePoint acrossBodyFootPosition = supportPolygon.getFootstep(quadrupedStep.getRobotQuadrant().getAcrossBodyQuadrant());
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), acrossBodyFootPosition);
      this.supportPolygon.getCentroid2d(swingVrpPosition);
      this.swingVrpPosition.add(0, 0, comHeight);

      // compute DCM position in final support phase
      quadrupedStep.getGoalPosition().changeFrame(worldFrame);
      this.supportPolygon.setFootstep(quadrupedStep.getRobotQuadrant(), quadrupedStep.getGoalPosition());
      this.supportPolygon.getCentroid2d(finalDcmPosition);
      this.finalDcmPosition.add(0, 0, comHeight);

      // compute DCM position and velocity at touch down
      this.touchDownDcmPosition.set(this.finalDcmPosition);
      this.touchDownDcmPosition.sub(this.swingVrpPosition);
      this.touchDownDcmPosition.scale(SWING_PHASE_DIVERGENCE);
      this.touchDownDcmPosition.add(this.swingVrpPosition);
      this.touchDownDcmVelocity.set(this.touchDownDcmPosition);
      this.touchDownDcmVelocity.sub(this.swingVrpPosition);
      this.touchDownDcmVelocity.scale(naturalFrequency);

      // compute DCM position at lift off
      this.liftOffDcmPosition.set(touchDownDcmPosition);
      this.liftOffDcmPosition.sub(swingVrpPosition);
      this.liftOffDcmPosition.scale(Math.exp(-naturalFrequency * (touchDownTime - liftOffTime)));
      this.liftOffDcmPosition.add(swingVrpPosition);

      // compute DCM position in initial support phase
      this.initialDcmPosition.setIncludingFrame(initialDcmPosition);
      this.initialDcmVelocity.setIncludingFrame(initialDcmVelocity);
      this.initialDcmPosition.changeFrame(worldFrame);
      this.initialDcmVelocity.changeFrame(worldFrame);

      // compute polynomial trajectory to interpolate between initial and liftOff Dcm state
      this.xUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDcmPosition.getX(), initialDcmVelocity.getX(), liftOffDcmPosition.getX(), 0.0);
      this.yUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDcmPosition.getY(), initialDcmVelocity.getY(), liftOffDcmPosition.getY(), 0.0);
      this.zUnloadTrajectory.setCubic(initialTime, liftOffTime, initialDcmPosition.getZ(), initialDcmVelocity.getZ(), liftOffDcmPosition.getZ(), 0.0);

      // compute polynomial trajectory to interpolate between touchdown and final DCM state
      this.xLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDcmPosition.getX(), touchDownDcmVelocity.getX(), finalDcmPosition.getX(), 0.0);
      this.yLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDcmPosition.getY(), touchDownDcmVelocity.getY(), finalDcmPosition.getY(), 0.0);
      this.zLoadTrajectory.setCubic(touchDownTime, finalTime, touchDownDcmPosition.getZ(), touchDownDcmVelocity.getZ(), finalDcmPosition.getZ(), 0.0);
   }

   public void computeTrajectory(double currentTime)
   {
      if (!initialized)
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");

      // compute trajectory
      currentTime = Math.min(Math.max(currentTime, initialTime), finalTime);
      if (currentTime < liftOffTime)
      {
         // cubic spline trajectory prior to lift off
         xUnloadTrajectory.compute(currentTime);
         yUnloadTrajectory.compute(currentTime);
         zUnloadTrajectory.compute(currentTime);
         currentDcmPosition.set(xUnloadTrajectory.getPosition(), yUnloadTrajectory.getPosition(), zUnloadTrajectory.getPosition());
         currentDcmVelocity.set(xUnloadTrajectory.getVelocity(), yUnloadTrajectory.getVelocity(), zUnloadTrajectory.getVelocity());
      }
      else if (currentTime < touchDownTime)
      {
         // constant virtual repellent point trajectory during swing phase
         double naturalFrequency = Math.sqrt(gravity / comHeight);
         currentDcmPosition.set(liftOffDcmPosition);
         currentDcmPosition.sub(swingVrpPosition);
         currentDcmPosition.scale(Math.exp(naturalFrequency * (currentTime - liftOffTime)));
         currentDcmPosition.add(swingVrpPosition);
         currentDcmVelocity.set(currentDcmPosition);
         currentDcmVelocity.sub(swingVrpPosition);
         currentDcmVelocity.scale(naturalFrequency);
      }
      else
      {
         // cubic spline trajectory following touch down
         xLoadTrajectory.compute(currentTime);
         yLoadTrajectory.compute(currentTime);
         zLoadTrajectory.compute(currentTime);
         currentDcmPosition.set(xLoadTrajectory.getPosition(), yLoadTrajectory.getPosition(), zLoadTrajectory.getPosition());
         currentDcmVelocity.set(xLoadTrajectory.getVelocity(), yLoadTrajectory.getVelocity(), zLoadTrajectory.getVelocity());
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
}
