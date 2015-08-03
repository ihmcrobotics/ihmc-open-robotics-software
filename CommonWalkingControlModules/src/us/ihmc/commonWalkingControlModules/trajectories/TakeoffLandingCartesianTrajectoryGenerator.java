package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;
import us.ihmc.yoUtilities.math.trajectories.CartesianTrajectoryGenerator;




/**
 * Class implements a trajectory generator that incorporates the ability to change the endpoint during trajectory execution.
 * The motion of the instantaneous desired position is bounded by acceleration and velocity limits.
 */
public class TakeoffLandingCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator    // , ReferenceFrameHolder
{
   private final boolean ALLOW_RETAKEOFF = false;

   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleGenerator");

   private final EnumYoVariable<SwingState> cartesianTrajectoryState = EnumYoVariable.create("cartesianTrajectoryState", SwingState.class, registry);

   private final DoubleYoVariable maxAccel = new DoubleYoVariable("maxAccel", registry);
   private final DoubleYoVariable maxVel = new DoubleYoVariable("maxVel", registry);
   private final DoubleYoVariable velocityMagnitude = new DoubleYoVariable("suggMaxVel", registry);
   private final DoubleYoVariable zClearance = new DoubleYoVariable("zClearance", registry);
   private final DoubleYoVariable takeOffSlope = new DoubleYoVariable("takeOffSlope", registry);
   private final DoubleYoVariable landingSlope = new DoubleYoVariable("landingSlope", registry);

   private final DoubleYoVariable currentDistanceFromTarget = new DoubleYoVariable("currentDistanceFromTarget", registry);
   private final DoubleYoVariable currentXYDistanceFromTarget = new DoubleYoVariable("currentXYDistanceFromTarget", registry);
   private final DoubleYoVariable currentVelocityMag = new DoubleYoVariable("currentVelocityMag", registry);
   private final DoubleYoVariable currentAccelMag = new DoubleYoVariable("currentAccelMag", registry);

   private final DoubleYoVariable groundZ = new DoubleYoVariable("groundZ", registry);

   private final YoFramePoint initialPosition;
   private final YoFrameVector initialVelocity;

   private final YoFramePoint finalDesiredPosition;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   private final YoFrameVector currentPositionToFinalPosition;
   private final YoFrameVector2d currentToFinal2d;
   private final FrameVector tempFrameVector;
   private final YoFrameVector desiredMaxVelocity;
   private final YoFrameVector desiredAcceleration;

   private final ReferenceFrame referenceFrame;

   public TakeoffLandingCartesianTrajectoryGenerator(double maxAccel, double maxVel, double zClearance, double takeOffSlope, double landingSlope,
           ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this("", maxAccel, maxVel, zClearance, takeOffSlope, landingSlope, referenceFrame, parentRegistry);
   }

   /**
    * @param maxAccel maximum acceleration of instantaneous desired position
    * @param maxVel maximum velocity of instantaneous desired position
    * @param zClearance height above ground to be reached
    * @param takeOffSlope slope (rise / run) in the initial 'take off' state
    * @param landingSlope slope (rise / run) in the final 'landing' state
    * @param referenceFrame TODO
    * @param parentRegistry
    */
   public TakeoffLandingCartesianTrajectoryGenerator(String prefix, double maxAccel, double maxVel, double zClearance, double takeOffSlope,
           double landingSlope, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.maxAccel.set(maxAccel);
      this.maxVel.set(maxVel);
      this.zClearance.set(zClearance);
      this.takeOffSlope.set(takeOffSlope);
      this.landingSlope.set(landingSlope);

      initialPosition = new YoFramePoint(prefix + "initialPosition", "", referenceFrame, registry);
      initialVelocity = new YoFrameVector(prefix + "initialVelocity", "", referenceFrame, registry);

      finalDesiredPosition = new YoFramePoint(prefix + "finalDesiredPosition", "", referenceFrame, registry);
      currentPosition = new YoFramePoint(prefix + "currentPosition", "", referenceFrame, registry);
      currentVelocity = new YoFrameVector(prefix + "currentVelocity", "", referenceFrame, registry);
      currentAcceleration = new YoFrameVector(prefix + "currentAcceleration", "", referenceFrame, registry);

      currentPositionToFinalPosition = new YoFrameVector(prefix + "currentToFinal", "", referenceFrame, registry);
      currentToFinal2d = new YoFrameVector2d(prefix + "currentToFinal2d", "", referenceFrame, registry);
      tempFrameVector = new FrameVector(referenceFrame);
      desiredMaxVelocity = new YoFrameVector(prefix + "desiredMaxVelocity", "", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(prefix + "desiredAcceleration", "", referenceFrame, registry);

      this.referenceFrame = referenceFrame;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }


   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FrameVector initialAcceleration, FramePoint finalDesiredPosition, FrameVector finalDesiredVelocity)
   {
      cartesianTrajectoryState.set(SwingState.TAKE_OFF);

      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
      this.finalDesiredPosition.set(finalDesiredPosition);

      this.currentPosition.set(initialPosition);
      this.currentVelocity.set(initialVelocity);

//    this.groundZ.set(Math.max(initialPosition.getZ(), finalDesiredPosition.getZ()));
      this.groundZ.set(finalDesiredPosition.getZ());
   }

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
   {
      this.finalDesiredPosition.set(finalDesiredPosition);
      this.groundZ.set(finalDesiredPosition.getZ());

//    this.groundZ.set(Math.max(initialPosition.getZ(), finalDesiredPosition.getZ()));
   }

   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      // Simple: If not pointing in right direction, turn toward right direction.
      // If too slow, speed up.
      // If too fast, slow down.
      // x = x0 + v0 t + 1/2 a * t * t;

      updateDistancesToTarget();

      handleStateTransitions();

      computeDesiredMaxVelocity();

      updatePositionVelocityAndAcceleration(deltaT);

      positionToPack.set(currentPosition.getFramePointCopy());
      velocityToPack.set(currentVelocity.getFrameVectorCopy());
      accelerationToPack.set(currentAcceleration.getFrameVectorCopy());
   }

   public void updateDistancesToTarget()
   {
      // Compute the current stuff:
      currentPositionToFinalPosition.sub(finalDesiredPosition.getFramePointCopy(), currentPosition.getFramePointCopy());
      currentDistanceFromTarget.set(currentPositionToFinalPosition.length());
      currentToFinal2d.set(currentPositionToFinalPosition.getX(), currentPositionToFinalPosition.getY());
      currentXYDistanceFromTarget.set(currentToFinal2d.length());

      currentVelocityMag.set(currentVelocity.length());
   }

   private void handleStateTransitions()
   {
      switch (cartesianTrajectoryState.getEnumValue())
      {
         case TAKE_OFF :
         {
            double minGroundClearanceFractionForTransitionToLanding = 0.3;
            boolean reachedClearanceHeight = currentPosition.getZ() >= groundZ.getDoubleValue() + zClearance.getDoubleValue();
            if (reachedClearanceHeight)
            {
               cartesianTrajectoryState.set(SwingState.CRUISE_STATE);
            }
            else if ((currentPosition.getZ() >= groundZ.getDoubleValue() + minGroundClearanceFractionForTransitionToLanding * zClearance.getDoubleValue())
                     && (isAtStartOfLandingSlope()))
            {
               cartesianTrajectoryState.set(SwingState.LANDING);
            }

            break;
         }

         case CRUISE_STATE :
         {
            if (isAtStartOfLandingSlope())
            {
               cartesianTrajectoryState.set(SwingState.LANDING);
            }

            break;
         }

         case LANDING :
         {
            boolean currentPositionBelowDesired = currentPosition.getZ() < finalDesiredPosition.getZ();
            if (currentPositionBelowDesired)
            {
               cartesianTrajectoryState.set(SwingState.DONE);
            }

            double overshootThreshold = 1.05;
            boolean targetWasOvershot = currentXYDistanceFromTarget.getDoubleValue() * landingSlope.getDoubleValue()
                                        > overshootThreshold * Math.max(zClearance.getDoubleValue(), Math.abs(currentPositionToFinalPosition.getZ()));
            if (ALLOW_RETAKEOFF)
            {
               if (targetWasOvershot)
               {
                  cartesianTrajectoryState.set(SwingState.TAKE_OFF);
               }
            }

            break;
         }

         case DONE :
         {
            break;
         }

         default :
         {
            throw new RuntimeException();
         }
      }
   }

   private void computeDesiredMaxVelocity()
   {
      switch (cartesianTrajectoryState.getEnumValue())
      {
         case TAKE_OFF :
         {
            desiredMaxVelocity.set(currentPositionToFinalPosition);
            desiredMaxVelocity.setZ(0.0);

            double distanceRemaining = (groundZ.getDoubleValue() + zClearance.getDoubleValue()) - currentPosition.getZ();
            double percentRemaining = distanceRemaining / zClearance.getDoubleValue();
            percentRemaining = MathTools.clipToMinMax(percentRemaining, 0.4, 1.0);

            desiredMaxVelocity.setZ(desiredMaxVelocity.length() * takeOffSlope.getDoubleValue() * percentRemaining);

            break;
         }

         case CRUISE_STATE :
         {
            desiredMaxVelocity.set(currentPositionToFinalPosition);
            desiredMaxVelocity.setZ(0.0);

            break;
         }

         case LANDING :
         {
            desiredMaxVelocity.set(currentPositionToFinalPosition);

            break;
         }

         case DONE :
         {
//          this.initialPosition.set(currentPosition);
            desiredMaxVelocity.set(0.0, 0.0, 0.0);

            break;
         }

         default :
         {
            throw new RuntimeException();
         }
      }

      enforceSpeedLimits();
   }


   private void enforceSpeedLimits()
   {
      double epsilon = 1e-7;
      if (desiredMaxVelocity.lengthSquared() > epsilon)
      {
         desiredMaxVelocity.normalize();
         velocityMagnitude.set(computeVelocityMagnitude());
         desiredMaxVelocity.scale(velocityMagnitude.getDoubleValue());
      }
   }

   private double computeVelocityMagnitude()
   {
      if (cartesianTrajectoryState.getEnumValue() == SwingState.LANDING)
      {
         double maximumVelocityToStopInTimeSquared = 2.0 * maxAccel.getDoubleValue() * currentDistanceFromTarget.getDoubleValue();

         return Math.min(maximumVelocityToStopInTimeSquared, maxVel.getDoubleValue());
      }
      else
      {
         return maxVel.getDoubleValue();
      }
   }

   private void updatePositionVelocityAndAcceleration(double deltaT)
   {
      boolean alreadyDone = cartesianTrajectoryState.getEnumValue() == SwingState.DONE;
      if (alreadyDone)
      {
         // don't change currentPosition
         currentVelocity.set(0.0, 0.0, 0.0);
         currentAcceleration.set(0.0, 0.0, 0.0);
      }
      else
      {
         computeDesiredMaxVelocity();

         computeCurrentAcceleration(deltaT);
         computeCurrentVelocity(deltaT);
         computeCurrentPosition(deltaT);
      }
   }

   private void computeCurrentAcceleration(double deltaT)
   {
      desiredAcceleration.set(desiredMaxVelocity);
      desiredAcceleration.sub(currentVelocity.getFrameVectorCopy());

      desiredAcceleration.scale(1.0/deltaT);
      
      double desiredAccelerationMagnitude = desiredAcceleration.length();
      if (desiredAccelerationMagnitude > maxAccel.getDoubleValue())
      {
         desiredAcceleration.normalize();
         desiredAcceleration.scale(maxAccel.getDoubleValue());
      }
      
      currentAcceleration.set(desiredAcceleration);
      currentAccelMag.set(currentAcceleration.length());
   }

   private void computeCurrentVelocity(double deltaT)
   {
      tempFrameVector.set(currentAcceleration.getFrameVectorCopy());
      tempFrameVector.scale(deltaT);
      currentVelocity.add(tempFrameVector);
   }

   private void computeCurrentPosition(double deltaT)
   {
//    tempFrameVector.set(currentAcceleration.getFrameVectorCopy());
//    tempFrameVector.scale(0.5 * deltaT);
//    tempFrameVector.add(currentVelocity.getFrameVectorCopy());
//    tempFrameVector.scale(deltaT);
//    currentPosition.add(tempFrameVector);

      tempFrameVector.set(currentVelocity.getFrameVectorCopy());
      tempFrameVector.scale(deltaT);
      currentPosition.add(tempFrameVector);
   }

   public void setTakeoffLandingCartesianTrajectoryParameters(double maxAccel, double maxVel, double zClearance, double takeOffSlope, double landingSlope)
   {
      this.maxAccel.set(maxAccel);
      this.maxVel.set(maxVel);
      this.zClearance.set(zClearance);
      this.takeOffSlope.set(takeOffSlope);
      this.landingSlope.set(landingSlope);
   }

   public void setMaxAcceleration(double maxAccel)
   {
      this.maxAccel.set(maxAccel);
   }

   public void setMaxVelocity(double maxVel)
   {
      this.maxVel.set(maxVel);
   }

   public void setZClearance(double zClearance)
   {
      this.zClearance.set(zClearance);
   }

   public void setTakeOffSlope(double takeOffSlope)
   {
      this.takeOffSlope.set(takeOffSlope);
   }

   public void setLandingSlope(double landingSlope)
   {
      this.landingSlope.set(landingSlope);
   }

   public double getZClearance()
   {
      return zClearance.getDoubleValue();
   }

   public double getMaximumAcceleration()
   {
      return maxAccel.getDoubleValue();
   }

   public double getMaximumVelocity()
   {
      return maxVel.getDoubleValue();
   }

   public double getTakeOffSlope()
   {
      return takeOffSlope.getDoubleValue();
   }

   public double getLandingSlope()
   {
      return landingSlope.getDoubleValue();
   }

   public double getCurrentXYDistanceFromTarget()
   {
      return currentXYDistanceFromTarget.getDoubleValue();
   }

   public boolean isDone()
   {
      return (cartesianTrajectoryState.getEnumValue() == SwingState.DONE);
   }

   private boolean isAtStartOfLandingSlope()
   {
      return currentXYDistanceFromTarget.getDoubleValue() * landingSlope.getDoubleValue() < Math.abs(currentPositionToFinalPosition.getZ());
   }

   public void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder)
   {
      if (this.referenceFrame != referenceFrameHolder.getReferenceFrame())
      {
         String msg = "Argument's frame " + referenceFrameHolder.getReferenceFrame() + " does not match " + this.referenceFrame;

         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      if (this.referenceFrame != frame)
      {
         String msg = "Argument's frame " + frame + " does not match " + this.referenceFrame;

         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

// public ReferenceFrameHolder changeFrameCopy(ReferenceFrame desiredFrame)
// {
//    return null;
// }

   public enum SwingState {TAKE_OFF, CRUISE_STATE, LANDING, DONE;}

   public static void main(String[] args)
   {
      YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("ParentRegistry");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double maxVel = 1.0;
      double maxAccel = 10.0 * maxVel;

      double zClearance = 0.1;
      double takeOffSlope = 1.0;
      double landingSlope = 1.0;

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      CartesianTrajectoryGenerator cartesianTrajectoryGenerator = new TakeoffLandingCartesianTrajectoryGenerator(maxAccel, maxVel, zClearance, takeOffSlope,
                                                                     landingSlope, referenceFrame, yoVariableRegistry);

      FramePoint initialPosition = new FramePoint(referenceFrame, 0.1, 0.1, 0.1);
      FrameVector initialVelocity = new FrameVector(referenceFrame, 0.1, 0.1, 0.1);
      FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 2.0, 2.0, 2.0);

      cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, null, finalDesiredPosition, null);

      new CartesianTrajectoryGeneratorTester(cartesianTrajectoryGenerator, yoVariableRegistry, yoGraphicsListRegistry,
              "cartesianTrajectoryGeneratorTester");
   }

   public double getFinalTime()
   {
      throw new RuntimeException("Final time not known until we get there...");
   }

}
