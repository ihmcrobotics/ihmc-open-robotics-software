package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.CartesianTrajectoryGeneratorTester;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrameHolder;
import us.ihmc.utilities.math.geometry.ReferenceFrameMismatchException;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;



/**
 * Class implements a trajectory generator that incorporates the ability to change the endpoint during trajectory execution.
 * The motion of the instantaneous desired position is bounded by acceleration and velocity limits.
 */
public class TakeoffLandingCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator, ReferenceFrameHolder
{
   private final boolean ALLOW_RETAKEOFF = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleGenerator");

   private final EnumYoVariable<SwingState> cartesianTrajectoryState = EnumYoVariable.create("cartesianTrajectoryState", SwingState.class, registry);

   private final DoubleYoVariable maxAccel = new DoubleYoVariable("maxAccel", registry);
   private final DoubleYoVariable maxVel = new DoubleYoVariable("maxVel", registry);
   private final DoubleYoVariable suggMaxVel = new DoubleYoVariable("suggMaxVel", registry);
   private final DoubleYoVariable zClearance = new DoubleYoVariable("zClearance", registry);
   private final DoubleYoVariable takeOffSlope = new DoubleYoVariable("takeOffSlope", registry);
   private final DoubleYoVariable landingSlope = new DoubleYoVariable("landingSlope", registry);

   private final DoubleYoVariable currentDistanceFromTarget = new DoubleYoVariable("currentDistanceFromTarget", registry);
   private final DoubleYoVariable currentXYDistanceFromTarget = new DoubleYoVariable("currentXYDistanceFromTarget", registry);
   private final DoubleYoVariable currentVelocityMag = new DoubleYoVariable("currentVelocityMag", registry);
   private final DoubleYoVariable currentAccelMag = new DoubleYoVariable("currentAccelMag", registry);

   @SuppressWarnings("unused")
   private final BooleanYoVariable accelFull = new BooleanYoVariable("accelFull", registry);

   private final DoubleYoVariable groundZ = new DoubleYoVariable("groundZ", registry);

   private final YoFramePoint initialPosition;
   private final YoFrameVector initialVelocity;

   private final YoFramePoint finalDesiredPosition;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   private final YoFrameVector currentToFinal;
   private final YoFrameVector2d currentToFinal2d;
   private final FrameVector tempFrameVector;
   private final YoFrameVector desiredMaxVelocity;
   private final YoFrameVector desiredAcceleration;

   private final ReferenceFrame referenceFrame;

   /**
    * ctor
    * @param maxAccel maximum acceleration of instantaneous desired position
    * @param maxVel maximum velocity of instantaneous desired position
    * @param zClearance height above ground to be reached
    * @param takeOffSlope slope (rise / run) in the initial 'take off' state
    * @param landingSlope slope (rise / run) in the final 'landing' state
    * @param referenceFrame TODO
    * @param parentRegistry
    */
   public TakeoffLandingCartesianTrajectoryGenerator(double maxAccel, double maxVel, double zClearance, double takeOffSlope, double landingSlope,
           ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this.maxAccel.set(maxAccel);
      this.maxVel.set(maxVel);
      this.zClearance.set(zClearance);
      this.takeOffSlope.set(takeOffSlope);
      this.landingSlope.set(landingSlope);

      initialPosition = new YoFramePoint("initialPosition", "", referenceFrame, registry);
      initialVelocity = new YoFrameVector("initialVelocity", "", referenceFrame, registry);

      finalDesiredPosition = new YoFramePoint("finalDesiredPosition", "", referenceFrame, registry);
      currentPosition = new YoFramePoint("currentPosition", "", referenceFrame, registry);
      currentVelocity = new YoFrameVector("currentVelocity", "", referenceFrame, registry);
      currentAcceleration = new YoFrameVector("currentAcceleration", "", referenceFrame, registry);

      currentToFinal = new YoFrameVector("currentToFinal", "", referenceFrame, registry);
      currentToFinal2d = new YoFrameVector2d("currentToFinal2d", "", referenceFrame, registry);
      tempFrameVector = new FrameVector(referenceFrame);
      desiredMaxVelocity = new YoFrameVector("desiredMaxVelocity", "", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector("desiredAcceleration", "", referenceFrame, registry);

      this.referenceFrame = referenceFrame;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * initializes the trajectory generator with an initial position and velocity, final position and ground height.
    * @param groundZ height of the ground.
    * @param initialPosition initial position of the trajectory
    * @param initialVelocity initial velocity of the trajectory
    * @param finalDesiredPosition final desired position of the trajectory (can be updated using updateFinalDesiredPosition later)
    */
   public void initialize(double groundZ, FramePoint initialPosition, FrameVector initialVelocity, FramePoint finalDesiredPosition)
   {
      cartesianTrajectoryState.set(SwingState.TAKE_OFF);

      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
      this.finalDesiredPosition.set(finalDesiredPosition);

      this.currentPosition.set(initialPosition);
      this.currentVelocity.set(initialVelocity);

      this.groundZ.set(groundZ);
   }

   public void setTakeoffLandingCartesianTrajectoryParameters(double maxAccel, double maxVel, double zClearance, double takeOffSlope, double landingSlope)
   {
      this.maxAccel.set(maxAccel);
      this.maxVel.set(maxVel);
      this.zClearance.set(zClearance);
      this.takeOffSlope.set(takeOffSlope);
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

   /**
    * Packs the new desired position, velocity and acceleration.
    * @param positionToPack new desired position to pack
    * @param velocityToPack new desired velocity to pack
    * @param accelerationToPack new desired acceleration to pack
    * @param deltaT time step
    */
   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      // Simple: If not pointing in right direction, turn toward right direction.
      // If too slow, speed up.
      // If too fast, slow down.
      // x = x0 + v0 t + 1/2 a * t * t;

      // Compute the current stuff:
      currentToFinal.sub(finalDesiredPosition.getFramePointCopy(), currentPosition.getFramePointCopy());
      currentDistanceFromTarget.set(currentToFinal.length());
      currentToFinal2d.set(currentToFinal.getX(), currentToFinal.getY());
      currentXYDistanceFromTarget.set(currentToFinal2d.length());

      currentVelocityMag.set(currentVelocity.length());

      // State Transition Conditions:
      switch ((SwingState) cartesianTrajectoryState.getEnumValue())
      {
         case TAKE_OFF :
         {
            double minGroundClearanceFractionForTransitionToLanding = 0.3;

            if (currentPosition.getZ() >= groundZ.getDoubleValue() + zClearance.getDoubleValue())
            {
               cartesianTrajectoryState.set(SwingState.CRUISE_STATE);
            }
            else if ((currentPosition.getZ() >= groundZ.getDoubleValue() + minGroundClearanceFractionForTransitionToLanding * zClearance.getDoubleValue())
                     && (currentXYDistanceFromTarget.getDoubleValue() * landingSlope.getDoubleValue() < Math.abs(currentToFinal.getZ())))
            {
               cartesianTrajectoryState.set(SwingState.LANDING);
            }

            break;
         }

         case CRUISE_STATE :
         {
            if (currentXYDistanceFromTarget.getDoubleValue() * landingSlope.getDoubleValue() < Math.abs(currentToFinal.getZ()))
            {
               cartesianTrajectoryState.set(SwingState.LANDING);
            }

            break;
         }

         case LANDING :
         {
            if (currentPosition.getZ() < finalDesiredPosition.getZ())
            {
               cartesianTrajectoryState.set(SwingState.DONE);
            }


            if ((ALLOW_RETAKEOFF)
                    && (currentXYDistanceFromTarget.getDoubleValue() * landingSlope.getDoubleValue()
                        > 1.05 * Math.max(zClearance.getDoubleValue(), Math.abs(currentToFinal.getZ()))))
            {
               cartesianTrajectoryState.set(SwingState.TAKE_OFF);
            }

            break;
         }

         case DONE :
         {
//          if ((ALLOW_RETAKEOFF) && (currentXYDistanceFromTarget.val * landingSlope.val > 1.05 * Math.max(zClearance.val, Math.abs(currentToFinal.getZ()))))
//          {
//             cartesianTrajectoryState.set(SwingState.TAKE_OFF);
//          }

            break;
         }

         default :
         {
            throw new RuntimeException();
         }
      }

      // Do the actions for each state:

      switch ((SwingState) cartesianTrajectoryState.getEnumValue())
      {
         case TAKE_OFF :
         {
            desiredMaxVelocity.set(currentToFinal);
            desiredMaxVelocity.setZ(0.0);

            double distanceRemaining = (groundZ.getDoubleValue() + zClearance.getDoubleValue()) - currentPosition.getZ();
            double percentRemaining = distanceRemaining / zClearance.getDoubleValue();
            if (percentRemaining < 0.4)
               percentRemaining = 0.4;
            else if (percentRemaining > 1.0)
               percentRemaining = 1.0;


            desiredMaxVelocity.setZ(desiredMaxVelocity.length() * takeOffSlope.getDoubleValue() * percentRemaining);

            break;
         }

         case CRUISE_STATE :
         {
            desiredMaxVelocity.set(currentToFinal);
            desiredMaxVelocity.setZ(0.0);

            break;
         }

         case LANDING :
         {
            desiredMaxVelocity.set(currentToFinal);

            break;
         }

         case DONE :
         {
            this.initialPosition.set(currentPosition);
            desiredMaxVelocity.set(0.0, 0.0, 0.0);

            break;
         }

         default :
         {
            throw new RuntimeException();
         }
      }

      if ((cartesianTrajectoryState.getEnumValue() == SwingState.DONE))
      {
         positionToPack.set(currentPosition.getFramePointCopy());
         velocityToPack.set(0.0, 0.0, 0.0);
         accelerationToPack.set(0.0, 0.0, 0.0);

         return;
      }

      // Set desired maximum velocity to its max length:
      if (desiredMaxVelocity.lengthSquared() > 1e-7)
      {
         desiredMaxVelocity.normalize();
      }


      // check if we need to slow down
      if (cartesianTrajectoryState.getEnumValue() == SwingState.LANDING)
         suggMaxVel.set(getSuggestedMaximumVeloicty());
      else
         suggMaxVel.set(maxVel.getDoubleValue());

//    desiredMaxVelocity.scale(maxVel.val);
      desiredMaxVelocity.scale(suggMaxVel.getDoubleValue());

//    double minTimeToStop = currentVelocityMag.val / maxAccel.val;
//     double distanceAtMinTimeToStop = currentVelocityMag.val * minTimeToStop + 0.5 * maxAccel.val * minTimeToStop * minTimeToStop;
//
//     if (distanceAtMinTimeToStop <= currentDistanceFromTarget.val)
//        accelFull.set(true);
//     else
//        accelFull.set(false);


//    if (cartesianTrajectoryState.getEnumValue() == SwingState.LANDING)
//    {
////  desiredMaxVelocity.scale(0.0); // Come to a halt as quickly as possible in landing?
//    }
//    else
//    {
//       desiredMaxVelocity.scale(maxVel.val);
//    }

      desiredAcceleration.set(desiredMaxVelocity);
      desiredAcceleration.sub(currentVelocity.getFrameVectorCopy());

      if (desiredAcceleration.lengthSquared() < 0.01 * maxVel.getDoubleValue() * maxVel.getDoubleValue())
      {
         desiredAcceleration.scale(0.0);
      }
      else
      {
         desiredAcceleration.normalize();
         desiredAcceleration.scale(maxAccel.getDoubleValue());
      }

      currentAccelMag.set(currentAcceleration.length());
      currentAcceleration.set(desiredAcceleration);

      tempFrameVector.set(currentAcceleration.getFrameVectorCopy());
      tempFrameVector.scale(deltaT);
      currentVelocity.add(tempFrameVector);

      tempFrameVector.set(currentVelocity.getFrameVectorCopy());
      tempFrameVector.scale(deltaT);
      currentPosition.add(tempFrameVector);

      positionToPack.set(currentPosition.getFramePointCopy());
      velocityToPack.set(currentVelocity.getFrameVectorCopy());
      accelerationToPack.set(currentAcceleration.getFrameVectorCopy());
   }

   private double getSuggestedMaximumVeloicty()
   {
      double maximumVelocityToStopInTimeSquared = 2.0 * maxAccel.getDoubleValue() * currentDistanceFromTarget.getDoubleValue();


      return Math.min(maximumVelocityToStopInTimeSquared, maxVel.getDoubleValue());
   }

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
   {
      this.finalDesiredPosition.set(finalDesiredPosition);
   }

   public boolean isDone()
   {
      return (cartesianTrajectoryState.getEnumValue() == SwingState.DONE);
   }

   public enum SwingState {TAKE_OFF, CRUISE_STATE, LANDING, DONE;}

   public static void main(String[] args)
   {
      YoVariableRegistry yoVariableRegistry = new YoVariableRegistry("ParentRegistry");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

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
      FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 1.0, 1.0, 1.0);

      cartesianTrajectoryGenerator.initialize(0.0, initialPosition, initialVelocity, finalDesiredPosition);

      new CartesianTrajectoryGeneratorTester(cartesianTrajectoryGenerator, yoVariableRegistry, dynamicGraphicObjectsListRegistry,
              "cartesianTrajectoryGeneratorTester");
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

   public ReferenceFrameHolder changeFrameCopy(ReferenceFrame desiredFrame)
   {
      return null;
   }

}
