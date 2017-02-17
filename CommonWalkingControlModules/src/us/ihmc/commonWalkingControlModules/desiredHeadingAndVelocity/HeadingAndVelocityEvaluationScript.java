package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/*
* TODO: wave cruise, change heading walking straight, turn while facing same direction not working properly
 */
public class HeadingAndVelocityEvaluationScript implements Updatable
{
   private final YoVariableRegistry registry = new YoVariableRegistry("HeadingAndVelocityEvaluationScript");

   private final EnumYoVariable<HeadingAndVelocityEvaluationEvent> evaluationEvent = new EnumYoVariable<HeadingAndVelocityEvaluationEvent>("evaluationEvent",
         registry, HeadingAndVelocityEvaluationEvent.class);

   private final IntegerYoVariable evaluationEventOrderingIndex = new IntegerYoVariable("evaluationEventOrderingIndex", registry);

   private final double controlDT;
   private final DoubleYoVariable acceleration = new DoubleYoVariable("acceleration", registry);
   private final DoubleYoVariable maxVelocity = new DoubleYoVariable("maxVelocity", registry);

   private final DoubleYoVariable maxHeadingDot = new DoubleYoVariable("maxHeadingDot", registry);

   private final DoubleYoVariable cruiseVelocity = new DoubleYoVariable("cruiseVelocity", registry);
   private final DoubleYoVariable sidestepVelocity = new DoubleYoVariable("sidestepVelocity", registry);
   private final DoubleYoVariable desiredVelocityMagnitude = new DoubleYoVariable("desiredVelocityMagnitude", registry);

   private final DoubleYoVariable lastSwitchTime = new DoubleYoVariable("lastSwitchTime", registry);
   private final DoubleYoVariable eventDuration = new DoubleYoVariable("eventDuration", registry);

   private final FrameVector2d desiredVelocityDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());

   private final DoubleYoVariable initialDesiredHeadingAngle = new DoubleYoVariable("initialDesiredHeadingAngle",
         "Temporary variable to hold the initial heading for doing s curves", registry);

   private final SimpleDesiredHeadingControlModule desiredHeadingControlModule;
   private final ManualDesiredVelocityControlModule desiredVelocityControlModule;

   private final HeadingAndVelocityEvaluationEvent[] eventsToCycleThrough;

   public HeadingAndVelocityEvaluationScript(boolean cycleThroughAllEvents, double controlDT, SimpleDesiredHeadingControlModule desiredHeadingControlModule,
         ManualDesiredVelocityControlModule desiredVelocityControlModule, HeadingAndVelocityEvaluationScriptParameters parameters, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;

      if(parameters == null)
      {
         parameters = new HeadingAndVelocityEvaluationScriptParameters();
      }
      
      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.01, 0.0));

      //    desiredVelocityControlModule.setVelocityAlwaysFacesHeading(false);
      //    
      desiredHeadingControlModule.setFinalHeadingTargetAngle(0.0);
      desiredHeadingControlModule.resetHeadingAngle(0.0);
      desiredHeadingControlModule.setMaxHeadingDot(parameters.getMaxHeadingDot());//0.1
      acceleration.set(parameters.getAcceleration());
      maxVelocity.set(parameters.getMaxVelocity()); // (1.5);
      cruiseVelocity.set(parameters.getCruiseVelocity()); // (0.8);
      maxHeadingDot.set(parameters.getHeadingDot());
      sidestepVelocity.set(parameters.getSideStepVelocity());
      eventDuration.set(evaluationEvent.getEnumValue().getMinTimeForScript());

      parentRegistry.addChild(registry);

      if (cycleThroughAllEvents)
      {
         eventsToCycleThrough = HeadingAndVelocityEvaluationEvent.getAllEventsEvaluationOrdering();
      }
      else
      {
         eventsToCycleThrough = HeadingAndVelocityEvaluationEvent.getSomeEventsEvaluationOrdering();

      }
   }

   public double getAcceleration()
   {
      return acceleration.getDoubleValue();
   }

   public double getMaxVelocity()
   {
      return maxVelocity.getDoubleValue();
   }

   public double getMaxHeadingDot()
   {
      return maxHeadingDot.getDoubleValue();
   }

   private final FrameVector2d desiredHeading = new FrameVector2d();

   public void update(double time)
   {
      //      desiredHeadingControlModule.updateDesiredHeadingFrame();

      desiredHeadingControlModule.getDesiredHeading(desiredHeading, 0.0);
      double previousDesiredHeadingAngle = desiredHeadingControlModule.getDesiredHeadingAngle();

      // State transitions:
      if (time + 1e-7 > lastSwitchTime.getDoubleValue() + eventDuration.getDoubleValue())
      {
         lastSwitchTime.set(time);
         switchEventInOrder();

         eventDuration.set(evaluationEvent.getEnumValue().getMinTimeForScript());

         switch (evaluationEvent.getEnumValue())
         {
         case DO_NOTHING_FOR_A_TINY_BIT:
         case STEP_IN_PLACE:
         case SPEED_UP_TO_MAX_STRAIGHT:
         case GO_TO_CRUISE_STRAIGHT:
         case SLOW_DOWN_TO_ZERO_STRAIGHT:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);
            desiredVelocityDirection.set(desiredHeading);

            break;
         }

         case TURN_180_CRUISE:
         {
            desiredHeadingControlModule.setMaxHeadingDot(maxHeadingDot.getDoubleValue() * 0.4);
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle + Math.PI);

            break;
         }

         case SLOW_DOWN_TO_ZERO:
         {
            desiredHeadingControlModule.setMaxHeadingDot(maxHeadingDot.getDoubleValue() * 0.4);

            break;
         }

         case SIDE_STEP_LEFT:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);
            desiredVelocityDirection.set(-desiredHeading.getY(), desiredHeading.getX());

            break;
         }

         case SIDE_STEP_RIGHT:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);
            desiredVelocityDirection.set(desiredHeading.getY(), -desiredHeading.getX());

            break;
         }

         case TURN_IN_PLACE180:
         {
            desiredHeadingControlModule.setMaxHeadingDot(maxHeadingDot.getDoubleValue());
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle - Math.PI);

            break;
         }

         case DIAGONALLY_LEFT_45:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

            Vector3D newDesiredVelocityDirection = new Vector3D(desiredHeading.getX(), desiredHeading.getY(), 0.0);
            newDesiredVelocityDirection.normalize(); // just to be sure
            RigidBodyTransform transform3D = new RigidBodyTransform();
            transform3D.setRotationYawAndZeroTranslation(-Math.PI / 4.0);
            transform3D.transform(newDesiredVelocityDirection);

            desiredVelocityDirection.set(newDesiredVelocityDirection.getX(), newDesiredVelocityDirection.getY());

            break;
         }

         case DIAGONALLY_RIGHT_45:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

            Vector3D newDesiredVelocityDirection = new Vector3D(desiredHeading.getX(), desiredHeading.getY(), 0.0);
            newDesiredVelocityDirection.normalize(); // just to be sure
            RigidBodyTransform transform3D = new RigidBodyTransform();
            transform3D.setRotationYawAndZeroTranslation(Math.PI / 4.0);
            transform3D.transform(newDesiredVelocityDirection);

            desiredVelocityDirection.set(newDesiredVelocityDirection.getX(), newDesiredVelocityDirection.getY());

            break;
         }

         case WAVE_CRUISE:
         {
            desiredHeadingControlModule.setMaxHeadingDot(maxHeadingDot.getDoubleValue() * 0.4);
            initialDesiredHeadingAngle.set(desiredHeadingControlModule.getDesiredHeadingAngle());

            break;
         }

         case CHANGE_HEADING_WALKING_STRAIGHT:
         {
            desiredHeadingControlModule.setMaxHeadingDot(maxHeadingDot.getDoubleValue() * 0.4);
            initialDesiredHeadingAngle.set(desiredHeadingControlModule.getDesiredHeadingAngle());

            break;
         }

         default:
         {
            throw new RuntimeException("Should never get here!");
         }
         }

      }

      // In each state:
      switch (evaluationEvent.getEnumValue())
      {
      case DO_NOTHING_FOR_A_TINY_BIT:
      {
         updateDesiredVelocityMagnitude(0.0);
         updateDesiredVelocityVector();

         break;
      }

      case STEP_IN_PLACE:
      {
         updateDesiredVelocityMagnitude(0.0);
         updateDesiredVelocityVector();

         break;
      }

      case SPEED_UP_TO_MAX_STRAIGHT:
      {
         updateDesiredVelocityMagnitude(maxVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;
      }

      case GO_TO_CRUISE_STRAIGHT:
      {
         updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;

      }

      case TURN_180_CRUISE:
      {
         updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
         updateDesiredVelocityUnitVector(desiredHeading);
         updateDesiredVelocityVector();

         break;
      }

      case SLOW_DOWN_TO_ZERO:
      {
         updateDesiredVelocityMagnitude(0.0);
         updateDesiredVelocityVector();

         break;
      }

      case SLOW_DOWN_TO_ZERO_STRAIGHT:
      {
         updateDesiredVelocityMagnitude(0.0);
         updateDesiredVelocityVector();

         break;
      }

      case SIDE_STEP_LEFT:
      {
         updateDesiredVelocityMagnitude(sidestepVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;
      }

      case SIDE_STEP_RIGHT:
      {
         updateDesiredVelocityMagnitude(sidestepVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;
      }

      case TURN_IN_PLACE180:
      {
         updateDesiredVelocityMagnitude(0.0);
         updateDesiredVelocityUnitVector(desiredHeading);
         updateDesiredVelocityVector();

         break;
      }

      case DIAGONALLY_LEFT_45:
      {
         updateDesiredVelocityMagnitude(sidestepVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;
      }

      case DIAGONALLY_RIGHT_45:
      {
         updateDesiredVelocityMagnitude(sidestepVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         break;
      }

      case WAVE_CRUISE:
      {
         updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
         updateDesiredVelocityUnitVector(desiredHeading);
         updateDesiredVelocityVector();

         double freq = 0.2; // Hz
         double amplitude = Math.PI / 4.0;

         desiredHeadingControlModule.setFinalHeadingTargetAngle(
               initialDesiredHeadingAngle.getDoubleValue() + amplitude * Math.sin(2.0 * Math.PI * freq * (time - lastSwitchTime.getDoubleValue())));

         break;
      }

      case CHANGE_HEADING_WALKING_STRAIGHT:
      {
         desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

         updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
         updateDesiredVelocityVector();

         double freq = 0.1; // Hz
         double amplitude = Math.PI / 4.0;

         desiredHeadingControlModule.setFinalHeadingTargetAngle(
               initialDesiredHeadingAngle.getDoubleValue() + amplitude * Math.sin(2.0 * Math.PI * freq * (time - lastSwitchTime.getDoubleValue())));

         break;
      }

      default:
      {
         throw new RuntimeException("Should never get here!");
      }
      }
   }

   public HeadingAndVelocityEvaluationEvent getEvaluationEvent()
   {
      return evaluationEvent.getEnumValue();
   }

   private void updateDesiredVelocityMagnitude(double finalVelocity)
   {
      double oldVelocity = this.desiredVelocityMagnitude.getDoubleValue();
      double newVelocity;
      double acceleration = this.acceleration.getDoubleValue();

      if (finalVelocity > oldVelocity)
      {
         newVelocity = oldVelocity + acceleration * controlDT;

         if (newVelocity > finalVelocity)
         {
            newVelocity = finalVelocity;
         }
      }
      else
      {
         newVelocity = oldVelocity - acceleration * controlDT;

         if (newVelocity < finalVelocity)
         {
            newVelocity = finalVelocity;
         }
      }

      desiredVelocityMagnitude.set(newVelocity);
   }

   private void updateDesiredVelocityUnitVector(FrameVector2d desiredVelocityDirection)
   {
      this.desiredVelocityDirection.set(desiredVelocityDirection);
   }

   private final FrameVector2d desiredVelocity = new FrameVector2d();

   private void updateDesiredVelocityVector()
   {
      desiredVelocityControlModule.getDesiredVelocity(desiredVelocity);
      desiredVelocity.set(desiredVelocityDirection);
      desiredVelocity.scale(desiredVelocityMagnitude.getDoubleValue());
      desiredVelocityControlModule.setDesiredVelocity(desiredVelocity);
   }

   private void switchEventInOrder()
   {
      evaluationEventOrderingIndex.increment();

      if (evaluationEventOrderingIndex.getIntegerValue() >= eventsToCycleThrough.length)
      {
         evaluationEventOrderingIndex.set(0);
      }

      HeadingAndVelocityEvaluationEvent nextEvent = eventsToCycleThrough[evaluationEventOrderingIndex.getIntegerValue()];

      evaluationEvent.set(nextEvent);
   }

   public enum HeadingAndVelocityEvaluationEvent
   {
      DO_NOTHING_FOR_A_TINY_BIT,
      STEP_IN_PLACE,
      GO_TO_CRUISE_STRAIGHT,
      TURN_180_CRUISE,
      SPEED_UP_TO_MAX_STRAIGHT,
      SLOW_DOWN_TO_ZERO,
      SLOW_DOWN_TO_ZERO_STRAIGHT,
      SIDE_STEP_LEFT,
      SIDE_STEP_RIGHT,
      TURN_IN_PLACE180,
      DIAGONALLY_RIGHT_45,
      DIAGONALLY_LEFT_45,
      WAVE_CRUISE,
      CHANGE_HEADING_WALKING_STRAIGHT;

      public static HeadingAndVelocityEvaluationEvent[] getAllEventsEvaluationOrdering()
      {
         return new HeadingAndVelocityEvaluationEvent[] {STEP_IN_PLACE, GO_TO_CRUISE_STRAIGHT, TURN_180_CRUISE, SPEED_UP_TO_MAX_STRAIGHT,
               SLOW_DOWN_TO_ZERO_STRAIGHT, SIDE_STEP_LEFT, SLOW_DOWN_TO_ZERO, SIDE_STEP_RIGHT, SLOW_DOWN_TO_ZERO, TURN_IN_PLACE180, DIAGONALLY_RIGHT_45,
               SLOW_DOWN_TO_ZERO, DIAGONALLY_LEFT_45, SLOW_DOWN_TO_ZERO, WAVE_CRUISE, SLOW_DOWN_TO_ZERO, TURN_IN_PLACE180, CHANGE_HEADING_WALKING_STRAIGHT,
               SLOW_DOWN_TO_ZERO};
      }

      public static HeadingAndVelocityEvaluationEvent[] getSomeEventsEvaluationOrdering()
      {
         return new HeadingAndVelocityEvaluationEvent[] {
               //               DIAGONALLY_LEFT_45,
               //               DIAGONALLY_RIGHT_45,
               GO_TO_CRUISE_STRAIGHT,

               //               SLOW_DOWN_TO_ZERO, SIDE_STEP_LEFT, SLOW_DOWN_TO_ZERO,
               //          SIDE_STEP_RIGHT, SLOW_DOWN_TO_ZERO
         };
      }

      public double getMinTimeForScript()
      {
         switch (this)
         {
         case DO_NOTHING_FOR_A_TINY_BIT:
         {
            return 0.1;
         }

         case STEP_IN_PLACE:
         {
            return 5.0;
         }

         case GO_TO_CRUISE_STRAIGHT:
         {
            return 6.0;
         }

         case TURN_180_CRUISE:
         {
            return 8.0;
         }

         case SPEED_UP_TO_MAX_STRAIGHT:
         {
            return 4.0;
         }

         case SLOW_DOWN_TO_ZERO:
         {
            return 4.0;
         }

         case SLOW_DOWN_TO_ZERO_STRAIGHT:
         {
            return 6.0;
         }

         case SIDE_STEP_LEFT:
         {
            return 5.0;
         }

         case SIDE_STEP_RIGHT:
         {
            return 5.0;
         }

         case TURN_IN_PLACE180:
         {
            return 8.0;
         }

         case DIAGONALLY_RIGHT_45:
         {
            return 6.0;
         }

         case DIAGONALLY_LEFT_45:
         {
            return 6.0;
         }

         case WAVE_CRUISE:
         {
            return 12.0;
         }

         case CHANGE_HEADING_WALKING_STRAIGHT:
         {
            return 12.0;
         }

         default:
         {
            throw new RuntimeException("Shouldn't get here");
         }
         }
      }

   }
}
