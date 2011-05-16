package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/*
* TODO: wave cruise, change heading walking straight, turn while facing same direction not working properly
 */
public class HeadingAndVelocityEvaluationScript
{
   private final YoVariableRegistry registry = new YoVariableRegistry("HeadingAndVelocityEvaluationScript");

   private final EnumYoVariable<HeadingAndVelocityEvaluationEvent> event = new EnumYoVariable<HeadingAndVelocityEvaluationEvent>("event", registry,
                                                                              HeadingAndVelocityEvaluationEvent.class);

   private final double controlDT;
   private final DoubleYoVariable acceleration = new DoubleYoVariable("acceleration", registry);
   private final DoubleYoVariable maxVelocity = new DoubleYoVariable("maxVelocity", registry);
   private final DoubleYoVariable cruiseVelocity = new DoubleYoVariable("cruiseVelocity", registry);
   private final DoubleYoVariable desiredVelocityMagnitude = new DoubleYoVariable("currentVelocity", registry);

   private final DoubleYoVariable lastSwitchTime = new DoubleYoVariable("lastSwitchTime", registry);
   private final DoubleYoVariable eventDuration = new DoubleYoVariable("eventDuration", registry);

   private final FrameVector2d desiredVelocityDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());

   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final ManualDesiredVelocityControlModule desiredVelocityControlModule;

   public HeadingAndVelocityEvaluationScript(double controlDT, DesiredHeadingControlModule desiredHeadingControlModule,
           ManualDesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;

      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.01, 0.0));

//    desiredVelocityControlModule.setVelocityAlwaysFacesHeading(false);
//    
      desiredHeadingControlModule.setFinalHeadingTargetAngle(0.0);
      desiredHeadingControlModule.resetHeadingAngle(0.0);

      acceleration.set(0.2);
      maxVelocity.set(1.0);
      cruiseVelocity.set(0.5);
      eventDuration.set(10.0);

      parentRegistry.addChild(registry);
   }


   public void update(double time)
   {
      desiredHeadingControlModule.updateDesiredHeadingFrame();

      FrameVector2d desiredHeading = desiredHeadingControlModule.getDesiredHeading();
      double previousDesiredHeadingAngle = desiredHeadingControlModule.getDesiredHeadingAngle();

      // State transitions:
      if (time > lastSwitchTime.getDoubleValue() + eventDuration.getDoubleValue())
      {
         lastSwitchTime.set(time);
         switchEventInOrder();

         switch (event.getEnumValue())
         {
            case STEP_IN_PLACE :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               break;
            }

            case SPEED_UP_TO_MAX_STRAIGHT :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               break;
            }

            case GO_TO_CRUISE_STRAIGHT :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               break;

            }

            case TURN_180_CRUISE :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle + Math.PI);

               break;
            }

            case SLOW_DOWN_TO_ZERO_STRAIGHT :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               break;
            }

            case SIDE_STEP_LEFT :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);
               desiredVelocityDirection.set(-desiredHeading.getY(), desiredHeading.getX());

               break;
            }

            case SIDE_STEP_RIGHT :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);
               desiredVelocityDirection.set(desiredHeading.getY(), -desiredHeading.getX());

               break;
            }

            case TURN_IN_PLACE360 :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle - 2.0 * Math.PI);

               break;
            }

            case DIAGONALLY_LEFT_45 :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               Vector3d newDesiredVelocityDirection = new Vector3d(desiredHeading.getX(), desiredHeading.getY(), 0.0);
               newDesiredVelocityDirection.normalize();    // just to be sure
               Transform3D transform3D = new Transform3D();
               transform3D.rotZ(-Math.PI / 4.0);
               transform3D.transform(newDesiredVelocityDirection);

               desiredVelocityDirection.set(newDesiredVelocityDirection.getX(), newDesiredVelocityDirection.getY());

               break;
            }

            case DIAGONALLY_RIGHT_45 :
            {
               desiredHeadingControlModule.setFinalHeadingTargetAngle(previousDesiredHeadingAngle);

               Vector3d newDesiredVelocityDirection = new Vector3d(desiredHeading.getX(), desiredHeading.getY(), 0.0);
               newDesiredVelocityDirection.normalize();    // just to be sure
               Transform3D transform3D = new Transform3D();
               transform3D.rotZ(Math.PI / 4.0);
               transform3D.transform(newDesiredVelocityDirection);

               desiredVelocityDirection.set(newDesiredVelocityDirection.getX(), newDesiredVelocityDirection.getY());

               break;
            }
         }
      }


      // In each state:
      switch (event.getEnumValue())
      {
         case STEP_IN_PLACE :
         {
            updateDesiredVelocityMagnitude(0.0);
            updateDesiredVelocityVector();

            break;
         }

         case SPEED_UP_TO_MAX_STRAIGHT :
         {
            updateDesiredVelocityMagnitude(maxVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;
         }

         case GO_TO_CRUISE_STRAIGHT :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;

         }

         case TURN_180_CRUISE :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityUnitVector(desiredHeading);
            updateDesiredVelocityVector();

            break;
         }

         case SLOW_DOWN_TO_ZERO_STRAIGHT :
         {
            updateDesiredVelocityMagnitude(0.0);
            updateDesiredVelocityVector();

            break;
         }

         case SIDE_STEP_LEFT :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;
         }

         case SIDE_STEP_RIGHT :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;
         }

         case TURN_IN_PLACE360 :
         {
            updateDesiredVelocityMagnitude(0.0);
            updateDesiredVelocityUnitVector(desiredHeading);
            updateDesiredVelocityVector();

            break;
         }

         case DIAGONALLY_LEFT_45 :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;
         }

         case DIAGONALLY_RIGHT_45 :
         {
            updateDesiredVelocityMagnitude(cruiseVelocity.getDoubleValue());
            updateDesiredVelocityVector();

            break;
         }
      }
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

   private void updateDesiredVelocityVector()
   {
      FrameVector2d desiredVelocity = new FrameVector2d(desiredVelocityControlModule.getDesiredVelocity().getReferenceFrame());
      desiredVelocity.set(desiredVelocityDirection);
      desiredVelocity.scale(desiredVelocityMagnitude.getDoubleValue());
      desiredVelocityControlModule.setDesiredVelocity(desiredVelocity);
   }

   private void switchEventInOrder()
   {
      HeadingAndVelocityEvaluationEvent currentEvent = event.getEnumValue();

      int nextOrdinal = (currentEvent.ordinal() + 1) % HeadingAndVelocityEvaluationEvent.values().length;

      HeadingAndVelocityEvaluationEvent nextEvent = HeadingAndVelocityEvaluationEvent.values()[nextOrdinal];

      event.set(nextEvent);
   }

   public enum HeadingAndVelocityEvaluationEvent
   {
      STEP_IN_PLACE, SPEED_UP_TO_MAX_STRAIGHT, GO_TO_CRUISE_STRAIGHT, TURN_180_CRUISE, SLOW_DOWN_TO_ZERO_STRAIGHT, SIDE_STEP_LEFT, SIDE_STEP_RIGHT,
      TURN_IN_PLACE360, DIAGONALLY_RIGHT_45, DIAGONALLY_LEFT_45, WAVE_CRUISE, CHANGE_HEADING_WALKING_STRAIGHT, TURN_WHILE_FACING_SAME_DIRECTION
   }
}
