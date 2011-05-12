package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class HeadingAndVelocityEvaluationScript
{
   private final YoVariableRegistry registry = new YoVariableRegistry("HeadingAndVelocityEvaluationScript");

   private final EnumYoVariable<HeadingAndVelocityEvaluationEvent> event = new EnumYoVariable<HeadingAndVelocityEvaluationEvent>("event", registry, HeadingAndVelocityEvaluationEvent.class);
   
   private final double controlDT;
   private final DoubleYoVariable acceleration = new DoubleYoVariable("acceleration", registry);
   private final DoubleYoVariable maxVelocity = new DoubleYoVariable("maxVelocity", registry);
   private final DoubleYoVariable cruiseVelocity = new DoubleYoVariable("cruiseVelocity", registry);

   private final DoubleYoVariable lastSwitchTime = new DoubleYoVariable("lastSwitchTime", registry);
   private final DoubleYoVariable eventDuration = new DoubleYoVariable("eventDuration", registry);
   
   private final DesiredHeadingControlModule desiredHeadingControlModule;
   private final SimpleDesiredVelocityControlModule desiredVelocityControlModule;
   
   public HeadingAndVelocityEvaluationScript(double controlDT, DesiredHeadingControlModule desiredHeadingControlModule, SimpleDesiredVelocityControlModule desiredVelocityControlModule, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      
      desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 0.01, 0.0));
//      desiredVelocityControlModule.setVelocityAlwaysFacesHeading(false);
//      
      desiredHeadingControlModule.setFinalHeadingTargetAngle(0.0);
      desiredHeadingControlModule.resetHeadingAngle(0.0);
      
      acceleration.set(0.2);
      eventDuration.set(4.0);
      
      parentRegistry.addChild(registry);
   }
   
   
   public void update(double time)
   {
      desiredHeadingControlModule.updateDesiredHeadingFrame();
      desiredVelocityControlModule.updateDesiredVelocity();
      
      FrameVector2d desiredHeading = desiredHeadingControlModule.getDesiredHeading();
      FrameVector2d finalDesiredHeading = desiredHeadingControlModule.getFinalHeadingTarget();
      double desiredHeadingAngle = desiredHeadingControlModule.getDesiredHeadingAngle();
      
      FrameVector2d desiredVelocity = desiredVelocityControlModule.getDesiredVelocity();
      
      
      // State transitions:
      if (time > lastSwitchTime.getDoubleValue() + eventDuration.getDoubleValue())
      {
         lastSwitchTime.set(time);
         switchEventInOrder();
         
         switch(event.getEnumValue())
         {
         case TURN_180_CRUISE:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle + Math.PI);
            
            break;
         }
         
         case SIDE_STEP_LEFT:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle);
            desiredVelocity.set(-desiredHeading.getY(), desiredHeading.getX());
            desiredVelocity.normalize();
            desiredVelocity.scale(0.01);
            break;
         }
         
         case SIDE_STEP_RIGHT:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle);
            desiredVelocity.set(desiredHeading.getY(), -desiredHeading.getX());
            desiredVelocity.normalize();
            desiredVelocity.scale(0.01);
            break;
         }
         
         case TURN_IN_PLACE360:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle - 2.0 * Math.PI);
            break;
         }
         
         case DIAGONALLY_LEFT_45:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle);
            
            Vector3d newDesiredVelocity = new Vector3d(desiredHeading.getX(), desiredHeading.getY(), 0.0);
            Transform3D transform3D = new Transform3D();
            transform3D.rotZ(-Math.PI/4.0);
            transform3D.transform(newDesiredVelocity);
            
            desiredVelocity.set(newDesiredVelocity.getX(), newDesiredVelocity.getY());
            desiredVelocity.scale(0.01);
            desiredVelocityControlModule.setDesiredVelocity(desiredVelocity);
            
            break;
         }
         
         case DIAGONALLY_RIGHT_45:
         {
            desiredHeadingControlModule.setFinalHeadingTargetAngle(desiredHeadingAngle);
            
            Vector3d newDesiredVelocity = new Vector3d(desiredHeading.getX(), desiredHeading.getY(), 0.0);
            Transform3D transform3D = new Transform3D();
            transform3D.rotZ(Math.PI/4.0);
            transform3D.transform(newDesiredVelocity);
            
            desiredVelocity.set(newDesiredVelocity.getX(), newDesiredVelocity.getY());
            desiredVelocity.scale(0.01);
            desiredVelocityControlModule.setDesiredVelocity(desiredVelocity);
            
            break;
         }
         }
      }
      
      
      // In each state:
      switch(event.getEnumValue())
      {
      case STEP_IN_PLACE:
      {
         desiredVelocity.set(0.0, 0.0);
         desiredVelocityControlModule.setDesiredVelocity(desiredVelocity);
         break;
      }
      
      case SPEED_UP_TO_MAX_STRAIGHT:
      {
         desiredHeadingControlModule.setFinalHeadingTarget(desiredHeading);
         accelerateDecelerateTo(maxVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         
         break;
      }
      
      case GO_TO_CRUISE_STRAIGHT:
      {
         desiredHeadingControlModule.setFinalHeadingTarget(desiredHeading);
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);

         break;

      }
      
      case TURN_180_CRUISE:
      {
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         
         break;
      }
            
      case SLOW_DOWN_TO_ZERO_STRAIGHT:
      {
         desiredHeadingControlModule.setFinalHeadingTarget(desiredHeading);

         accelerateDecelerateTo(0.0, desiredVelocity, desiredHeading);
         
         break;
      }
      case SIDE_STEP_LEFT:
      {
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         break;
      }
      
      case SIDE_STEP_RIGHT:
      {
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         break;
      }
      
      case TURN_IN_PLACE360:
      {
         accelerateDecelerateTo(0.0, desiredVelocity, desiredHeading);
         break;
      }
      
      case DIAGONALLY_LEFT_45:
      {
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         break;
      }
      
      case DIAGONALLY_RIGHT_45:
      {
         accelerateDecelerateTo(cruiseVelocity.getDoubleValue(), desiredVelocity, desiredHeading);
         break;
      }

      }
      
//      desiredHeadingControlModule.setFinalDesiredHeading(desiredHeading);
   }
   
   
   private void accelerateDecelerateTo(double finalVelocity, FrameVector2d desiredVelocity, FrameVector2d desiredHeading)
   {
      double scalarVelocity = desiredVelocity.length();
      double newScalarVelocity;
      
      if (finalVelocity > scalarVelocity)
      {
         newScalarVelocity = scalarVelocity + acceleration.getDoubleValue() * controlDT;
         
         if (newScalarVelocity > finalVelocity)
         {
            newScalarVelocity = finalVelocity;
         }
      }
      else
      {
         newScalarVelocity = scalarVelocity - acceleration.getDoubleValue() * controlDT;
         
         if (newScalarVelocity < finalVelocity)
         {
            newScalarVelocity = finalVelocity;
         }
      }
      
      if (scalarVelocity < 1e-7)
      {
         desiredVelocity.set(desiredHeading);
         desiredVelocity.scale(scalarVelocity);
      }
      else
      {
         desiredVelocity.scale(newScalarVelocity/scalarVelocity);
      }
      
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
      STEP_IN_PLACE, SPEED_UP_TO_MAX_STRAIGHT, GO_TO_CRUISE_STRAIGHT, TURN_180_CRUISE, SLOW_DOWN_TO_ZERO_STRAIGHT, 
      SIDE_STEP_LEFT, SIDE_STEP_RIGHT, TURN_IN_PLACE360, DIAGONALLY_RIGHT_45, DIAGONALLY_LEFT_45, WAVE_CRUISE, 
      CHANGE_HEADING_WALKING_STRAIGHT, TURN_WHILE_FACING_SAME_DIRECTION
   }
}
