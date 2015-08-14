package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.YoVariableType;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class EncoderStateMachineVelocityEstimatorTwo implements EncoderStateMachineVelocityEstimator
{
   private final YoVariable rawPosition;
   private final YoVariable processedPosition, processedRate;

   private final YoVariable state;
   private final YoVariable previousPosition, previousTime;
   private final YoVariable previousPositionTwoBack, previousTimeTwoBack;

   private final YoVariable time;

   public EncoderStateMachineVelocityEstimatorTwo(String name, YoVariable rawPosition, YoVariable time, 
	   Robot robot)
   {
      this.rawPosition = rawPosition;
      this.time = time;

      YoVariableRegistry registry = new YoVariableRegistry(name);
      
      this.processedPosition = new YoVariable(name + "procPos", registry);
      this.processedRate = new YoVariable(name + "procRate", registry);
      this.state = new YoVariable(name + "state", EncoderState.values(), registry);

      this.previousPosition = new YoVariable(name + "prevPos", registry);
      this.previousTime = new YoVariable(name + "prevTime", registry);
      this.previousPositionTwoBack = new YoVariable(name + "prevPos2", registry);
      this.previousTimeTwoBack = new YoVariable(name + "prevTime2", registry);
      
      if (robot != null)
	    robot.addYoVariableRegistry(registry);
   }

   public YoVariable getProcessedPosition()
   {
      return this.processedPosition;
   }

   public YoVariable getProcessedRate()
   {
      return this.processedRate;
   }


   public void update()
   {
      double position = this.rawPosition.val;
      boolean positionChanged = (Math.abs(position - previousPosition.val) > 1e-7);

      // First state transitions:
      switch ((EncoderState) state.getEnumValue())
      {
         case Start:
         {
            if (position > previousPosition.val)
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.val)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardOne:
         {
            if (position > previousPosition.val)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (position < previousPosition.val)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardTwo:
         {
            if (position > previousPosition.val)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (position < previousPosition.val)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case BackwardOne:
         {
            if (position > previousPosition.val)
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.val)
            {
               state.set(EncoderState.BackwardTwo);
            }

            break;
         }

         case BackwardTwo:
         {
            if (position > previousPosition.val)
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.val)
            {
               state.set(EncoderState.BackwardTwo);
            }

            break;
         }
      }

      // State Actiions:
      switch ((EncoderState) state.getEnumValue())
      {
         case Start:
         {
            this.previousPositionTwoBack.val = position;
            this.previousPosition.val = position;

            this.previousTimeTwoBack.val = time.val;
            this.previousTime.val = time.val;

            this.processedPosition.val = position;
            this.processedRate.val = 0.0;

            break;
         }

         case ForwardOne:
         {
            this.processedPosition.val = position - 0.5; //this.processedPosition.val;
            this.processedRate.val = 0.0;
            break;
         }

         case ForwardTwo:
         {
            this.processedPosition.val = position - 0.5;
            this.processedRate.val = (this.previousPositionTwoBack.val - (position))/(this.previousTimeTwoBack.val - this.time.val);
//            this.processedRate.val = (this.previousPositionTwoBack.val - (position - 0.5))/(this.previousTimeTwoBack.val - this.time.val);
            break;
         }

         case BackwardOne:
         {
            this.processedPosition.val = position + 0.5; //this.processedPosition.val;
            this.processedRate.val = 0.0;

            break;
         }

         case BackwardTwo:
         {
            this.processedPosition.val = position + 0.5;
            this.processedRate.val = (this.previousPositionTwoBack.val - (position))/(this.previousTimeTwoBack.val - this.time.val);
//            this.processedRate.val = (this.previousPositionTwoBack.val - (position + 0.5))/(this.previousTimeTwoBack.val - this.time.val);
            break;
         }
      }

      if (positionChanged)
      {
         this.previousPositionTwoBack.val = this.previousPosition.val;
         this.previousPosition.val = position;

         this.previousTimeTwoBack.val = this.previousTime.val;
         this.previousTime.val = this.time.val;
      }
   }

   private enum EncoderState
   {
      Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;
   }
}
