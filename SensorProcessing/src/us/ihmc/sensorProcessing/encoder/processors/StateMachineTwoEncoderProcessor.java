package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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
public class StateMachineTwoEncoderProcessor implements EncoderProcessor
{
   private final IntYoVariable rawPosition;
   private final DoubleYoVariable processedPosition, processedRate;

   private final EnumYoVariable<EncoderState> state;
   private final DoubleYoVariable previousPosition, previousTime;
   private final DoubleYoVariable previousPositionTwoBack, previousTimeTwoBack;

   private final DoubleYoVariable time;

   private double unitDistancePerCount = 1.0;
   
   public StateMachineTwoEncoderProcessor(String name, IntYoVariable rawPosition, DoubleYoVariable time, YoVariableRegistry registry)
   {
      this.rawPosition = rawPosition;
      this.time = time;

      this.processedPosition = new DoubleYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);
      this.state = EnumYoVariable.create(name + "encoderState", EncoderState.class, registry);

      this.previousPosition = new DoubleYoVariable(name + "prevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "prevTime", registry);
      this.previousPositionTwoBack = new DoubleYoVariable(name + "prevPos2", registry);
      this.previousTimeTwoBack = new DoubleYoVariable(name + "prevTime2", registry);
   }

   public double getQ()
   {
      return this.processedPosition.getDoubleValue() * unitDistancePerCount;
   }

   public double getQd()
   {
      return this.processedRate.getDoubleValue() * unitDistancePerCount;
   }


   public void update()
   {
      double position = this.rawPosition.getIntegerValue();
      boolean positionChanged = (Math.abs(position - previousPosition.getDoubleValue()) > 1e-7);

      // First state transitions:
      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            if (position > previousPosition.getDoubleValue())
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.getDoubleValue())
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardOne :
         {
            if (position > previousPosition.getDoubleValue())
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (position < previousPosition.getDoubleValue())
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardTwo :
         {
            if (position > previousPosition.getDoubleValue())
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (position < previousPosition.getDoubleValue())
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case BackwardOne :
         {
            if (position > previousPosition.getDoubleValue())
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.getDoubleValue())
            {
               state.set(EncoderState.BackwardTwo);
            }

            break;
         }

         case BackwardTwo :
         {
            if (position > previousPosition.getDoubleValue())
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (position < previousPosition.getDoubleValue())
            {
               state.set(EncoderState.BackwardTwo);
            }

            break;
         }
      }

      // State Actions:
      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            this.previousPositionTwoBack.set(position);
            this.previousPosition.set(position);

            this.previousTimeTwoBack.set(time.getDoubleValue());
            this.previousTime.set(time.getDoubleValue());

            this.processedPosition.set(position);
            this.processedRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
            this.processedPosition.set(position - 0.5);    // this.processedPosition.val;
            this.processedRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
            this.processedPosition.set(position - 0.5);
            double positionChange = this.previousPositionTwoBack.getDoubleValue() - position;
            double timeChange = this.previousTimeTwoBack.getDoubleValue() - this.time.getDoubleValue();
            
            this.processedRate.set(positionChange / timeChange); 
            
//          this.processedRate.val = (this.previousPositionTwoBack.val - (position - 0.5))/(this.previousTimeTwoBack.val - this.time.val);
            break;
         }

         case BackwardOne :
         {
            this.processedPosition.set(position + 0.5);    // this.processedPosition.val;
            this.processedRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
            this.processedPosition.set(position + 0.5);
            double positionChange = this.previousPositionTwoBack.getDoubleValue() - position;
            double timeChange = this.previousTimeTwoBack.getDoubleValue() - this.time.getDoubleValue();
            
            this.processedRate.set(positionChange / timeChange); 
            
//          this.processedRate.val = (this.previousPositionTwoBack.val - (position + 0.5))/(this.previousTimeTwoBack.val - this.time.val);
            break;
         }
      }

      if (positionChanged)
      {
         this.previousPositionTwoBack.set(this.previousPosition.getDoubleValue());
         this.previousPosition.set(position);

         this.previousTimeTwoBack.set(this.previousTime.getDoubleValue());
         this.previousTime.set(this.time.getDoubleValue());
      }
   }

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }
}
