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
public class StateMachineEncoderProcessor implements EncoderProcessor
{
   private final IntYoVariable rawPosition;
   private final DoubleYoVariable processedPosition, processedRate;

   private final EnumYoVariable<EncoderState> state;
   private final DoubleYoVariable previousPosition;

   private final double dt;

   private double unitDistancePerCount = 1.0;
   
   @SuppressWarnings("unused")
   private final double alphaOne = 0.0;    // 0.9;

   public StateMachineEncoderProcessor(String name, IntYoVariable rawPosition, double dt, YoVariableRegistry registry)
   {
      this.rawPosition = rawPosition;
      this.dt = dt;

      this.processedPosition = new DoubleYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);
      this.state = EnumYoVariable.create(name + "encoderState", EncoderState.class, registry);
      this.previousPosition = new DoubleYoVariable(name + "prevPos", registry);
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

      // First state transitions:
      EncoderState stateValue = state.getEnumValue();
      switch (stateValue)
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
      switch (state.getEnumValue())
      {
         case Start :
         {
            this.processedPosition.set(position);
            this.processedRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
//          this.processedPosition.val = position;
//          this.processedPosition.val = alphaOne * this.processedPosition.val + (1.0 - alphaOne) * position;
            this.processedPosition.set(this.processedPosition.getDoubleValue());
            this.processedRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
//          this.processedPosition.val = position;
            this.processedPosition.set(position - 1.0);
            this.processedRate.set((position - previousPosition.getDoubleValue()) / (dt));

            break;
         }

         case BackwardOne :
         {
//          this.processedPosition.val = position;
//          this.processedPosition.val = alphaOne * this.processedPosition.val + (1.0 - alphaOne) * position;
            this.processedPosition.set(this.processedPosition.getDoubleValue());
            this.processedRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
//          this.processedPosition.val = position;
            this.processedPosition.set(position + 1.0);
            this.processedRate.set((position - previousPosition.getDoubleValue()) / (dt));

            break;
         }
      }

      previousPosition.set(position);

   }

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }
}
