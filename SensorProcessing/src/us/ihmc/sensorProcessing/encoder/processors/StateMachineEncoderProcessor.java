package us.ihmc.sensorProcessing.encoder.processors;


import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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
public class StateMachineEncoderProcessor extends AbstractEncoderProcessor
{
   private final YoEnum<EncoderState> state;
   private final YoDouble previousPosition;
   private final YoDouble previousTime;

   public StateMachineEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);
      this.state = YoEnum.create(name + "EncoderState", EncoderState.class, registry);
      this.previousPosition = new YoDouble(name + "PrevPos", registry);
      this.previousTime = new YoDouble(name + "PrevTime", registry);
   }
   
   public void initialize()
   {
      // empty
   }

   public void update()
   {
      double position = this.rawTicks.getIntegerValue();
      double dt = time.getDoubleValue() - previousTime.getDoubleValue();

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
            this.processedTicks.set(position);
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
//          this.processedTicks.val = position;
//          this.processedTicks.val = alphaOne * this.processedTicks.val + (1.0 - alphaOne) * position;
            this.processedTicks.set(this.processedTicks.getDoubleValue());
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
//          this.processedTicks.val = position;
            this.processedTicks.set(position - 1.0);
            this.processedTickRate.set((position - previousPosition.getDoubleValue()) / (dt));

            break;
         }

         case BackwardOne :
         {
//          this.processedTicks.val = position;
//          this.processedTicks.val = alphaOne * this.processedTicks.val + (1.0 - alphaOne) * position;
            this.processedTicks.set(this.processedTicks.getDoubleValue());
            this.processedTickRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
//          this.processedTicks.val = position;
            this.processedTicks.set(position + 1.0);
            this.processedTickRate.set((position - previousPosition.getDoubleValue()) / (dt));

            break;
         }
      }

      previousPosition.set(position);
      previousTime.set(time.getDoubleValue());
   }

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}
}
