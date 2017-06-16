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
public class StateMachineTwoEncoderProcessor extends AbstractEncoderProcessor
{
   private final YoEnum<EncoderState> state;
   private final YoDouble previousPosition, previousTime;
   private final YoDouble previousPositionTwoBack, previousTimeTwoBack;


   public StateMachineTwoEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.state = YoEnum.create(name + "EncoderState", EncoderState.class, registry);
      this.previousPosition = new YoDouble(name + "PrevPos", registry);
      this.previousTime = new YoDouble(name + "PrevTime", registry);
      this.previousPositionTwoBack = new YoDouble(name + "PrevPos2", registry);
      this.previousTimeTwoBack = new YoDouble(name + "PrevTime2", registry);
   }
   
   public void initialize()
   {
      update();
   }

   public void update()
   {
      double position = this.rawTicks.getIntegerValue();
      boolean positionChanged = (Math.abs(position - previousPosition.getDoubleValue()) > 1e-7);

      // First state transitions:
      switch (state.getEnumValue())
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
            this.previousPositionTwoBack.set(position);
            this.previousPosition.set(position);

            this.previousTimeTwoBack.set(time.getDoubleValue());
            this.previousTime.set(time.getDoubleValue());

            this.processedTicks.set(position);
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
            this.processedTicks.set(position - 0.5);    // this.processedTicks.val;
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
            this.processedTicks.set(position - 0.5);
            double positionChange = this.previousPositionTwoBack.getDoubleValue() - position;
            double timeChange = this.previousTimeTwoBack.getDoubleValue() - this.time.getDoubleValue();

            this.processedTickRate.set(positionChange / timeChange);

//          this.processedTickRate.val = (this.previousPositionTwoBack.val - (position - 0.5))/(this.previousTimeTwoBack.val - this.time.val);
            break;
         }

         case BackwardOne :
         {
            this.processedTicks.set(position + 0.5);    // this.processedTicks.val;
            this.processedTickRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
            this.processedTicks.set(position + 0.5);
            double positionChange = this.previousPositionTwoBack.getDoubleValue() - position;
            double timeChange = this.previousTimeTwoBack.getDoubleValue() - this.time.getDoubleValue();

            this.processedTickRate.set(positionChange / timeChange);

//          this.processedTickRate.val = (this.previousPositionTwoBack.val - (position + 0.5))/(this.previousTimeTwoBack.val - this.time.val);
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

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo}
}
