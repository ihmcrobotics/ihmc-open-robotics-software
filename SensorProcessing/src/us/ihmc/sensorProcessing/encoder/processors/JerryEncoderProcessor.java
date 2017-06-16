package us.ihmc.sensorProcessing.encoder.processors;


import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class JerryEncoderProcessor extends AbstractEncoderProcessor
{
   private static final double

//    ALPHA = 0.15, BETA = 0.15, GAMMA = 0.15;
//    ALPHA = 0.3, BETA = 0.3, GAMMA = 0.3; //0.1; //0.15;
//   ALPHA = 1.0, BETA = 1.0, GAMMA = 1.0; //0.1; //0.15;
//   ALPHA = 0.8, BETA = 0.8, GAMMA = 0.0; //0.1; //0.15;

      ALPHA = 0.5, BETA = 0.5, GAMMA = 0.15;    // 0.1; //0.15;

   private final YoEnum<EncoderState> state;

   private final YoInteger previousRawTicks, previousRawTicksTwoBack;
   private final YoInteger previousProcessedTicks, previousProcessedTicksTwoBack;
   private final YoDouble previousTime, previousTimeTwoBack;

   private final double dt;

   private final YoDouble maxPossibleRate;
   private final YoDouble minPriorRate, maxPriorRate, averagePriorRate;
   private int updateCount=0;
   private final int slowUpdateFactor;
   public JerryEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, double dt,YoVariableRegistry registry)
   {
      this( name,  rawTicks,  time,  distancePerTick,  dt, 1, registry);
   }

   public JerryEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, double dt, int slowUpdateFactor,YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.slowUpdateFactor = slowUpdateFactor;
      this.dt = dt*slowUpdateFactor;

      this.state = YoEnum.create(name + "state", EncoderState.class, registry);

      this.minPriorRate = new YoDouble(name + "minPriorRate", registry);
      this.maxPriorRate = new YoDouble(name + "maxPriorRate", registry);


      this.maxPossibleRate = new YoDouble(name + "maxPossibleRate", registry);
      this.averagePriorRate = new YoDouble(name + "averagePriorRate", registry);

      this.previousRawTicksTwoBack = new YoInteger(name + "prevRawPos2", registry);
      this.previousRawTicks = new YoInteger(name + "prevRawPos", registry);
      this.previousTime = new YoDouble(name + "prevTime", registry);

      this.previousProcessedTicks = new YoInteger(name + "prevPos", registry);
      this.previousProcessedTicksTwoBack = new YoInteger(name + "prevPos2", registry);
      this.previousTimeTwoBack = new YoDouble(name + "prevTime2", registry);
   }

   public void initialize()
   {
      update();
   }

   public void update()
   {
      if(updateCount==0)
         performUpdate();
      
      updateCount = (updateCount+1) % slowUpdateFactor;           
   }
   public void performUpdate()
   {
      boolean positionChanged = rawTicks.getIntegerValue() != previousRawTicks.getIntegerValue();

      if (positionChanged)
         doStateTransitions();

      doStateActionsForPosition(positionChanged);


      if (positionChanged)
      {
         double positionChange = processedTicks.getDoubleValue() - previousProcessedTicks.getIntegerValue();
         int positionChangeInt = (int) positionChange;
         double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();

//       int positionChangeInt = processedTicks.getIntegerValue() - previousProcessedTicksTwoBack.getIntegerValue();
//       double positionChange = (double) positionChangeInt;
//       double timeChange = time.getDoubleValue() - previousTimeTwoBack.getDoubleValue();

         if (positionChangeInt >= 2)
         {
            timeChange = dt;    // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!

            minPriorRate.set((positionChange - 1.0) / timeChange);
            maxPriorRate.set((positionChange + 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else if (positionChangeInt <= -2)
         {
            timeChange = dt;    // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!

            minPriorRate.set((positionChange + 1.0) / timeChange);
            maxPriorRate.set((positionChange - 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else if (timeChange > 1.5 * dt)
         {
            minPriorRate.set(positionChange / (timeChange + dt));
            maxPriorRate.set(positionChange / (timeChange - dt));
            averagePriorRate.set(positionChange / timeChange);
         }


         else if (positionChangeInt == 1)
         {
            minPriorRate.set(positionChange / (timeChange + dt));
            maxPriorRate.set((positionChange + 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else if (positionChangeInt == -1)
         {
            minPriorRate.set(positionChange / (timeChange + dt));
            maxPriorRate.set((positionChange - 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else if (positionChangeInt == 0)
         {
            maxPriorRate.set((positionChange + 1.0) / timeChange);
            minPriorRate.set((positionChange - 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else
         {
            System.err.println("Should never get here!");
            System.err.println("positionChangeInt = " + positionChangeInt);
            System.err.println("timeChange = " + timeChange);

//          throw new RuntimeException("Should never get here!");
         }

      }

      doStateActionsForVelocity(positionChanged);

      if (positionChanged)
      {
         this.previousProcessedTicksTwoBack.set(this.previousProcessedTicks.getIntegerValue());
         this.previousProcessedTicks.set((int) processedTicks.getDoubleValue());
         this.previousRawTicksTwoBack.set(previousRawTicks.getIntegerValue());
         this.previousRawTicks.set(rawTicks.getIntegerValue());

         this.previousTimeTwoBack.set(this.previousTime.getDoubleValue());
         this.previousTime.set(this.time.getDoubleValue());
      }
   }



   private void doStateActionsForPosition(boolean positionChanged)
   {
      int rawTicks = this.rawTicks.getIntegerValue();

      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            this.previousProcessedTicksTwoBack.set(rawTicks);
            this.previousProcessedTicks.set(rawTicks);

            this.previousRawTicks.set(rawTicks);

            this.previousTimeTwoBack.set(time.getDoubleValue());
            this.previousTime.set(time.getDoubleValue());

            this.processedTicks.set(rawTicks);
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
            this.processedTicks.set(rawTicks - 1);

            break;
         }

         case ForwardTwo :
         {
            this.processedTicks.set(rawTicks - 1);

            break;
         }

         case BackwardOne :
         {
            this.processedTicks.set(rawTicks);

            break;
         }

         case BackwardTwo :
         {
            this.processedTicks.set(rawTicks);

            break;
         }
      }


   }


   private void doStateActionsForVelocity(boolean positionChanged)
   {
      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            break;
         }

         case ForwardOne :
         {
            this.processedTickRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
//          if (positionChanged)
            {
               if (processedTickRate.getDoubleValue() < minPriorRate.getDoubleValue())
               {
//                this.processedTickRate.set(minPriorRate.getDoubleValue() + ALPHA * (averagePriorRate.getDoubleValue() - minPriorRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + ALPHA * (minPriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
               else if (processedTickRate.getDoubleValue() > maxPriorRate.getDoubleValue())
               {
//                this.processedTickRate.set(maxPriorRate.getDoubleValue() + BETA * (averagePriorRate.getDoubleValue() - maxPriorRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + BETA * (maxPriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
               else
               {
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
            }

            if (!positionChanged)
            {
               double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();
               maxPossibleRate.set(1.0 / timeChange);

               this.processedTickRate.set(Math.min(maxPossibleRate.getDoubleValue(), processedTickRate.getDoubleValue()));
            }

            break;
         }

         case BackwardOne :
         {
            this.processedTickRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
//          if (positionChanged)
            {
               if (processedTickRate.getDoubleValue() > minPriorRate.getDoubleValue())
               {
//                this.processedTickRate.set(minPriorRate.getDoubleValue() + ALPHA * (averagePriorRate.getDoubleValue() - minPriorRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + ALPHA * (minPriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue()
                                             + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
               else if (processedTickRate.getDoubleValue() < maxPriorRate.getDoubleValue())
               {
//                this.processedTickRate.set(maxPriorRate.getDoubleValue() + BETA * (averagePriorRate.getDoubleValue() - maxPriorRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue() + BETA * (maxPriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
                  this.processedTickRate.set(processedTickRate.getDoubleValue()
                                             + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
               else
               {
                  this.processedTickRate.set(processedTickRate.getDoubleValue()
                                             + GAMMA * (averagePriorRate.getDoubleValue() - processedTickRate.getDoubleValue()));
               }
            }

            if (!positionChanged)
            {
               double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();
               maxPossibleRate.set(-1.0 / timeChange);

               this.processedTickRate.set(Math.max(maxPossibleRate.getDoubleValue(), processedTickRate.getDoubleValue()));
            }

            break;
         }
      }


   }

   private void doStateTransitions()
   {
      int rawTicks = this.rawTicks.getIntegerValue();
      int previousPosition = this.previousRawTicks.getIntegerValue();

      boolean increasing = rawTicks > previousPosition;
      boolean decreasing = rawTicks < previousPosition;

      boolean increasingMoreThanOne = rawTicks > previousPosition + 1;
      boolean decreasingMoreThanOne = rawTicks < previousPosition - 1;


      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            if (increasing)
            {
               state.set(EncoderState.ForwardOne);
            }
            else if (decreasing)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardOne :
         {
            if (increasing)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (decreasingMoreThanOne)
            {
               state.set(EncoderState.BackwardTwo);
            }
            else if (decreasing)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardTwo :
         {
            if (increasing)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (decreasingMoreThanOne)
            {
               state.set(EncoderState.BackwardTwo);
            }
            else if (decreasing)
            {
               state.set(EncoderState.BackwardOne);
            }

            break;
         }

         case BackwardOne :
         {
            if (decreasing)
            {
               state.set(EncoderState.BackwardTwo);
            }
            else if (increasingMoreThanOne)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (increasing)
            {
               state.set(EncoderState.ForwardOne);
            }

            break;
         }

         case BackwardTwo :
         {
            if (decreasing)
            {
               state.set(EncoderState.BackwardTwo);
            }
            else if (increasingMoreThanOne)
            {
               state.set(EncoderState.ForwardTwo);
            }
            else if (increasing)
            {
               state.set(EncoderState.ForwardOne);
            }


            break;
         }
      }
   }

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}

}
