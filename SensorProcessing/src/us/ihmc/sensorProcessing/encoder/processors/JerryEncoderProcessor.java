package us.ihmc.sensorProcessing.encoder.processors;


import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class JerryEncoderProcessor implements EncoderProcessor
{
   private static final double
//   ALPHA = 0.15, BETA = 0.15, GAMMA = 0.15;
//   ALPHA = 0.3, BETA = 0.3, GAMMA = 0.3; //0.1; //0.15;
//   ALPHA = 1.0, BETA = 1.0, GAMMA = 1.0; //0.1; //0.15;

 ALPHA = 0.5, BETA = 0.5, GAMMA = 0.15; //0.1; //0.15;

   
   private double unitDistancePerCount = 1.0;
   
   private final IntYoVariable rawPosition;
   private final IntYoVariable processedPosition;
   private final DoubleYoVariable processedRate;

   private final EnumYoVariable<EncoderState> state;

   private final IntYoVariable previousRawPosition, previousRawPositionTwoBack;


   private final IntYoVariable previousProcessedPosition, previousProcessedPositionTwoBack;
   private final DoubleYoVariable previousTime, previousTimeTwoBack;

   private final DoubleYoVariable time;
   private final double dt;

   private final DoubleYoVariable maxPossibleRate;
   private final DoubleYoVariable minPriorRate, maxPriorRate, averagePriorRate;

   public JerryEncoderProcessor(String name, IntYoVariable rawPosition, DoubleYoVariable time, double dt, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.rawPosition = rawPosition;
      this.time = time;
      this.dt = dt;

      this.processedPosition = new IntYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);
      this.state = EnumYoVariable.create(name + "state", EncoderState.class, registry);

      this.minPriorRate = new DoubleYoVariable(name + "minPriorRate", registry);
      this.maxPriorRate = new DoubleYoVariable(name + "maxPriorRate", registry);


      this.maxPossibleRate = new DoubleYoVariable(name + "maxPossibleRate", registry);
      this.averagePriorRate = new DoubleYoVariable(name + "averagePriorRate", registry);

      this.previousRawPositionTwoBack = new IntYoVariable(name + "prevRawPos2", registry);
      this.previousRawPosition = new IntYoVariable(name + "prevRawPos", registry);
      this.previousTime = new DoubleYoVariable(name + "prevTime", registry);

      this.previousProcessedPosition = new IntYoVariable(name + "prevPos", registry);
      this.previousProcessedPositionTwoBack = new IntYoVariable(name + "prevPos2", registry);
      this.previousTimeTwoBack = new DoubleYoVariable(name + "prevTime2", registry);

      parentRegistry.addChild(registry);
   }

   public double getQ()
   {
      return ((double) (this.processedPosition.getIntegerValue())) * unitDistancePerCount;
   }

   public double getQd()
   {
      return this.processedRate.getDoubleValue() * unitDistancePerCount;
   }



   public void update()
   {
      boolean positionChanged = rawPosition.getIntegerValue() != previousRawPosition.getIntegerValue();

      if (positionChanged)
         doStateTransitions();

      doStateActionsForPosition(positionChanged);


      if (positionChanged)
      {
         int positionChangeInt = processedPosition.getIntegerValue() - previousProcessedPosition.getIntegerValue();
         double positionChange = (double) positionChangeInt;
         double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();

//       int positionChangeInt = processedPosition.getIntegerValue() - previousProcessedPositionTwoBack.getIntegerValue();
//       double positionChange = (double) positionChangeInt;
//       double timeChange = time.getDoubleValue() - previousTimeTwoBack.getDoubleValue();

         if (positionChangeInt >= 2)
         {
            timeChange = dt; // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!
            
            minPriorRate.set((positionChange - 1.0) / timeChange);
            maxPriorRate.set((positionChange + 1.0) / timeChange);
            averagePriorRate.set(positionChange / timeChange);
         }

         else if (positionChangeInt <= -2)
         {
            timeChange = dt; // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!

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
//            throw new RuntimeException("Should never get here!");
         }

      }

      doStateActionsForVelocity(positionChanged);

      if (positionChanged)
      {
         this.previousProcessedPositionTwoBack.set(this.previousProcessedPosition.getIntegerValue());
         this.previousProcessedPosition.set(processedPosition.getIntegerValue());
         this.previousRawPositionTwoBack.set(previousRawPosition.getIntegerValue());
         this.previousRawPosition.set(rawPosition.getIntegerValue());

         this.previousTimeTwoBack.set(this.previousTime.getDoubleValue());
         this.previousTime.set(this.time.getDoubleValue());
      }
   }



   private void doStateActionsForPosition(boolean positionChanged)
   {
      int rawPosition = this.rawPosition.getIntegerValue();

      switch ((EncoderState) state.getEnumValue())
      {
         case Start :
         {
            this.previousProcessedPositionTwoBack.set(rawPosition);
            this.previousProcessedPosition.set(rawPosition);

            this.previousRawPosition.set(rawPosition);

            this.previousTimeTwoBack.set(time.getDoubleValue());
            this.previousTime.set(time.getDoubleValue());

            this.processedPosition.set(rawPosition);
            this.processedRate.set(0.0);

            break;
         }

         case ForwardOne :
         {
            this.processedPosition.set(rawPosition - 1);

            break;
         }

         case ForwardTwo :
         {
            this.processedPosition.set(rawPosition - 1);

            break;
         }

         case BackwardOne :
         {
            this.processedPosition.set(rawPosition);

            break;
         }

         case BackwardTwo :
         {
            this.processedPosition.set(rawPosition);

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
            this.processedRate.set(0.0);

            break;
         }

         case ForwardTwo :
         {
//            if (positionChanged)
            {
               if (processedRate.getDoubleValue() < minPriorRate.getDoubleValue())
               {
//                  this.processedRate.set(minPriorRate.getDoubleValue() + ALPHA * (averagePriorRate.getDoubleValue() - minPriorRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + ALPHA * (minPriorRate.getDoubleValue() - processedRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
               else if (processedRate.getDoubleValue() > maxPriorRate.getDoubleValue())
               {
//                  this.processedRate.set(maxPriorRate.getDoubleValue() + BETA * (averagePriorRate.getDoubleValue() - maxPriorRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + BETA * (maxPriorRate.getDoubleValue() - processedRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
               else
               {
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
            }
            if (!positionChanged)
            {
               double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();
               maxPossibleRate.set(1.0 / timeChange);

               this.processedRate.set(Math.min(maxPossibleRate.getDoubleValue(), processedRate.getDoubleValue()));
            }

            break;
         }

         case BackwardOne :
         {
            this.processedRate.set(0.0);

            break;
         }

         case BackwardTwo :
         {
//            if (positionChanged)
            {
               if (processedRate.getDoubleValue() > minPriorRate.getDoubleValue())
               {
//                  this.processedRate.set(minPriorRate.getDoubleValue() + ALPHA * (averagePriorRate.getDoubleValue() - minPriorRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + ALPHA * (minPriorRate.getDoubleValue() - processedRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
               else if (processedRate.getDoubleValue() < maxPriorRate.getDoubleValue())
               {
//                  this.processedRate.set(maxPriorRate.getDoubleValue() + BETA * (averagePriorRate.getDoubleValue() - maxPriorRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + BETA * (maxPriorRate.getDoubleValue() - processedRate.getDoubleValue()));
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
               else
               {
                  this.processedRate.set(processedRate.getDoubleValue() + GAMMA * (averagePriorRate.getDoubleValue() - processedRate.getDoubleValue()));
               }
            } 
            if (!positionChanged)
            {
               double timeChange = time.getDoubleValue() - previousTime.getDoubleValue();
               maxPossibleRate.set(-1.0 / timeChange);

               this.processedRate.set(Math.max(maxPossibleRate.getDoubleValue(), processedRate.getDoubleValue()));
            }

            break;
         }
      }


   }

   private void doStateTransitions()
   {
      int rawPosition = this.rawPosition.getIntegerValue();
      int previousPosition = this.previousRawPosition.getIntegerValue();

      boolean increasing = rawPosition > previousPosition;
      boolean decreasing = rawPosition < previousPosition;

      boolean increasingMoreThanOne = rawPosition > previousPosition + 1;
      boolean decreasingMoreThanOne = rawPosition < previousPosition - 1;

      
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
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }



   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}

}
