package us.ihmc.sensorProcessing.encoder.processors;



public class JerryEncoderProcessorNoYoVariables
{
   private static final double

//    ALPHA = 0.15, BETA = 0.15, GAMMA = 0.15;
//    ALPHA = 0.3, BETA = 0.3, GAMMA = 0.3; //0.1; //0.15;
//    ALPHA = 1.0, BETA = 1.0, GAMMA = 1.0; //0.1; //0.15;

   ALPHA = 0.5, BETA = 0.5, GAMMA = 0.15;    // 0.1; //0.15;

   protected double processedTicks;
   protected double processedTickRate;
   
   private int previousRawTicks, previousRawTicksTwoBack;
   private int previousProcessedTicks;

   private int previousProcessedTicksTwoBack;
   private double previousTime, previousTimeTwoBack;

   private double dt;
   EncoderState state = EncoderState.Start;
   
   private double maxPossibleRate;
   private double minPriorRate;

   private double maxPriorRate;

   private double averagePriorRate;
   private double distancePerTick;

   public JerryEncoderProcessorNoYoVariables(double dt, double distancePerTick)
   {
      this.dt = dt;
      this.distancePerTick = distancePerTick;
   }

   public void update(int rawTicks, double time)
   {
      boolean positionChanged = rawTicks != previousRawTicks;

      if (positionChanged)
         doStateTransitions(rawTicks);

      doStateActionsForPosition(positionChanged, rawTicks, time);


      if (positionChanged)
      {
         double positionChange = processedTicks - previousProcessedTicks;
         int positionChangeInt = (int) positionChange;
         double timeChange = time - previousTime;

         if (positionChangeInt >= 2)
         {
            timeChange = dt;    // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!

            minPriorRate = ((positionChange - 1.0) / timeChange);
            maxPriorRate = ((positionChange + 1.0) / timeChange);
            averagePriorRate = (positionChange / timeChange);
         }

         else if (positionChangeInt <= -2)
         {
            timeChange = dt;    // If there were multiple time ticks and multiple position ticks, then can assume only one time tick!

            minPriorRate = ((positionChange + 1.0) / timeChange);
            maxPriorRate = ((positionChange - 1.0) / timeChange);
            averagePriorRate = (positionChange / timeChange);
         }

         else if (timeChange > 1.5 * dt)
         {
            minPriorRate = (positionChange / (timeChange + dt));
            maxPriorRate = (positionChange / (timeChange - dt));
            averagePriorRate = (positionChange / timeChange);
         }


         else if (positionChangeInt == 1)
         {
            minPriorRate = (positionChange / (timeChange + dt));
            maxPriorRate = ((positionChange + 1.0) / timeChange);
            averagePriorRate = (positionChange / timeChange);
         }

         else if (positionChangeInt == -1)
         {
            minPriorRate = (positionChange / (timeChange + dt));
            maxPriorRate = ((positionChange - 1.0) / timeChange);
            averagePriorRate = (positionChange / timeChange);
         }

         else if (positionChangeInt == 0)
         {
            maxPriorRate = ((positionChange + 1.0) / timeChange);
            minPriorRate = ((positionChange - 1.0) / timeChange);
            averagePriorRate = (positionChange / timeChange);
         }

         else
         {
            System.err.println("Should never get here!");
            System.err.println("positionChangeInt = " + positionChangeInt);
            System.err.println("timeChange = " + timeChange);

//          throw new RuntimeException("Should never get here!");
         }

      }

      doStateActionsForVelocity(positionChanged, time);

      if (positionChanged)
      {
         this.previousProcessedTicksTwoBack = (this.previousProcessedTicks);
         this.previousProcessedTicks = ((int) processedTicks);
         this.previousRawTicksTwoBack = (previousRawTicks);
         this.previousRawTicks = (rawTicks);

         this.previousTimeTwoBack = (this.previousTime);
         this.previousTime = (time);
      }
   }



   private void doStateActionsForPosition(boolean positionChanged, int rawTicks, double time)
   {

      switch (state)
      {
         case Start :
         {
            this.previousProcessedTicksTwoBack = (rawTicks);
            this.previousProcessedTicks = (rawTicks);

            this.previousRawTicks = (rawTicks);

            this.previousTimeTwoBack = (time);
            this.previousTime = (time);

            this.processedTicks = (rawTicks);
            this.processedTickRate = (0.0);

            break;
         }

         case ForwardOne :
         {
            this.processedTicks = (rawTicks - 1);

            break;
         }

         case ForwardTwo :
         {
            this.processedTicks = (rawTicks - 1);

            break;
         }

         case BackwardOne :
         {
            this.processedTicks = (rawTicks);

            break;
         }

         case BackwardTwo :
         {
            this.processedTicks = (rawTicks);

            break;
         }
      }


   }


   private void doStateActionsForVelocity(boolean positionChanged, double time)
   {
      switch (state)
      {
         case Start :
         {
            break;
         }

         case ForwardOne :
         {
            this.processedTickRate = (0.0);

            break;
         }

         case ForwardTwo :
         {
//          if (positionChanged)
            {
               if (processedTickRate < minPriorRate)
               {
//                this.processedTickRate = (minPriorRate + ALPHA * (averagePriorRate - minPriorRate));
                  this.processedTickRate = (processedTickRate + ALPHA * (minPriorRate - processedTickRate));
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
               else if (processedTickRate > maxPriorRate)
               {
//                this.processedTickRate = (maxPriorRate + BETA * (averagePriorRate - maxPriorRate));
                  this.processedTickRate = (processedTickRate + BETA * (maxPriorRate - processedTickRate));
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
               else
               {
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
            }

            if (!positionChanged)
            {
               double timeChange = time - previousTime;
               maxPossibleRate = (1.0 / timeChange);

               this.processedTickRate = (Math.min(maxPossibleRate, processedTickRate));
            }

            break;
         }

         case BackwardOne :
         {
            this.processedTickRate = (0.0);

            break;
         }

         case BackwardTwo :
         {
//          if (positionChanged)
            {
               if (processedTickRate > minPriorRate)
               {
//                this.processedTickRate = (minPriorRate + ALPHA * (averagePriorRate - minPriorRate));
                  this.processedTickRate = (processedTickRate + ALPHA * (minPriorRate - processedTickRate));
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
               else if (processedTickRate < maxPriorRate)
               {
//                this.processedTickRate = (maxPriorRate + BETA * (averagePriorRate - maxPriorRate));
                  this.processedTickRate = (processedTickRate + BETA * (maxPriorRate - processedTickRate));
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
               else
               {
                  this.processedTickRate = (processedTickRate
                                             + GAMMA * (averagePriorRate - processedTickRate));
               }
            }

            if (!positionChanged)
            {
               double timeChange = time - previousTime;
               maxPossibleRate = (-1.0 / timeChange);

               this.processedTickRate = (Math.max(maxPossibleRate, processedTickRate));
            }

            break;
         }
      }


   }

   private void doStateTransitions(int rawTicks)
   {
      int previousPosition = this.previousRawTicks;

      boolean increasing = rawTicks > previousPosition;
      boolean decreasing = rawTicks < previousPosition;

      boolean increasingMoreThanOne = rawTicks > previousPosition + 1;
      boolean decreasingMoreThanOne = rawTicks < previousPosition - 1;


      switch (state)
      {
         case Start :
         {
            if (increasing)
            {
               state = (EncoderState.ForwardOne);
            }
            else if (decreasing)
            {
               state = (EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardOne :
         {
            if (increasing)
            {
               state = (EncoderState.ForwardTwo);
            }
            else if (decreasingMoreThanOne)
            {
               state = (EncoderState.BackwardTwo);
            }
            else if (decreasing)
            {
               state = (EncoderState.BackwardOne);
            }

            break;
         }

         case ForwardTwo :
         {
            if (increasing)
            {
               state = EncoderState.ForwardTwo;
            }
            else if (decreasingMoreThanOne)
            {
               state = EncoderState.BackwardTwo;
            }
            else if (decreasing)
            {
               state = EncoderState.BackwardOne;
            }

            break;
         }

         case BackwardOne :
         {
            if (decreasing)
            {
               state = EncoderState.BackwardTwo;
            }
            else if (increasingMoreThanOne)
            {
               state = EncoderState.ForwardTwo;
            }
            else if (increasing)
            {
               state = EncoderState.ForwardOne;
            }

            break;
         }

         case BackwardTwo :
         {
            if (decreasing)
            {
               state = EncoderState.BackwardTwo;
            }
            else if (increasingMoreThanOne)
            {
               state = EncoderState.ForwardTwo;
            }
            else if (increasing)
            {
               state = EncoderState.ForwardOne;
            }


            break;
         }
      }
   }

   private enum EncoderState {Start, ForwardOne, ForwardTwo, BackwardOne, BackwardTwo;}

   public double getQ()
   {
      return processedTicks * distancePerTick;
   }

   public double getQd()
   {
      return processedTickRate * distancePerTick;
   }

}
