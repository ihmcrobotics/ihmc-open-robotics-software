package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


public class SmoothSupportState
{
   private boolean isFirstStep;
   private boolean isSingleSupport;
   private double singleSupportTime;
   private double doubleSupportTime;
   private double initialTransferSupportTime;
   private double currentTime;
   private double moveTime;
   private double steppingTime;
   private boolean stepListUpdateRequestFlag;

   public SmoothSupportState()
   {
   }

   public void initializeSupportState(boolean isSingleSupportIni, double singleSupportTimeExt, double doubleSuppotTimeExt, double initialTransferSupportTimeExt)
   {
      isFirstStep = true;

      singleSupportTime = singleSupportTimeExt;

      doubleSupportTime = doubleSuppotTimeExt;

      initialTransferSupportTime = initialTransferSupportTimeExt;

      isSingleSupport = isSingleSupportIni;

      currentTime = 0;

      updateMoveTime();

      stepListUpdateRequestFlag = true;
   }

   public void setStepListUpdateRequestFlag(boolean value)
   {
      stepListUpdateRequestFlag = value;
   }

   public boolean getStepListUpdateRequestFlag()
   {
      return stepListUpdateRequestFlag;
   }

   private void updateMoveTime()
   {
      if (isSingleSupport)
         moveTime = singleSupportTime;
      else
      {
         if (isFirstStep)
         {
            moveTime = initialTransferSupportTime;
         }
         else
            moveTime = doubleSupportTime;
      }
   }

   public boolean swapState()
   {
      isSingleSupport = !isSingleSupport;
      updateMoveTime();

      return isSingleSupport;
   }

   public void propagateStateAndStateTime(double simDT)
   {
      currentTime += simDT;

      if (currentTime >= moveTime)
      {
         if (!isSingleSupport)
         {
            stepListUpdateRequestFlag = true;
            isFirstStep = false;
         }

         this.swapState();

         currentTime = 0;

         updateMoveTime();
      }
   }


   public boolean getIsFirstStep()
   {
      return isFirstStep;
   }


   public boolean getIsSingleSupport()
   {
      return isSingleSupport;
   }

   public double getMoveTime()
   {
      return moveTime;
   }

   public double getCurrentTime()
   {
      return currentTime;
   }

   public double getSteppingTime()
   {
      steppingTime = singleSupportTime + doubleSupportTime;

      return steppingTime;
   }

   public double getDoubleSupportTime()
   {
      return doubleSupportTime;
   }

   public double getSingleSupportTime()
   {
      return singleSupportTime;
   }

   public double getInitialTransferSupportTime()
   {
      return initialTransferSupportTime;
   }


}
