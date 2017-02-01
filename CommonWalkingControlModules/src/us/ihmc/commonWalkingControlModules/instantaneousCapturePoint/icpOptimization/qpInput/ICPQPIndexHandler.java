package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

public class ICPQPIndexHandler
{
   protected int numberOfFootstepsToConsider;
   protected int numberOfCMPVertices = 0;
   protected int numberOfReachabilityVertices = 0;
   protected int numberOfFreeVariables = 0;
   protected int numberOfFootstepVariables = 0;
   protected int numberOfLagrangeMultipliers = 2;

   protected int feedbackCMPIndex;
   protected int dynamicRelaxationIndex;
   protected int cmpConstraintIndex;
   protected int reachabilityConstraintIndex;
   protected int lagrangeMultiplierIndex;

   public ICPQPIndexHandler()
   {
   }

   public void setNumberOfCMPVertices(int numberOfCMPVertices)
   {
      this.numberOfCMPVertices = numberOfCMPVertices;
   }

   public void setNumberOfReachabilityVertices(int numberOfReachabilityVertices)
   {
      this.numberOfReachabilityVertices = numberOfReachabilityVertices;
   }

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      if (!useFeedback && (!useStepAdjustment || numberOfFootstepsToConsider < 1))
      {
         throw new RuntimeException("No possible feedback mechanism available.");
      }

      if (useFeedback && !useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;
      else
         this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      numberOfFootstepVariables = 2 * this.numberOfFootstepsToConsider;

      numberOfLagrangeMultipliers = 2;
      numberOfFreeVariables = numberOfFootstepVariables + 2;
      if (useFeedback)
      {
         feedbackCMPIndex = numberOfFootstepVariables;
         dynamicRelaxationIndex = feedbackCMPIndex + 2;

         if (numberOfCMPVertices > 0)
            numberOfLagrangeMultipliers += 3;

         numberOfFreeVariables += 2;
      }
      else
      {
         numberOfCMPVertices = 0;

         feedbackCMPIndex = 0;
         dynamicRelaxationIndex = numberOfFootstepVariables;
      }

      if (numberOfReachabilityVertices > 0)
         numberOfLagrangeMultipliers += 3;

      cmpConstraintIndex = dynamicRelaxationIndex + 2;
      reachabilityConstraintIndex = cmpConstraintIndex + numberOfCMPVertices;
      lagrangeMultiplierIndex = reachabilityConstraintIndex + numberOfReachabilityVertices;

      /*
      if (DEBUG)
      {
         yoFeedbackCMPIndex.set(feedbackCMPIndex);
         yoCmpConstraintIndex.set(cmpConstraintIndex);
         yoDynamicRelaxtionIndex.set(dynamicRelaxationIndex);
         yoLagrangeMultiplierIndex.set(lagrangeMultiplierIndex);
      }

      reset();
      reshape();
      */
   }

   public int getFeedbackCMPIndex()
   {
      return feedbackCMPIndex;
   }

   public int getCMPConstraintIndex()
   {
      return cmpConstraintIndex;
   }

   public int getReachabilityConstraintIndex()
   {
      return reachabilityConstraintIndex;
   }

   public int getDynamicRelaxationIndex()
   {
      return dynamicRelaxationIndex;
   }

   public int getLagrangeMultiplierIndex()
   {
      return lagrangeMultiplierIndex;
   }
}
