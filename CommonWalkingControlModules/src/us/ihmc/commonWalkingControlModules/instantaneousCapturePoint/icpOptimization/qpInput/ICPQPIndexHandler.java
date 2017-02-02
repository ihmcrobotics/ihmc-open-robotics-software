package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

public class ICPQPIndexHandler
{
   private int numberOfFootstepsToConsider;
   private int numberOfCMPVertices = 0;
   private int numberOfReachabilityVertices = 0;
   private int numberOfFreeVariables = 0;
   private int numberOfFootstepVariables = 0;
   private int numberOfLagrangeMultipliers = 2;

   private int feedbackCMPIndex;
   private int dynamicRelaxationIndex;
   private int cmpConstraintIndex;
   private int reachabilityConstraintIndex;
   private int lagrangeMultiplierIndex;

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

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment)
   {
      if (!useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;
      else
         this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      numberOfFootstepVariables = 2 * this.numberOfFootstepsToConsider;

      numberOfLagrangeMultipliers = 2;
      numberOfFreeVariables = numberOfFootstepVariables + 2;

      feedbackCMPIndex = numberOfFootstepVariables;
      dynamicRelaxationIndex = feedbackCMPIndex + 2;

      if (numberOfCMPVertices > 0)
         numberOfLagrangeMultipliers += 3;

      numberOfFreeVariables += 2;

      if (numberOfReachabilityVertices > 0)
         numberOfLagrangeMultipliers += 3;

      cmpConstraintIndex = dynamicRelaxationIndex + 2;
      reachabilityConstraintIndex = cmpConstraintIndex + numberOfCMPVertices;
      lagrangeMultiplierIndex = reachabilityConstraintIndex + numberOfReachabilityVertices;
   }

   public int getFootstepStartIndex()
   {
      return 0;
   }

   public int getFootstepIndex(int footstepIndex)
   {
      return 2 * footstepIndex;
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

   public int getNumberOfFootstepVariables()
   {
      return numberOfFootstepVariables;
   }
}
