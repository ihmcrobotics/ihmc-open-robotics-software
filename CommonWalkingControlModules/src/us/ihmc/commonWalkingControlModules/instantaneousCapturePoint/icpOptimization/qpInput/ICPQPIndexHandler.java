package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

public class ICPQPIndexHandler
{
   private int numberOfFootstepsToConsider;
   private int numberOfCMPVertices = 0;
   private int numberOfReachabilityVertices = 0;
   private int numberOfFreeVariables = 0;
   private int numberOfFootstepVariables = 0;
   private int numberOfLagrangeMultipliers = 2;

   private int problemSize;
   private int numberOfEqualityConstraints;
   private int numberOfInequalityConstraints;

   private int feedbackCMPIndex;
   private int dynamicRelaxationIndex;
   private int cmpConstraintIndex;
   private int reachabilityConstraintIndex;
   private int lagrangeMultiplierIndex;

   private boolean useStepAdjustment;

   private final boolean useEfficientConstraints;

   public ICPQPIndexHandler(boolean useEfficientConstraints)
   {
      this.useEfficientConstraints = useEfficientConstraints;
   }

   public void resetConstraints()
   {
      numberOfCMPVertices = 0;
      numberOfReachabilityVertices = 0;
   }

   public void resetSupportPolygonConstraint()
   {
      numberOfCMPVertices = 0;
   }

   public void resetReachabilityConstraint()
   {
      numberOfReachabilityVertices = 0;
   }

   public void registerCMPVertex()
   {
      numberOfCMPVertices++;
   }

   public int getNumberOfCMPVertices()
   {
      return numberOfCMPVertices;
   }

   public boolean constrainCMP()
   {
      return numberOfCMPVertices > 0;
   }

   public void registerReachabilityVertex()
   {
      numberOfReachabilityVertices++;
   }

   public int getNumberOfReachabilityVertices()
   {
      return numberOfReachabilityVertices;
   }

   public boolean constrainReachability()
   {
      return numberOfReachabilityVertices > 0;
   }

   public void resetFootsteps()
   {
      useStepAdjustment = false;
      numberOfFootstepsToConsider = 0;
   }

   public void registerFootstep()
   {
      useStepAdjustment = true;
      numberOfFootstepsToConsider++;
   }

   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider;
   }

   public boolean useStepAdjustment()
   {
      return useStepAdjustment;
   }

   public void computeProblemSize()
   {
      numberOfFootstepVariables = 2 * numberOfFootstepsToConsider;

      numberOfLagrangeMultipliers = 2;
      numberOfFreeVariables = numberOfFootstepVariables + 2;

      feedbackCMPIndex = numberOfFootstepVariables;
      dynamicRelaxationIndex = feedbackCMPIndex + 2;

      if (!useEfficientConstraints)
      {
         if (numberOfCMPVertices > 0)
            numberOfLagrangeMultipliers += 3;

         if (numberOfReachabilityVertices > 0)
            numberOfLagrangeMultipliers += 3;
      }

      numberOfFreeVariables += 2;

      cmpConstraintIndex = dynamicRelaxationIndex + 2;
      if (!useEfficientConstraints)
      {
         reachabilityConstraintIndex = cmpConstraintIndex + numberOfCMPVertices;
         lagrangeMultiplierIndex = reachabilityConstraintIndex + numberOfReachabilityVertices;

         problemSize = numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices;
      }
      else
      {
         reachabilityConstraintIndex = cmpConstraintIndex;
         lagrangeMultiplierIndex = reachabilityConstraintIndex;

         problemSize = numberOfFreeVariables;
      }

      numberOfEqualityConstraints = numberOfLagrangeMultipliers;
      numberOfInequalityConstraints = numberOfCMPVertices + numberOfReachabilityVertices;
   }

   public int getProblemSize()
   {
      return problemSize;
   }

   public int getNumberOfEqualityConstraints()
   {
      return numberOfEqualityConstraints;
   }

   public int getNumberOfInequalityConstraints()
   {
      return numberOfInequalityConstraints;
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

   public int getNumberOfFreeVariables()
   {
      return numberOfFreeVariables;
   }
}
