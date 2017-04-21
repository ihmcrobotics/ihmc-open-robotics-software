package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

public class ICPQPIndexHandler
{
   private int numberOfFootstepsToConsider = 0;
   private int numberOfFreeVariables = 0;
   private int numberOfFootstepVariables = 0;

   private final static int numberOfEqualityConstraints = 2; // these are the dynamics

   private final int footstepStartingIndex = 0;
   private int feedbackCMPIndex;
   private int dynamicRelaxationIndex;

   private boolean useStepAdjustment;

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
      numberOfFreeVariables = numberOfFootstepVariables + 4; // all the footstep locations, the CMP delta, and the dynamic relaxation

      feedbackCMPIndex = footstepStartingIndex + numberOfFootstepVariables; // this variable is stored after the footsteps

      dynamicRelaxationIndex = feedbackCMPIndex + 2; // this variable is stored after the feedback value
   }

   public int getNumberOfEqualityConstraints()
   {
      return numberOfEqualityConstraints;
   }

   public int getFootstepStartIndex()
   {
      return 0;
   }

   public int getFootstepIndex(int footstepIndex)
   {
      return footstepStartingIndex + 2 * footstepIndex;
   }

   public int getFeedbackCMPIndex()
   {
      return feedbackCMPIndex;
   }

   public int getDynamicRelaxationIndex()
   {
      return dynamicRelaxationIndex;
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
