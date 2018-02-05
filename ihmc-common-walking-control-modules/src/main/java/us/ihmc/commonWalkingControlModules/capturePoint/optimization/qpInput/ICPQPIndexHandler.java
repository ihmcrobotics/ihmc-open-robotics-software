package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;

/**
 * Class intended to manage the indices of all components used in the {@link ICPOptimizationQPSolver}.
 */
public class ICPQPIndexHandler
{
   /** Number of footsteps registered with the solver. */
   private int numberOfFootstepsToConsider = 0;
   /** The total number of free variables for consideration in the optimization. */
   private int numberOfFreeVariables = 0;
   /** Number of footstep variables registered with the solver. Should be two times {@link #numberOfFootstepsToConsider} */
   private int numberOfFootstepVariables = 0;

   /** Index for the start of the footstep variables. */
   private int footstepStartingIndex;
   /** Index for the CoP Feedback action term. */
   private int copFeedbackIndex;
   /** Index for the CMP feedback term. */
   private int cmpFeedbackIndex;

   /** Whether or not to use step adjustment in the optimization. If {@link #numberOfFootstepsToConsider} is 0, this term should be false */
   private boolean useStepAdjustment;
   /** Whether or not to include cmp feedback task in the optimization. */
   private boolean hasCMPFeedbackTask = false;
   /** Whether or not to use angular momentum during feedback. This means the CMP will be constrained to being in the support polygon. */
   private boolean useAngularMomentum = false;

   /**
    * Resets the number of footsteps for the controller to consider.
    */
   public void resetFootsteps()
   {
      useStepAdjustment = false;
      numberOfFootstepsToConsider = 0;
   }

   /**
    * Registers that a footstep was added to the solver.
    */
   public void registerFootstep()
   {
      useStepAdjustment = true;
      numberOfFootstepsToConsider++;
   }

   /**
    * Gets the total number of footsteps to consider in the current optimization tick.
    *
    * @return number of footsteps
    */
   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider;
   }

   /**
    * Whether or not the solver should include the recursive dynamics of the upcoming footsteps.
    *
    * @return whether or not to use step adjustment.
    */
   public boolean useStepAdjustment()
   {
      return useStepAdjustment;
   }

   /**
    * Sets whether or not to use CMP feedback in the optimization.
    * @param hasCMPFeedbackTask whether or not to use CMP feedback
    */
   public void setHasCMPFeedbackTask(boolean hasCMPFeedbackTask)
   {
      this.hasCMPFeedbackTask = hasCMPFeedbackTask;
   }

   /**
    * Sets whether or not to use angular momentum in the optimization.
    * @param useAngularMomentum whether or not to use angular momentum
    */
   public void setUseAngularMomentum(boolean useAngularMomentum)
   {
      this.useAngularMomentum = useAngularMomentum;
   }

   /**
    * Whether or not the solver should include the CMP feedback.
    * @return whether or not to has CMP feedback task.
    */
   public boolean hasCMPFeedbackTask()
   {
      return hasCMPFeedbackTask;
   }

   /**
    * Whether or not the solver should use angular momentum with feedback.
    * @return whether or not use angular momentum.
    */
   public boolean useAngularMomentum()
   {
      return useAngularMomentum;
   }

   /**
    * Computes the total problem size for the optimization. Should be called after all steps have been registered using {@link #registerFootstep()}.
    */
   public void computeProblemSize()
   {
      copFeedbackIndex = 0;
      cmpFeedbackIndex = copFeedbackIndex + 2;
      if (hasCMPFeedbackTask)
         footstepStartingIndex = cmpFeedbackIndex + 2;
      else
         footstepStartingIndex = copFeedbackIndex + 2;

      numberOfFootstepVariables = 2 * numberOfFootstepsToConsider;
      numberOfFreeVariables = 2; // the CMP delta
      if (useStepAdjustment)
         numberOfFreeVariables += numberOfFootstepVariables; // all the footstep locations
      if (hasCMPFeedbackTask)
         numberOfFreeVariables += 2;
   }

   /**
    * Gets the index for the first footstep location.
    *
    * @return footstep start index.
    */
   public int getFootstepStartIndex()
   {
      return footstepStartingIndex;
   }

   /**
    * Gets the index for the footstep location of the footstep indicated by {@param footstepIndex}.
    *
    * @param footstepIndex footstep in question.
    * @return footstep index number
    */
   public int getFootstepIndex(int footstepIndex)
   {
      return footstepStartingIndex + 2 * footstepIndex;
   }

   /**
    * Gets the index of the CoP feedback action term.
    *
    * @return cmp feedback action index.
    */
   public int getCoPFeedbackIndex()
   {
      return copFeedbackIndex;
   }

   /**
    * Gets the index of the cmp feedback term.
    *
    * @return cmp feedback index.
    */
   public int getCMPFeedbackIndex()
   {
      return cmpFeedbackIndex;
   }

   /**
    * Gets the total number of variables to be used to describe the footstep locations.
    *
    * @return number of footstep variables.
    */
   public int getNumberOfFootstepVariables()
   {
      return numberOfFootstepVariables;
   }

   /**
    * Gets the total number of free variables to be considered by the optimization.
    *
    * @return number of free variables.
    */
   public int getNumberOfFreeVariables()
   {
      return numberOfFreeVariables;
   }
}
