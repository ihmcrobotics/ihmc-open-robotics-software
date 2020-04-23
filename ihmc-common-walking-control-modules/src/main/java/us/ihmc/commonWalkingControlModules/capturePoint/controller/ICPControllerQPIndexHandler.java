package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * Class intended to manage the indices of all components used in the {@link ICPOptimizationQPSolver}.
 */
public class ICPControllerQPIndexHandler
{
   private static final String prefix = "indexHandler";
   /** The total number of free variables for consideration in the optimization. */
   private int numberOfFreeVariables = 0;

   /** Index for the CoP Feedback action term. */
   private int copFeedbackIndex;
   /** Index for the CMP feedback term. */
   private int cmpFeedbackIndex;

   /** Whether or not to include cmp feedback task in the optimization. */
   private final YoBoolean hasCMPFeedbackTask;
   /** Whether or not to use angular momentum during feedback. This means the CMP will be constrained to being in the support polygon. */
   private final YoBoolean useAngularMomentum;

   public ICPControllerQPIndexHandler(YoVariableRegistry registry)
   {
      hasCMPFeedbackTask = new YoBoolean(prefix + "HasCMPFeedbackTask", registry);
      useAngularMomentum = new YoBoolean(prefix + "UseAngularMomentum", registry);
      hasCMPFeedbackTask.set(false);
      useAngularMomentum.set(false);
   }

   /**
    * Sets whether or not to use CMP feedback in the optimization.
    * @param hasCMPFeedbackTask whether or not to use CMP feedback
    */
   public void setHasCMPFeedbackTask(boolean hasCMPFeedbackTask)
   {
      this.hasCMPFeedbackTask.set(hasCMPFeedbackTask);
   }

   /**
    * Sets whether or not to use angular momentum in the optimization.
    * @param useAngularMomentum whether or not to use angular momentum
    */
   public void setUseAngularMomentum(boolean useAngularMomentum)
   {
      this.useAngularMomentum.set(useAngularMomentum);
   }

   /**
    * Whether or not the solver should include the CMP feedback.
    * @return whether or not to has CMP feedback task.
    */
   public boolean hasCMPFeedbackTask()
   {
      return hasCMPFeedbackTask.getBooleanValue();
   }

   /**
    * Whether or not the solver should use angular momentum with feedback.
    * @return whether or not use angular momentum.
    */
   public boolean useAngularMomentum()
   {
      return useAngularMomentum.getBooleanValue();
   }

   /**
    * Computes the total problem size for the optimization. Should be called after all steps have been registered using {@link #registerFootstep()}.
    */
   public void computeProblemSize()
   {
      copFeedbackIndex = 0;
      cmpFeedbackIndex = copFeedbackIndex + 2;

      numberOfFreeVariables = 2; // the CMP delta
      if (hasCMPFeedbackTask.getBooleanValue())
         numberOfFreeVariables += 2;
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
    * Gets the total number of free variables to be considered by the optimization.
    *
    * @return number of free variables.
    */
   public int getNumberOfFreeVariables()
   {
      return numberOfFreeVariables;
   }
}
