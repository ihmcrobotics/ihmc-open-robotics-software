package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.EntryCMPRecursionMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.recursion.ExitCMPRecursionMultiplier;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.*;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;

public class ICPOptimizationSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double betaSmoothing = 0.01;
   private static final double betaMinimum = 0.0001;

   private final ICPQPIndexHandler indexHandler;
   private final ICPQPInputCalculator inputCalculator;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;
   protected final DenseMatrix64F solverInputResidualCost;

   private final ICPQPInput feedbackTaskInput;
   private final ICPQPInput dynamicRelaxationTask;
   private final ICPQPInput footstepTaskInput;

   private final DynamicsConstraintInput dynamicsConstraintInput;
   private final ConstraintToConvexRegion cmpLocationConstraint;
   private final ConstraintToConvexRegion reachabilityConstraint;

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_AeqTrans;
   protected final DenseMatrix64F solverInput_beq;

   protected final DenseMatrix64F solverInput_Aineq;
   protected final DenseMatrix64F solverInput_AineqTrans;
   protected final DenseMatrix64F solverInput_bineq;

   protected final ArrayList<DenseMatrix64F> footstepRecursionMultipliers = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();

   protected final DenseMatrix64F finalICPRecursion = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F cmpConstantEffect = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F currentICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F referenceICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);

   protected final ArrayList<DenseMatrix64F> footstepWeights = new ArrayList<>();
   protected final DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F dynamicRelaxationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   private final SimpleEfficientActiveSetQPSolver activeSetSolver;

   protected final DenseMatrix64F solution;
   protected final DenseMatrix64F freeVariableSolution;
   protected final DenseMatrix64F lagrangeMultiplierSolution;
   protected final DenseMatrix64F footstepLocationSolution;
   protected final DenseMatrix64F feedbackDeltaSolution;
   protected final DenseMatrix64F dynamicRelaxationSolution;

   protected final DenseMatrix64F previousFeedbackDeltaSolution;
   protected final ArrayList<DenseMatrix64F> previousFootstepLocations = new ArrayList<>();

   private final DenseMatrix64F tmpCost;
   private final DenseMatrix64F tmpFootstepCost;
   private final DenseMatrix64F tmpFeedbackCost;

   private final DenseMatrix64F costToGo;
   private final DenseMatrix64F footstepCostToGo;
   private final DenseMatrix64F feedbackCostToGo;
   private final DenseMatrix64F dynamicRelaxationCostToGo;

   private final DenseMatrix64F referenceICPReconstruction = new DenseMatrix64F(2, 1);

   protected final int maximumNumberOfFootstepsToConsider;
   private static final int maximumNumberOfReachabilityVertices = 4;

   private int numberOfIterations;

   private int currentEqualityConstraintIndex;
   private int currentInequalityConstraintIndex;

   private final boolean computeCostToGo;

   private boolean hasFootstepRegularizationTerm = false;
   private boolean hasFeedbackRegularizationTerm = false;

   private final double minimumFootstepWeight;
   private final double minimumFeedbackWeight;

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, boolean computeCostToGo)
   {
      this.computeCostToGo = computeCostToGo;
      indexHandler = new ICPQPIndexHandler();
      inputCalculator = new ICPQPInputCalculator(indexHandler);

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices + 4;
      int maximumNumberOfLagrangeMultipliers = 8;

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      feedbackTaskInput = new ICPQPInput(2);
      dynamicRelaxationTask = new ICPQPInput(2);
      footstepTaskInput = new ICPQPInput(2 * maximumNumberOfFootstepsToConsider);

      dynamicsConstraintInput = new DynamicsConstraintInput(maximumNumberOfFreeVariables);
      cmpLocationConstraint = new ConstraintToConvexRegion(maximumNumberOfCMPVertices);
      reachabilityConstraint = new ConstraintToConvexRegion(maximumNumberOfReachabilityVertices);

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers);
      solverInput_AeqTrans = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      solverInput_Aineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_AineqTrans = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_bineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, 1);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         previousFootstepLocations.add(new DenseMatrix64F(2, 1));

         footstepRecursionMultipliers.add(new DenseMatrix64F(2, 2));
         footstepWeights.add(new DenseMatrix64F(2, 2));
      }

      solution = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      lagrangeMultiplierSolution = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);
      freeVariableSolution = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      footstepLocationSolution = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      feedbackDeltaSolution = new DenseMatrix64F(2, 1);
      dynamicRelaxationSolution = new DenseMatrix64F(2, 1);

      previousFeedbackDeltaSolution = new DenseMatrix64F(2, 1);

      tmpCost = new DenseMatrix64F(maximumNumberOfFreeVariables + maximumNumberOfLagrangeMultipliers, 1);
      tmpFootstepCost = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      tmpFeedbackCost = new DenseMatrix64F(2, 1);
      costToGo = new DenseMatrix64F(1, 1);
      footstepCostToGo = new DenseMatrix64F(1, 1);
      feedbackCostToGo = new DenseMatrix64F(1, 1);
      dynamicRelaxationCostToGo = new DenseMatrix64F(1, 1);

      activeSetSolver = new SimpleEfficientActiveSetQPSolver();
   }

   public void resetSupportPolygonConstraint()
   {
      cmpLocationConstraint.reset();
      indexHandler.resetSupportPolygonConstraint();
   }

   private final FramePoint tmpPoint = new FramePoint();
   public void addSupportPolygonVertex(FramePoint2d vertexLocation, ReferenceFrame frame, double xBuffer, double yBuffer)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);

      if (tmpPoint.getX() > 0.0)
         tmpPoint.setX(tmpPoint.getX() + xBuffer);
      else
         tmpPoint.setX(tmpPoint.getX() - xBuffer);

      if (tmpPoint.getY() > 0.0)
         tmpPoint.setY(tmpPoint.getY() + yBuffer);
      else
         tmpPoint.setY(tmpPoint.getY() - yBuffer);

      tmpPoint.changeFrame(worldFrame);

      cmpLocationConstraint.addVertex(tmpPoint);
      indexHandler.registerCMPVertex();
   }

   public void resetReachabilityConstraint()
   {
      reachabilityConstraint.reset();
      indexHandler.resetReachabilityConstraint();
   }

   public void addReachabilityVertex(FramePoint2d vertexLocation, ReferenceFrame frame)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);
      tmpPoint.changeFrame(worldFrame);

      reachabilityConstraint.addVertex(tmpPoint);
      indexHandler.registerReachabilityVertex();
   }




   private void reset()
   {
      solverInput_H.zero();
      solverInput_h.zero();
      solverInputResidualCost.zero();

      solverInput_Aeq.zero();
      solverInput_AeqTrans.zero();
      solverInput_beq.zero();

      solverInput_Aineq.zero();
      solverInput_AineqTrans.zero();
      solverInput_bineq.zero();

      dynamicRelaxationTask.reset();
      dynamicsConstraintInput.reset();

      footstepTaskInput.reset();
      feedbackTaskInput.reset();

      finalICPRecursion.zero();
      cmpConstantEffect.zero();
      currentICP.zero();
      referenceICP.zero();
      perfectCMP.zero();

      solution.zero();
      lagrangeMultiplierSolution.zero();
      freeVariableSolution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      dynamicRelaxationSolution.zero();

      currentEqualityConstraintIndex = 0;
      currentInequalityConstraintIndex = 0;
   }

   private void reshape()
   {
      int problemSize = indexHandler.getProblemSize();
      int numberOfInequalityConstraints = indexHandler.getNumberOfInequalityConstraints();
      int numberOfEqualityConstraints = indexHandler.getNumberOfEqualityConstraints();
      int numberOfFootstepsToConsider = indexHandler.getNumberOfFootstepsToConsider();

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      feedbackTaskInput.reshape(2);
      dynamicRelaxationTask.reshape(2);
      footstepTaskInput.reshape(2 * numberOfFootstepsToConsider);

      solverInput_Aeq.reshape(problemSize, numberOfEqualityConstraints);
      solverInput_AeqTrans.reshape(numberOfEqualityConstraints, problemSize);
      solverInput_beq.reshape(numberOfEqualityConstraints, 1);

      solverInput_Aineq.reshape(problemSize, numberOfInequalityConstraints);
      solverInput_AineqTrans.reshape(numberOfInequalityConstraints, problemSize);
      solverInput_bineq.reshape(numberOfInequalityConstraints, 1);

      dynamicsConstraintInput.reshape(problemSize);

      solution.reshape(problemSize + numberOfEqualityConstraints, 1);
      freeVariableSolution.reshape(problemSize, 1);
      lagrangeMultiplierSolution.reshape(numberOfEqualityConstraints, 1);
      footstepLocationSolution.reshape(2 * numberOfFootstepsToConsider, 1);
   }

   public void resetFeedbackConditions()
   {
      feedbackWeight.zero();
      feedbackGain.zero();
      dynamicRelaxationWeight.zero();

      hasFeedbackRegularizationTerm = false;
   }

   public void resetFootstepConditions()
   {
      indexHandler.resetFootsteps();

      footstepRegularizationWeight.zero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         footstepRecursionMultipliers.get(i).zero();
         footstepWeights.get(i).zero();
      }

      hasFootstepRegularizationTerm = false;
   }

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);
   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, FramePoint2d referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, weight, referenceFootstepLocation);
   }

   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double xWeight, double yWeight, FramePoint2d referenceFootstepLocation)
   {
      CommonOps.setIdentity(identity);
      MatrixTools.setMatrixBlock(footstepRecursionMultipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);

      xWeight = Math.max(minimumFootstepWeight, xWeight);
      yWeight = Math.max(minimumFootstepWeight, yWeight);

      identity.zero();
      identity.set(0, 0, xWeight);
      identity.set(1, 1, yWeight);
      MatrixTools.setMatrixBlock(footstepWeights.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, 1.0);

      referenceFootstepLocation.changeFrame(worldFrame);
      referenceFootstepLocations.get(footstepIndex).set(0, 0, referenceFootstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, 0, referenceFootstepLocation.getY());

      indexHandler.registerFootstep();
   }

   public void setFootstepRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(footstepRegularizationWeight);
      CommonOps.scale(regularizationWeight, footstepRegularizationWeight);

      hasFootstepRegularizationTerm = true;
   }

   public void resetFootstepRegularization(int footstepIndex, FramePoint2d previousFootstepLocation)
   {
      previousFootstepLocation.changeFrame(worldFrame);
      previousFootstepLocations.get(footstepIndex).set(0, 0, previousFootstepLocation.getX());
      previousFootstepLocations.get(footstepIndex).set(1, 0, previousFootstepLocation.getY());
   }

   protected void addStepAdjustmentTask()
   {
      for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
      {
         inputCalculator.computeFootstepTask(i, footstepTaskInput, footstepWeights.get(i), referenceFootstepLocations.get(i));

         if (hasFootstepRegularizationTerm)
            inputCalculator.computeFootstepRegularizationTask(i, footstepTaskInput, footstepRegularizationWeight, previousFootstepLocations.get(i));
      }

      inputCalculator.submitFootstepTask(footstepTaskInput, solverInput_H, solverInput_h);
   }



   public void setFeedbackConditions(double feedbackWeight, double feedbackGain, double dynamicRelaxationWeight)
   {
      this.setFeedbackConditions(feedbackWeight, feedbackWeight, feedbackGain, feedbackGain, dynamicRelaxationWeight);
   }

   public void setFeedbackConditions(double feedbackXWeight, double feedbackYWeight, double feedbackXGain, double feedbackYGain, double dynamicRelaxationWeight)
   {
      feedbackXWeight = Math.max(feedbackXWeight, minimumFeedbackWeight);
      feedbackYWeight = Math.max(feedbackYWeight, minimumFeedbackWeight);

      this.feedbackWeight.zero();
      this.feedbackWeight.set(0, 0, feedbackXWeight);
      this.feedbackWeight.set(1, 1, feedbackYWeight);

      this.feedbackGain.zero();
      this.feedbackGain.set(0, 0, feedbackXGain);
      this.feedbackGain.set(1, 1, feedbackYGain);

      CommonOps.setIdentity(this.dynamicRelaxationWeight);
      CommonOps.scale(dynamicRelaxationWeight, this.dynamicRelaxationWeight);
   }

   public void setFeedbackRegularizationWeight(double regularizationWeight)
   {
      CommonOps.setIdentity(feedbackRegularizationWeight);
      CommonOps.scale(regularizationWeight, feedbackRegularizationWeight);

      hasFeedbackRegularizationTerm = true;
   }

   public void resetFeedbackRegularization()
   {
      previousFeedbackDeltaSolution.zero();
   }







   public void compute(FramePoint2d finalICPRecursion, FramePoint2d cmpConstantEffect, FramePoint2d currentICP, FramePoint2d perfectCMP) throws NoConvergenceException
   {
      indexHandler.computeProblemSize();

      reset();
      reshape();

      finalICPRecursion.changeFrame(worldFrame);
      cmpConstantEffect.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);

      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());

      this.currentICP.set(0, 0, currentICP.getX());
      this.currentICP.set(1, 0, currentICP.getY());

      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      this.cmpConstantEffect.set(0, 0, cmpConstantEffect.getX());
      this.cmpConstantEffect.set(1, 0, cmpConstantEffect.getY());

      addFeedbackTask();
      addDynamicRelaxationTask();
      addDynamicConstraint();

      if (indexHandler.constrainCMP())
         addCMPLocationConstraint();

      if (indexHandler.constrainReachability())
         addReachabilityConstraint();

      if (indexHandler.useStepAdjustment())
         addStepAdjustmentTask();

      NoConvergenceException noConvergenceException = null;
      try
      {
         solve(solution);
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
         throw noConvergenceException;
      }

      if (noConvergenceException == null)
      {
         extractLagrangeMultiplierSolution(lagrangeMultiplierSolution);
         extractFreeVariableSolution(freeVariableSolution);

         if (indexHandler.useStepAdjustment())
         {
            extractFootstepSolutions(footstepLocationSolution);
            setPreviousFootstepSolution(footstepLocationSolution);
         }

         extractFeedbackDeltaSolution(feedbackDeltaSolution);
         setPreviousFeedbackDeltaSolution(feedbackDeltaSolution);
         extractDynamicRelaxationSolution(dynamicRelaxationSolution);

         if (computeCostToGo)
            computeCostToGo();

         reconstructReferenceICPPosition();
      }
   }

   protected void addFeedbackTask()
   {
      inputCalculator.computeFeedbackTask(feedbackTaskInput, feedbackWeight);

      if (hasFeedbackRegularizationTerm)
         inputCalculator.computeFeedbackRegularizationTask(feedbackTaskInput, feedbackRegularizationWeight, previousFeedbackDeltaSolution);

      inputCalculator.submitFeedbackTask(feedbackTaskInput, solverInput_H, solverInput_h);
   }

   protected void addDynamicRelaxationTask()
   {
      inputCalculator.computeDynamicRelaxationTask(dynamicRelaxationTask, dynamicRelaxationWeight);
      inputCalculator.submitDynamicRelaxationTask(dynamicRelaxationTask, solverInput_H, solverInput_h);
   }

   private void addCMPLocationConstraint()
   {
      cmpLocationConstraint.setPositionOffset(perfectCMP);
      cmpLocationConstraint.setIndexOfVariableToConstrain(indexHandler.getFeedbackCMPIndex());
      cmpLocationConstraint.setSmoothingWeight(betaSmoothing / indexHandler.getNumberOfCMPVertices());
      cmpLocationConstraint.setVertexMinimum(betaMinimum / indexHandler.getNumberOfCMPVertices());
      cmpLocationConstraint.formulateConstraint();

      int numberOfVertices = cmpLocationConstraint.getNumberOfVertices();
      int cmpContstraintIndex = indexHandler.getCMPConstraintIndex();
      MatrixTools.addMatrixBlock(solverInput_H, cmpContstraintIndex, cmpContstraintIndex, cmpLocationConstraint.quadraticTerm, 0, 0, numberOfVertices, numberOfVertices, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, cmpLocationConstraint.getIndexOfVariableToConstrain(), currentEqualityConstraintIndex, cmpLocationConstraint.indexSelectionMatrix, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_Aeq, cmpContstraintIndex, currentEqualityConstraintIndex, cmpLocationConstraint.dynamics_Aeq, 0, 0, numberOfVertices, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, cmpLocationConstraint.dynamics_beq, 0, 0, 2, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, cmpContstraintIndex, currentEqualityConstraintIndex + 2, cmpLocationConstraint.sum_Aeq, 0, 0, numberOfVertices, 1, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex + 2, 0, cmpLocationConstraint.sum_beq, 0, 0, 1, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aineq, indexHandler.getCMPConstraintIndex(), currentInequalityConstraintIndex, cmpLocationConstraint.Aineq, 0, 0, numberOfVertices, numberOfVertices, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, cmpLocationConstraint.bineq, 0, 0, numberOfVertices, 1, 1.0);

      currentEqualityConstraintIndex += 3;
      currentInequalityConstraintIndex += cmpLocationConstraint.getNumberOfVertices();
   }

   private void addReachabilityConstraint()
   {
      reachabilityConstraint.setIndexOfVariableToConstrain(indexHandler.getFootstepStartIndex());
      reachabilityConstraint.setSmoothingWeight(betaSmoothing / indexHandler.getNumberOfReachabilityVertices());
      reachabilityConstraint.setVertexMinimum(betaMinimum / indexHandler.getNumberOfReachabilityVertices());
      reachabilityConstraint.formulateConstraint();

      int numberOfVertices = reachabilityConstraint.getNumberOfVertices();
      int reachabilityConstraintIndex = indexHandler.getReachabilityConstraintIndex();
      MatrixTools.addMatrixBlock(solverInput_H, reachabilityConstraintIndex, reachabilityConstraintIndex, reachabilityConstraint.quadraticTerm, 0, 0, numberOfVertices, numberOfVertices, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, indexHandler.getFootstepStartIndex(), currentEqualityConstraintIndex, reachabilityConstraint.indexSelectionMatrix, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_Aeq, reachabilityConstraintIndex, currentEqualityConstraintIndex, reachabilityConstraint.dynamics_Aeq, 0, 0, numberOfVertices, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, reachabilityConstraint.dynamics_beq, 0, 0, 2, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, reachabilityConstraintIndex, currentEqualityConstraintIndex + 2, reachabilityConstraint.sum_Aeq, 0, 0, numberOfVertices, 1, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex + 2, 0, reachabilityConstraint.sum_beq, 0, 0, 1, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aineq, reachabilityConstraintIndex, currentInequalityConstraintIndex, reachabilityConstraint.Aineq, 0, 0, numberOfVertices, numberOfVertices, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, reachabilityConstraint.bineq, 0, 0, numberOfVertices, 1, 1.0);

      currentEqualityConstraintIndex += 3;
      currentInequalityConstraintIndex += reachabilityConstraint.getNumberOfVertices();
   }

   private void addDynamicConstraint()
   {
      inputCalculator.computeDynamicsConstraint(dynamicsConstraintInput, currentICP, finalICPRecursion, cmpConstantEffect, feedbackGain, footstepRecursionMultipliers);

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, currentEqualityConstraintIndex, dynamicsConstraintInput.Aeq, 0, 0, indexHandler.getNumberOfFreeVariables(), 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, dynamicsConstraintInput.beq, 0, 0, 2, 1, 1.0);

      currentEqualityConstraintIndex += 2;
   }


   private void solve(DenseMatrix64F solutionToPack) throws NoConvergenceException
   {
      CommonOps.scale(-1.0, solverInput_h);

      activeSetSolver.clear();

      CommonOps.transpose(solverInput_Aeq, solverInput_AeqTrans);
      CommonOps.transpose(solverInput_Aineq, solverInput_AineqTrans);

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, 0.0);
      activeSetSolver.setLinearEqualityConstraints(solverInput_AeqTrans, solverInput_beq);
      activeSetSolver.setLinearInequalityConstraints(solverInput_AineqTrans, solverInput_bineq);

      numberOfIterations = activeSetSolver.solve(solutionToPack);

      if (MatrixTools.containsNaN(solutionToPack))
         throw new NoConvergenceException(numberOfIterations);
   }

   private void extractFootstepSolutions(DenseMatrix64F footstepLocationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(footstepLocationSolutionToPack, 0, 0, solution, 0, 0, indexHandler.getNumberOfFootstepVariables(), 1, 1.0);
   }

   private void extractFeedbackDeltaSolution(DenseMatrix64F feedbackSolutionToPack)
   {
      MatrixTools.setMatrixBlock(feedbackSolutionToPack, 0, 0, solution, indexHandler.getFeedbackCMPIndex(), 0, 2, 1, 1.0);
   }

   private void extractDynamicRelaxationSolution(DenseMatrix64F dynamicRelaxationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(dynamicRelaxationSolutionToPack, 0, 0, solution, indexHandler.getDynamicRelaxationIndex(), 0, 2, 1, 1.0);
   }

   private void extractLagrangeMultiplierSolution(DenseMatrix64F lagrangeMultiplierSolutionToPack)
   {
      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutionToPack, 0, 0, solution, indexHandler.getLagrangeMultiplierIndex(), 0, indexHandler.getNumberOfEqualityConstraints(), 1, 1.0);
   }

   private void extractFreeVariableSolution(DenseMatrix64F freeVariableSolution)
   {
      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, indexHandler.getNumberOfFreeVariables(), 1, 1.0);
   }

   protected void setPreviousFootstepSolution(DenseMatrix64F footstepLocationSolution)
   {
      for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
         MatrixTools.setMatrixBlock(previousFootstepLocations.get(i), 0, 0, footstepLocationSolution, 2 * i, 0, 2, 1, 1.0);
   }

   private void setPreviousFeedbackDeltaSolution(DenseMatrix64F feedbackDeltaSolution)
   {
      previousFeedbackDeltaSolution.set(feedbackDeltaSolution);
   }

   private final DenseMatrix64F tmpCostScalar = new DenseMatrix64F(1, 1);
   private void computeCostToGo()
   {
      costToGo.zero();
      footstepCostToGo.zero();
      feedbackCostToGo.zero();
      dynamicRelaxationCostToGo.zero();

      tmpCost.zero();
      tmpFootstepCost.zero();
      tmpFeedbackCost.zero();

      tmpCost.reshape(indexHandler.getProblemSize(), 1);
      tmpFootstepCost.reshape(indexHandler.getNumberOfFootstepVariables(), 1);
      tmpFeedbackCost.reshape(2, 1);

      // quadratic cost;
      CommonOps.mult(solverInput_H, freeVariableSolution, tmpCost);
      CommonOps.multTransA(freeVariableSolution, tmpCost, costToGo);

      CommonOps.mult(footstepTaskInput.quadraticTerm, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepCostToGo);

      CommonOps.mult(feedbackTaskInput.quadraticTerm, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackCostToGo);

      CommonOps.mult(dynamicRelaxationTask.quadraticTerm, dynamicRelaxationSolution, tmpFeedbackCost);
      CommonOps.multTransA(dynamicRelaxationSolution, tmpFeedbackCost, dynamicRelaxationCostToGo);

      // linear cost
      CommonOps.multTransA(solverInput_h, freeVariableSolution, tmpCostScalar);
      CommonOps.addEquals(costToGo, tmpCostScalar);

      CommonOps.multTransA(-1.0, footstepTaskInput.linearTerm, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepCostToGo, tmpCostScalar);

      CommonOps.multTransA(-1.0, feedbackTaskInput.linearTerm, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackCostToGo, tmpCostScalar);

      CommonOps.multTransA(-1.0, dynamicRelaxationTask.linearTerm, dynamicRelaxationSolution, tmpCostScalar);
      CommonOps.addEquals(dynamicRelaxationCostToGo, tmpCostScalar);

      // residual cost
      CommonOps.addEquals(costToGo, solverInputResidualCost);
      CommonOps.addEquals(footstepCostToGo, footstepTaskInput.residualCost);
      CommonOps.addEquals(feedbackCostToGo, feedbackTaskInput.residualCost);
      CommonOps.addEquals(dynamicRelaxationCostToGo, dynamicRelaxationTask.residualCost);
   }

   private final DenseMatrix64F tmpVector = new DenseMatrix64F(2, 1);
   private void reconstructReferenceICPPosition()
   {
      referenceICPReconstruction.set(finalICPRecursion);
      CommonOps.addEquals(referenceICPReconstruction, cmpConstantEffect);
      CommonOps.addEquals(referenceICPReconstruction, dynamicRelaxationSolution);

      for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
      {
         tmpVector.set(0, 0, footstepLocationSolution.get(2 * i));
         tmpVector.set(1, 0, footstepLocationSolution.get(2 * i + 1));
         CommonOps.scale(footstepRecursionMultipliers.get(i).get(0, 0), tmpVector);

         CommonOps.addEquals(referenceICPReconstruction, tmpVector);
      }
   }

   public void getFootstepSolutionLocation(int footstepIndex, FramePoint2d footstepLocationToPack)
   {
      footstepLocationToPack.setToZero(worldFrame);
      footstepLocationToPack.setX(footstepLocationSolution.get(2 * footstepIndex, 0));
      footstepLocationToPack.setY(footstepLocationSolution.get(2 * footstepIndex + 1, 0));
   }

   public void getCMPFeedbackDifference(FrameVector2d cmpFeedbackDifferenceToPack)
   {
      cmpFeedbackDifferenceToPack.setToZero(worldFrame);
      cmpFeedbackDifferenceToPack.setX(feedbackDeltaSolution.get(0, 0));
      cmpFeedbackDifferenceToPack.setY(feedbackDeltaSolution.get(1, 0));
   }

   public void getReconstructedReferenceICPPosition(FramePoint2d reconstructedReferenceICPToPack)
   {
      reconstructedReferenceICPToPack.setX(referenceICPReconstruction.get(0, 0));
      reconstructedReferenceICPToPack.setY(referenceICPReconstruction.get(1, 0));
   }

   public void getDynamicRelaxation(FramePoint2d dynamicRelaxationToPack)
   {
      dynamicRelaxationToPack.setX(dynamicRelaxationSolution.get(0, 0));
      dynamicRelaxationToPack.setY(dynamicRelaxationSolution.get(1, 0));
   }

   public double getCostToGo()
   {
      return costToGo.get(0, 0);
   }

   public double getFootstepCostToGo()
   {
      return footstepCostToGo.get(0, 0);
   }

   public double getFeedbackCostToGo()
   {
      return feedbackCostToGo.get(0, 0);
   }

   public double getDynamicRelaxationCostToGo()
   {
      return dynamicRelaxationCostToGo.get(0, 0);
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }
}
