package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.*;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.exceptions.NoConvergenceException;

import java.util.ArrayList;

public class ICPOptimizationSolver
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean DEBUG = false;
   private final boolean localDebug;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double betaSmoothing = 0.01;

   private final YoMatrix yoWeightG;
   private final YoMatrix yoWeightg;

   private final YoMatrix yoSolver_Aeq;
   private final YoMatrix yoSolver_beq;

   private final ICPQPIndexHandler indexHandler;
   private final ICPQPInputCalculator inputCalculator;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;
   protected final DenseMatrix64F solverInputResidualCost;

   private final FeedbackTaskInput feedbackTaskInput;
   private final DynamicRelaxationTaskInput dynamicRelaxationTask;
   private final FootstepTaskInput footstepTaskInput;

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
   protected final ArrayList<DenseMatrix64F> previousFootstepLocations = new ArrayList<>();

   protected final DenseMatrix64F finalICPRecursion = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F cmpOffsetRecursionEffect = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F stanceCMPProjection = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F initialICPProjection = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F currentICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F referenceICP = new DenseMatrix64F(2, 1);
   protected final DenseMatrix64F perfectCMP = new DenseMatrix64F(2, 1);

   protected final ArrayList<DenseMatrix64F> footstepWeights = new ArrayList<>();
   protected final DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F dynamicRelaxationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackRegularizationWeight = new DenseMatrix64F(2, 2);
   protected final DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);

   private final SimpleEfficientActiveSetQPSolver activeSetSolver;

   protected final DenseMatrix64F solution;
   protected final DenseMatrix64F freeVariableSolution;
   protected final DenseMatrix64F lagrangeMultiplierSolution;
   protected final DenseMatrix64F footstepLocationSolution;
   protected final DenseMatrix64F feedbackDeltaSolution;
   protected final DenseMatrix64F dynamicRelaxationSolution;
   protected final DenseMatrix64F previousFeedbackDeltaSolution;

   private final DenseMatrix64F tmpCost;
   private final DenseMatrix64F tmpFootstepCost;
   private final DenseMatrix64F tmpFeedbackCost;
   private final DenseMatrix64F costToGo;
   private final DenseMatrix64F footstepCostToGo;
   private final DenseMatrix64F footstepRegularizationCostToGo;
   private final DenseMatrix64F feedbackCostToGo;
   private final DenseMatrix64F feedbackRegularizationCostToGo;
   private final DenseMatrix64F dynamicRelaxationCostToGo;

   protected final int maximumNumberOfFootstepsToConsider;
   private final int maximumNumberOfReachabilityVertices = 4;

   protected int numberOfFootstepsToConsider;
   protected int numberOfCMPVertices = 0;
   protected int numberOfReachabilityVertices = 0;
   protected int numberOfFreeVariables = 0;
   protected int numberOfFootstepVariables = 0;
   protected int numberOfLagrangeMultipliers = 2;

   private int numberOfIterations;

   private int currentEqualityConstraintIndex;
   private int currentInequalityConstraintIndex;

   private boolean useFeedback = false;
   private boolean useStepAdjustment = true;
   private boolean useTwoCMPs = false;

   private boolean hasFootstepRegularizationTerm = false;
   private boolean hasFeedbackRegularizationTerm = false;

   private final double minimumFootstepWeight;
   private final double minimumFeedbackWeight;

   private final double feedbackWeightHardeningMultiplier;

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices)
   {
      this(icpOptimizationParameters, maximumNumberOfCMPVertices, null);
   }

   public ICPOptimizationSolver(ICPOptimizationParameters icpOptimizationParameters, int maximumNumberOfCMPVertices, YoVariableRegistry parentRegistry)
   {
      indexHandler = new ICPQPIndexHandler();
      inputCalculator = new ICPQPInputCalculator(indexHandler);

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      feedbackWeightHardeningMultiplier = icpOptimizationParameters.getFeedbackWeightHardeningMultiplier();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices + 4;
      int maximumNumberOfLagrangeMultipliers = 8;

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      feedbackTaskInput = new FeedbackTaskInput();
      dynamicRelaxationTask = new DynamicRelaxationTaskInput();
      footstepTaskInput = new FootstepTaskInput(maximumNumberOfFootstepsToConsider);

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
      footstepRegularizationCostToGo = new DenseMatrix64F(1, 1);
      feedbackCostToGo = new DenseMatrix64F(1, 1);
      feedbackRegularizationCostToGo = new DenseMatrix64F(1, 1);
      dynamicRelaxationCostToGo = new DenseMatrix64F(1, 1);

      activeSetSolver = new SimpleEfficientActiveSetQPSolver();

      if (parentRegistry != null && DEBUG)
      {
         localDebug = true;

         yoWeightG = new YoMatrix("solverQuadraticCost", maximumNumberOfFreeVariables, maximumNumberOfFreeVariables, registry);
         yoWeightg = new YoMatrix("solverLinearCost", maximumNumberOfFreeVariables, 1, registry);

         yoSolver_Aeq = new YoMatrix("solver_Aeq", maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers, registry);
         yoSolver_beq = new YoMatrix("solver_beq", maximumNumberOfLagrangeMultipliers, 1, registry);

         parentRegistry.addChild(registry);
      }
      else
      {
         localDebug = false;

         yoWeightG = null;
         yoWeightg = null;

         yoSolver_Aeq = null;
         yoSolver_beq = null;
      }
   }

   public void setNumberOfCMPVertices(int numberOfCMPVertices)
   {
      this.numberOfCMPVertices = numberOfCMPVertices;
      indexHandler.setNumberOfCMPVertices(numberOfCMPVertices);

      cmpLocationConstraint.reset();
   }

   public void setNumberOfReachabilityVertices(int numberOfReachabilityVertices)
   {
      indexHandler.setNumberOfReachabilityVertices(numberOfReachabilityVertices);
      this.numberOfReachabilityVertices = numberOfReachabilityVertices;

      reachabilityConstraint.reset();
   }

   public void submitProblemConditions(int numberOfFootstepsToConsider, boolean useStepAdjustment, boolean useFeedback, boolean useTwoCMPs)
   {
      if (!useFeedback && (!useStepAdjustment || numberOfFootstepsToConsider < 1))
      {
         throw new RuntimeException("No possible feedback mechanism available.");
      }

      indexHandler.submitProblemConditions(numberOfFootstepsToConsider, useStepAdjustment, useFeedback, useTwoCMPs);

      this.useFeedback = useFeedback;
      this.useStepAdjustment = useStepAdjustment;
      this.useTwoCMPs = useTwoCMPs;

      if (useFeedback && !useStepAdjustment)
         this.numberOfFootstepsToConsider = 0;
      else
         this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;

      numberOfFootstepVariables = 2 * this.numberOfFootstepsToConsider;

      numberOfLagrangeMultipliers = 2;
      numberOfFreeVariables = numberOfFootstepVariables + 2;
      if (useFeedback)
      {
         if (numberOfCMPVertices > 0)
            numberOfLagrangeMultipliers += 3;

         numberOfFreeVariables += 2;
      }
      else
      {
         numberOfCMPVertices = 0;
      }

      if (numberOfReachabilityVertices > 0)
         numberOfLagrangeMultipliers += 3;

      reset();
      reshape();
   }

   private void reset()
   {
      solverInput_H.zero();
      solverInput_h.zero();
      solverInputResidualCost.zero();

      footstepTaskInput.reset();
      feedbackTaskInput.reset();
      dynamicRelaxationTask.reset();

      dynamicsConstraintInput.reset();

      solverInput_Aeq.zero();
      solverInput_AeqTrans.zero();
      solverInput_beq.zero();

      solverInput_Aineq.zero();
      solverInput_AineqTrans.zero();
      solverInput_bineq.zero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         footstepRecursionMultipliers.get(i).zero();
         footstepWeights.get(i).zero();
      }

      finalICPRecursion.zero();
      initialICPProjection.zero();
      cmpOffsetRecursionEffect.zero();
      currentICP.zero();
      referenceICP.zero();
      perfectCMP.zero();

      footstepRegularizationWeight.zero();
      feedbackWeight.zero();
      feedbackGain.zero();
      dynamicRelaxationWeight.zero();

      solution.zero();
      lagrangeMultiplierSolution.zero();
      freeVariableSolution.zero();
      footstepLocationSolution.zero();
      feedbackDeltaSolution.zero();
      dynamicRelaxationSolution.zero();

      hasFootstepRegularizationTerm = false;
      currentEqualityConstraintIndex = 0;
      currentInequalityConstraintIndex = 0;
   }

   private void reshape()
   {
      solverInput_H.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_h.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 1);

      footstepTaskInput.reshape(numberOfFootstepsToConsider);

      solverInput_Aeq.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, numberOfLagrangeMultipliers);
      solverInput_AeqTrans.reshape(numberOfLagrangeMultipliers, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      solverInput_Aineq.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_AineqTrans.reshape(numberOfCMPVertices + numberOfReachabilityVertices, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_bineq.reshape(numberOfCMPVertices + numberOfReachabilityVertices, 1);

      dynamicsConstraintInput.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);

      solution.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices + numberOfLagrangeMultipliers, 1);
      freeVariableSolution.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 1);
      lagrangeMultiplierSolution.reshape(numberOfLagrangeMultipliers, 1);
      footstepLocationSolution.reshape(numberOfFootstepVariables, 1);
   }

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);
   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double weight, FramePoint2d referenceFootstepLocation)
   {
      this.setFootstepAdjustmentConditions(footstepIndex, recursionMultiplier, weight, weight, referenceFootstepLocation);
   }

   public void setFootstepAdjustmentConditions(int footstepIndex, double recursionMultiplier, double xWeight, double yWeight, FramePoint2d referenceFootstepLocation)
   {
      setFootstepRecursionMutliplier(footstepIndex, recursionMultiplier);
      setFootstepWeight(footstepIndex, xWeight, yWeight);
      setReferenceFootstepLocation(footstepIndex, referenceFootstepLocation);
   }

   private void setFootstepRecursionMutliplier(int footstepIndex, double recursionMultiplier)
   {
      CommonOps.setIdentity(identity);
      MatrixTools.setMatrixBlock(footstepRecursionMultipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);
   }

   protected void setFootstepWeight(int footstepIndex, double xWeight, double yWeight)
   {
      xWeight = Math.max(minimumFootstepWeight, xWeight);
      yWeight = Math.max(minimumFootstepWeight, yWeight);

      identity.zero();
      identity.set(0, 0, xWeight);
      identity.set(1, 1, yWeight);
      MatrixTools.setMatrixBlock(footstepWeights.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, 1.0);
   }

   private void setReferenceFootstepLocation(int footstepIndex, FramePoint2d referenceFootstepLocation)
   {
      referenceFootstepLocation.changeFrame(worldFrame);
      referenceFootstepLocations.get(footstepIndex).set(0, 0, referenceFootstepLocation.getX());
      referenceFootstepLocations.get(footstepIndex).set(1, 0, referenceFootstepLocation.getY());
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

   public void setUseFeedbackWeightHardening()
   {
      double xWeight = feedbackWeight.get(0, 0);
      double yWeight = feedbackWeight.get(1, 1);

      xWeight *= (1.0 + feedbackWeightHardeningMultiplier * Math.abs(previousFeedbackDeltaSolution.get(0, 0)));
      yWeight *= (1.0 + feedbackWeightHardeningMultiplier * Math.abs(previousFeedbackDeltaSolution.get(1, 0)));

      feedbackWeight.set(0, 0, xWeight);
      feedbackWeight.set(1, 1, yWeight);
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

   public void resetFeedbackRegularization()
   {
      previousFeedbackDeltaSolution.zero();
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
   }

   public void addReachabilityVertex(FramePoint2d vertexLocation, ReferenceFrame frame)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);
      tmpPoint.changeFrame(worldFrame);

      reachabilityConstraint.addVertex(tmpPoint);
   }

   public void compute(FramePoint2d finalICPRecursion, FramePoint2d cmpOffsetRecursionEffect, FramePoint2d currentICP, FramePoint2d perfectCMP,
         FramePoint2d stanceCMPProjection, FramePoint2d initialICPProjection) throws NoConvergenceException
   {
      finalICPRecursion.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);
      perfectCMP.changeFrame(worldFrame);
      stanceCMPProjection.changeFrame(worldFrame);

      if (cmpOffsetRecursionEffect != null)
         cmpOffsetRecursionEffect.changeFrame(worldFrame);

      this.finalICPRecursion.set(0, 0, finalICPRecursion.getX());
      this.finalICPRecursion.set(1, 0, finalICPRecursion.getY());

      this.currentICP.set(0, 0, currentICP.getX());
      this.currentICP.set(1, 0, currentICP.getY());

      this.perfectCMP.set(0, 0, perfectCMP.getX());
      this.perfectCMP.set(1, 0, perfectCMP.getY());

      this.stanceCMPProjection.set(0, 0, stanceCMPProjection.getX());
      this.stanceCMPProjection.set(1, 0, stanceCMPProjection.getY());

      this.initialICPProjection.set(0, 0, initialICPProjection.getX());
      this.initialICPProjection.set(1, 0, initialICPProjection.getY());


      if (useTwoCMPs)
      {
         this.cmpOffsetRecursionEffect.set(0, 0, cmpOffsetRecursionEffect.getX());
         this.cmpOffsetRecursionEffect.set(1, 0, cmpOffsetRecursionEffect.getY());
      }

      if (useFeedback)
      {
         addFeedbackTask();
         addDynamicRelaxationTask();

         if (numberOfCMPVertices > 0)
            addCMPLocationConstraint();
      }

      if (numberOfReachabilityVertices > 0)
         addReachabilityConstraint();

      if (useStepAdjustment)
      {
         addStepAdjustmentTask();
      }

      addDynamicConstraint();

      assembleTotalProblem();

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

         if (useStepAdjustment)
         {
            extractFootstepSolutions(footstepLocationSolution);
            setPreviousFootstepSolution(footstepLocationSolution);
         }
         if (useFeedback)
         {
            extractFeedbackDeltaSolution(feedbackDeltaSolution);
            extractDynamicRelaxationSolution(dynamicRelaxationSolution);
            setPreviousFeedbackDeltaSolution(feedbackDeltaSolution);
         }

         computeCostToGo();
      }
   }

   protected void addFeedbackTask()
   {
      inputCalculator.computeFeedbackTask(feedbackTaskInput, feedbackWeight);

      if (hasFeedbackRegularizationTerm)
         inputCalculator.computeFeedbackRegularizationTask(feedbackTaskInput, feedbackRegularizationWeight, previousFeedbackDeltaSolution);
;

      int feedbackCMPIndex = indexHandler.getFeedbackCMPIndex();
      MatrixTools.addMatrixBlock(solverInput_H, feedbackCMPIndex, feedbackCMPIndex, feedbackTaskInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, feedbackCMPIndex, 0, feedbackTaskInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   protected void addDynamicRelaxationTask()
   {
      inputCalculator.computeDynamicRelaxationTask(dynamicRelaxationTask, dynamicRelaxationWeight);

      int dynamicRelaxationIndex = indexHandler.getDynamicRelaxationIndex();
      MatrixTools.addMatrixBlock(solverInput_H, dynamicRelaxationIndex, dynamicRelaxationIndex, dynamicRelaxationTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, dynamicRelaxationIndex, 0, dynamicRelaxationTask.linearTerm, 0, 0, 2, 1, 1.0);
   }

   protected void addStepAdjustmentTask()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         inputCalculator.computeFootstepTask(i, footstepTaskInput, footstepWeights.get(i), referenceFootstepLocations.get(i));

         if (hasFootstepRegularizationTerm)
            inputCalculator.computeFootstepRegularizationTask(i, footstepTaskInput, footstepRegularizationWeight, previousFootstepLocations.get(i));
      }

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepTaskInput.quadraticTerm, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepTaskInput.linearTerm, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }

   private void addCMPLocationConstraint()
   {
      cmpLocationConstraint.setPositionOffset(perfectCMP);
      cmpLocationConstraint.setIndexOfVariableToConstrain(indexHandler.getFeedbackCMPIndex());
      cmpLocationConstraint.setSmoothingWeight(betaSmoothing);
      cmpLocationConstraint.formulateConstraint();

      int numberOfVertices = cmpLocationConstraint.getNumberOfVertices();
      int cmpContstraintIndex = indexHandler.getCMPConstraintIndex();
      MatrixTools.setMatrixBlock(solverInput_H, cmpContstraintIndex, cmpContstraintIndex, cmpLocationConstraint.smoothingCost, 0, 0, numberOfVertices, numberOfVertices, 1.0);

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
      reachabilityConstraint.setIndexOfVariableToConstrain(indexHandler.getFootstepIndex());
      reachabilityConstraint.setSmoothingWeight(betaSmoothing);
      reachabilityConstraint.formulateConstraint();

      int numberOfVertices = reachabilityConstraint.getNumberOfVertices();
      int reachabilityConstraintIndex = indexHandler.getReachabilityConstraintIndex();
      MatrixTools.setMatrixBlock(solverInput_H, reachabilityConstraintIndex, reachabilityConstraintIndex, cmpLocationConstraint.smoothingCost, 0, 0, numberOfVertices, numberOfVertices, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, indexHandler.getFootstepIndex(), currentEqualityConstraintIndex, reachabilityConstraint.indexSelectionMatrix, 0, 0, 2, 2, 1.0);
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
      inputCalculator.computeDynamicsConstraint(dynamicsConstraintInput, currentICP, finalICPRecursion, stanceCMPProjection, initialICPProjection, useTwoCMPs,
            cmpOffsetRecursionEffect, useFeedback, feedbackGain, useStepAdjustment, numberOfFootstepsToConsider, footstepRecursionMultipliers);

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, currentEqualityConstraintIndex, dynamicsConstraintInput.Aeq, 0, 0, numberOfFreeVariables, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, dynamicsConstraintInput.beq, 0, 0, 2, 1, 1.0);

      currentEqualityConstraintIndex += 2;
   }

   private void assembleTotalProblem()
   {
      CommonOps.transpose(solverInput_Aeq, solverInput_AeqTrans);
      CommonOps.transpose(solverInput_Aineq, solverInput_AineqTrans);
   }

   private void solve(DenseMatrix64F solutionToPack) throws NoConvergenceException
   {
      CommonOps.scale(-1.0, solverInput_h);

      activeSetSolver.clear();

      if (localDebug)
      {
         yoWeightG.set(solverInput_H);
         yoWeightg.set(solverInput_h);
         yoSolver_Aeq.set(solverInput_Aeq);
         yoSolver_beq.set(solverInput_beq);
      }

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, 0.0);
      activeSetSolver.setLinearEqualityConstraints(solverInput_AeqTrans, solverInput_beq);
      activeSetSolver.setLinearInequalityConstraints(solverInput_AineqTrans, solverInput_bineq);

      numberOfIterations = activeSetSolver.solve(solutionToPack);

      if (MatrixTools.containsNaN(solutionToPack))
         throw new NoConvergenceException(numberOfIterations);
   }

   private void extractFootstepSolutions(DenseMatrix64F footstepLocationSolutionToPack)
   {
      MatrixTools.setMatrixBlock(footstepLocationSolutionToPack, 0, 0, solution, 0, 0, numberOfFootstepVariables, 1, 1.0);
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
      MatrixTools.setMatrixBlock(lagrangeMultiplierSolutionToPack, 0, 0, solution, indexHandler.getLagrangeMultiplierIndex(), 0, numberOfLagrangeMultipliers, 1, 1.0);
   }

   private void extractFreeVariableSolution(DenseMatrix64F freeVariableSolution)
   {
      MatrixTools.setMatrixBlock(freeVariableSolution, 0, 0, solution, 0, 0, numberOfFreeVariables, 1, 1.0);
   }

   protected void setPreviousFootstepSolution(DenseMatrix64F footstepLocationSolution)
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         MatrixTools.setMatrixBlock(previousFootstepLocations.get(i), 0, 0, footstepLocationSolution, 2 * i, 0, 2, 1, 1.0);
   }

   private void setPreviousFeedbackDeltaSolution(DenseMatrix64F feedbackDeltaSolution)
   {
      MatrixTools.setMatrixBlock(previousFeedbackDeltaSolution, 0, 0, feedbackDeltaSolution, 0, 0, 2, 1, 1.0);
   }

   private final DenseMatrix64F tmpCostScalar = new DenseMatrix64F(1, 1);
   private void computeCostToGo()
   {
      costToGo.zero();
      footstepCostToGo.zero();
      footstepRegularizationCostToGo.zero();
      feedbackCostToGo.zero();
      feedbackRegularizationCostToGo.zero();
      dynamicRelaxationCostToGo.zero();

      tmpCost.zero();
      tmpFootstepCost.zero();
      tmpFeedbackCost.zero();

      tmpCost.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 1);
      tmpFootstepCost.reshape(numberOfFootstepVariables, 1);
      tmpFeedbackCost.reshape(2, 1);

      // quadratic cost;
      CommonOps.mult(solverInput_H, freeVariableSolution, tmpCost);
      CommonOps.multTransA(freeVariableSolution, tmpCost, costToGo);

      /*
      CommonOps.mult(footstepCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepCostToGo);
      */

      /*
      CommonOps.mult(footstepRegularizationCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepRegularizationCostToGo);
      */

      CommonOps.mult(feedbackTaskInput.quadraticTerm, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackCostToGo);

      CommonOps.mult(dynamicRelaxationTask.quadraticTerm, dynamicRelaxationSolution, tmpFeedbackCost);
      CommonOps.multTransA(dynamicRelaxationSolution, tmpFeedbackCost, dynamicRelaxationCostToGo);

      /*
      CommonOps.mult(feedbackRegularizationCost_H, feedbackDeltaSolution, tmpFeedbackCost);
      CommonOps.multTransA(feedbackDeltaSolution, tmpFeedbackCost, feedbackRegularizationCostToGo);
      */

      // linear cost
      CommonOps.multTransA(solverInput_h, freeVariableSolution, tmpCostScalar);
      CommonOps.addEquals(costToGo, tmpCostScalar);

      /*
      CommonOps.multTransA(-1.0, footstepCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepCostToGo, tmpCostScalar);
      */

      /*
      CommonOps.multTransA(-1.0, footstepRegularizationCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepRegularizationCostToGo, tmpCostScalar);
      */

      CommonOps.multTransA(-1.0, feedbackTaskInput.linearTerm, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackCostToGo, tmpCostScalar);

      /*
      CommonOps.multTransA(-1.0, feedbackRegularizationCost_h, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackRegularizationCostToGo, tmpCostScalar);
      */

      // residual cost
      CommonOps.addEquals(costToGo, solverInputResidualCost);
      //CommonOps.addEquals(footstepCostToGo, footstepResidualCost);
      //CommonOps.addEquals(footstepRegularizationCostToGo, footstepRegularizationResidualCost);
      CommonOps.addEquals(feedbackCostToGo, feedbackTaskInput.residualCost);
      //CommonOps.addEquals(feedbackRegularizationCostToGo, feedbackRegularizationResidualCost);
      CommonOps.addEquals(dynamicRelaxationCostToGo, dynamicRelaxationTask.residualCost);
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

   public double getCostToGo()
   {
      return costToGo.get(0, 0);
   }

   public double getFootstepCostToGo()
   {
      return footstepCostToGo.get(0, 0);
   }

   public double getFootstepRegularizationCostToGo()
   {
      return footstepRegularizationCostToGo.get(0, 0);
   }

   public double getFeedbackCostToGo()
   {
      return feedbackCostToGo.get(0, 0);
   }

   public double getFeedbackRegularizationCostToGo()
   {
      return feedbackRegularizationCostToGo.get(0, 0);
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
