package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.DynamicRelaxationTaskInput;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.FeedbackTaskInput;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.ICPQPIndexHandler;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.ICPQPInputCalculator;
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
   private final YoMatrix footstepH;
   private final YoMatrix footsteph;
   private final YoMatrix footstepReferenceLocation;

   private final YoMatrix yoDynamics_Aeq;
   private final YoMatrix yoDynamics_beq;
   private final YoMatrix yoSolver_Aeq;
   private final YoMatrix yoSolver_beq;

   private final YoMatrix yoStanceCMP_Aeq;
   private final YoMatrix yoStanceCMP_beq;
   private final YoMatrix yoStanceCMPDynamics_Aeq;
   private final YoMatrix yoStanceCMPDynamics_beq;
   private final YoMatrix yoStanceCMPSum_Aeq;
   private final YoMatrix yoStanceCMPSum_beq;

   private final ICPQPIndexHandler indexHandler;
   private final ICPQPInputCalculator inputCalculator;

   protected final DenseMatrix64F solverInput_H;
   protected final DenseMatrix64F solverInput_h;
   protected final DenseMatrix64F solverInputResidualCost;

   protected final DenseMatrix64F footstepCost_H;
   protected final DenseMatrix64F footstepCost_h;
   protected final DenseMatrix64F footstepResidualCost;

   protected final DenseMatrix64F footstepRegularizationCost_H;
   protected final DenseMatrix64F footstepRegularizationCost_h;
   protected final DenseMatrix64F footstepRegularizationResidualCost;


   private final FeedbackTaskInput feedbackTaskInput;
   private final DynamicRelaxationTaskInput dynamicRelaxationTask;

   protected final DenseMatrix64F stanceCMPCost_G;

   protected final DenseMatrix64F solverInput_Aeq;
   protected final DenseMatrix64F solverInput_AeqTrans;
   protected final DenseMatrix64F solverInput_beq;

   protected final DenseMatrix64F solverInput_Aineq;
   protected final DenseMatrix64F solverInput_AineqTrans;
   protected final DenseMatrix64F solverInput_bineq;

   protected final DenseMatrix64F dynamics_Aeq;
   protected final DenseMatrix64F dynamics_beq;

   protected final DenseMatrix64F stanceCMP_Aeq;
   protected final DenseMatrix64F stanceCMP_beq;
   protected final DenseMatrix64F stanceCMPDynamics_Aeq;
   protected final DenseMatrix64F stanceCMPDynamics_beq;
   protected final DenseMatrix64F stanceCMPSum_Aeq;
   protected final DenseMatrix64F stanceCMPSum_beq;
   protected final DenseMatrix64F stanceCMP_Aineq;
   protected final DenseMatrix64F stanceCMP_bineq;

   protected final DenseMatrix64F reachabilityCost_G;

   protected final DenseMatrix64F reachability_Aeq;
   protected final DenseMatrix64F reachability_beq;
   protected final DenseMatrix64F reachabilityDynamics_Aeq;
   protected final DenseMatrix64F reachabilityDynamics_beq;
   protected final DenseMatrix64F reachabilitySum_Aeq;
   protected final DenseMatrix64F reachabilitySum_beq;
   protected final DenseMatrix64F reachability_Aineq;
   protected final DenseMatrix64F reachability_bineq;

   protected final ArrayList<DenseMatrix64F> cmpVertexLocations = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> reachabilityVertexLocations = new ArrayList<>();

   protected final ArrayList<DenseMatrix64F> footstepRecursionMutlipliers = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> referenceFootstepLocations = new ArrayList<>();
   protected final ArrayList<DenseMatrix64F> previousFootstepLocations = new ArrayList<>();
   private final DenseMatrix64F footstepObjectiveVector;

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
   private final int maximumNumberOfCMPVertices;
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
      inputCalculator = new ICPQPInputCalculator(icpOptimizationParameters, maximumNumberOfCMPVertices);

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      this.maximumNumberOfCMPVertices = maximumNumberOfCMPVertices;

      minimumFootstepWeight = icpOptimizationParameters.getMinimumFootstepWeight();
      minimumFeedbackWeight = icpOptimizationParameters.getMinimumFeedbackWeight();

      feedbackWeightHardeningMultiplier = icpOptimizationParameters.getFeedbackWeightHardeningMultiplier();

      int maximumNumberOfFreeVariables = 2 * maximumNumberOfFootstepsToConsider + maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices + 4;
      int maximumNumberOfLagrangeMultipliers = 8;

      solverInput_H = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfFreeVariables);
      solverInput_h = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      solverInputResidualCost = new DenseMatrix64F(1, 1);

      footstepCost_H = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      footstepCost_h = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      footstepResidualCost = new DenseMatrix64F(1, 1);

      footstepRegularizationCost_H = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider);
      footstepRegularizationCost_h = new DenseMatrix64F(2 * maximumNumberOfFootstepsToConsider, 1);
      footstepRegularizationResidualCost = new DenseMatrix64F(1, 1);

      feedbackTaskInput = new FeedbackTaskInput();
      dynamicRelaxationTask = new DynamicRelaxationTaskInput();

      solverInput_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers);
      solverInput_AeqTrans = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, maximumNumberOfFreeVariables);
      solverInput_beq = new DenseMatrix64F(maximumNumberOfLagrangeMultipliers, 1);

      dynamics_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 2);
      dynamics_beq = new DenseMatrix64F(2, 1);

      stanceCMPCost_G = new DenseMatrix64F(maximumNumberOfCMPVertices, maximumNumberOfCMPVertices);
      stanceCMP_Aeq = new DenseMatrix64F(4 + maximumNumberOfCMPVertices, 3);
      stanceCMP_beq = new DenseMatrix64F(3, 1);
      stanceCMPDynamics_Aeq = new DenseMatrix64F(4 + maximumNumberOfCMPVertices, 2);
      stanceCMPDynamics_beq = new DenseMatrix64F(2, 1);
      stanceCMPSum_Aeq = new DenseMatrix64F(maximumNumberOfCMPVertices, 1);
      stanceCMPSum_beq = new DenseMatrix64F(1, 1);

      stanceCMP_Aineq = new DenseMatrix64F(maximumNumberOfCMPVertices, maximumNumberOfCMPVertices);
      stanceCMP_bineq = new DenseMatrix64F(maximumNumberOfCMPVertices, 1);

      reachabilityCost_G = new DenseMatrix64F(maximumNumberOfReachabilityVertices, maximumNumberOfReachabilityVertices);
      reachability_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 3);
      reachability_beq = new DenseMatrix64F(3, 1);
      reachabilityDynamics_Aeq = new DenseMatrix64F(maximumNumberOfFreeVariables, 1);
      reachabilityDynamics_beq = new DenseMatrix64F(2, 1);
      reachabilitySum_Aeq = new DenseMatrix64F(maximumNumberOfReachabilityVertices, 1);
      reachabilitySum_beq = new DenseMatrix64F(2, 1);

      reachability_Aineq = new DenseMatrix64F(maximumNumberOfReachabilityVertices, maximumNumberOfReachabilityVertices);
      reachability_bineq = new DenseMatrix64F(maximumNumberOfReachabilityVertices, 1);

      solverInput_Aineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_AineqTrans = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices);
      solverInput_bineq = new DenseMatrix64F(maximumNumberOfCMPVertices + maximumNumberOfReachabilityVertices, 1);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.add(new DenseMatrix64F(2, 1));
         previousFootstepLocations.add(new DenseMatrix64F(2, 1));

         footstepRecursionMutlipliers.add(new DenseMatrix64F(2, 2));
         footstepWeights.add(new DenseMatrix64F(2, 2));
      }
      footstepObjectiveVector =  new DenseMatrix64F(2 * maximumNumberOfFreeVariables, 1);

      for (int i = 0; i < maximumNumberOfCMPVertices; i++)
      {
         cmpVertexLocations.add(new DenseMatrix64F(2, 1));
      }

      for (int i = 0; i < maximumNumberOfReachabilityVertices; i++)
      {
         reachabilityVertexLocations.add(new DenseMatrix64F(2, 1));
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

         footstepH = new YoMatrix("footstepQuadraticCost", 2 * maximumNumberOfFootstepsToConsider, 2 * maximumNumberOfFootstepsToConsider, registry);
         footsteph = new YoMatrix("footstepLinearCost", 2 * maximumNumberOfFootstepsToConsider, 1, registry);

         yoDynamics_Aeq = new YoMatrix("dynamics_Aeq", maximumNumberOfFreeVariables, 2, registry);
         yoDynamics_beq = new YoMatrix("dynamics_beq", 2, 1, registry);

         yoSolver_Aeq = new YoMatrix("solver_Aeq", maximumNumberOfFreeVariables, maximumNumberOfLagrangeMultipliers, registry);
         yoSolver_beq = new YoMatrix("solver_beq", maximumNumberOfLagrangeMultipliers, 1, registry);

         yoStanceCMP_Aeq = new YoMatrix("stanceCMP_Aeq", 4 + maximumNumberOfCMPVertices, 3, registry);
         yoStanceCMP_beq = new YoMatrix("stanceCMP_beq", 3, 1, registry);
         yoStanceCMPDynamics_Aeq = new YoMatrix("stanceCMPDynamics_Aeq", 4 + maximumNumberOfCMPVertices, 2, registry);
         yoStanceCMPDynamics_beq = new YoMatrix("stanceCMPDynamics_beq", 2, 1, registry);
         yoStanceCMPSum_Aeq = new YoMatrix("stanceCMPSum_Aeq", maximumNumberOfCMPVertices, 1, registry);
         yoStanceCMPSum_beq = new YoMatrix("stanceCMPSum_beq", 1, 1, registry);

         footstepReferenceLocation = new YoMatrix("footstepReferenceLocation", 2 * maximumNumberOfFootstepsToConsider, 1, registry);

         parentRegistry.addChild(registry);
      }
      else
      {
         localDebug = false;

         yoWeightG = null;
         yoWeightg = null;

         footstepH = null;
         footsteph = null;

         yoDynamics_Aeq = null;
         yoDynamics_beq = null;

         yoSolver_Aeq = null;
         yoSolver_beq = null;

         yoStanceCMP_Aeq = null;
         yoStanceCMP_beq = null;
         yoStanceCMPDynamics_Aeq = null;
         yoStanceCMPDynamics_beq = null;
         yoStanceCMPSum_Aeq = null;
         yoStanceCMPSum_beq = null;

         footstepReferenceLocation = null;
      }
   }

   public void setNumberOfCMPVertices(int numberOfCMPVertices)
   {
      this.numberOfCMPVertices = numberOfCMPVertices;
      indexHandler.setNumberOfCMPVertices(numberOfCMPVertices);

      stanceCMPCost_G.zero();
      stanceCMP_Aeq.zero();
      stanceCMP_beq.zero();
      stanceCMPDynamics_Aeq.zero();
      stanceCMPDynamics_beq.zero();
      stanceCMPSum_Aeq.zero();
      stanceCMPSum_beq.zero();

      stanceCMP_Aineq.zero();
      stanceCMP_bineq.zero();

      stanceCMPCost_G.reshape(numberOfCMPVertices, numberOfCMPVertices);
      stanceCMP_Aeq.reshape(4 + numberOfCMPVertices, 3);
      stanceCMPDynamics_Aeq.reshape(4 + numberOfCMPVertices, 2);
      stanceCMPSum_Aeq.reshape(numberOfCMPVertices, 1);

      stanceCMP_Aineq.reshape(numberOfCMPVertices, numberOfCMPVertices);
      stanceCMP_bineq.reshape(numberOfCMPVertices, 1);

      for (int i = 0; i < maximumNumberOfCMPVertices; i++)
      {
         cmpVertexLocations.get(i).zero();
      }
   }

   public void setNumberOfReachabilityVertices(int numberOfReachabilityVertices)
   {
      indexHandler.setNumberOfReachabilityVertices(numberOfReachabilityVertices);
      this.numberOfReachabilityVertices = numberOfReachabilityVertices;

      reachabilityCost_G.zero();
      reachability_Aeq.zero();
      reachability_beq.zero();
      reachabilityDynamics_Aeq.zero();
      reachabilityDynamics_beq.zero();
      reachabilitySum_Aeq.zero();
      reachabilitySum_beq.zero();

      reachability_Aineq.zero();
      reachability_bineq.zero();

      reachabilityCost_G.reshape(numberOfReachabilityVertices, numberOfReachabilityVertices);
      reachability_Aeq.reshape(2 * maximumNumberOfFootstepsToConsider + 4 + numberOfCMPVertices + numberOfReachabilityVertices, 3);
      reachabilityDynamics_Aeq.reshape(2 * maximumNumberOfFootstepsToConsider + 4 + numberOfCMPVertices + numberOfReachabilityVertices, 2);
      reachabilitySum_Aeq.reshape(numberOfReachabilityVertices, 1);

      reachability_Aineq.reshape(numberOfReachabilityVertices, numberOfReachabilityVertices);
      reachability_bineq.reshape(numberOfReachabilityVertices, 1);

      for (int i = 0; i < maximumNumberOfReachabilityVertices; i++)
         reachabilityVertexLocations.get(i).zero();
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

      footstepCost_H.zero();
      footstepCost_h.zero();
      footstepResidualCost.zero();

      footstepRegularizationCost_H.zero();
      footstepRegularizationCost_h.zero();
      footstepRegularizationResidualCost.zero();

      feedbackTaskInput.reset();
      dynamicRelaxationTask.reset();

      solverInput_Aeq.zero();
      solverInput_AeqTrans.zero();
      solverInput_beq.zero();

      solverInput_Aineq.zero();
      solverInput_AineqTrans.zero();
      solverInput_bineq.zero();

      dynamics_Aeq.zero();
      dynamics_beq.zero();

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         referenceFootstepLocations.get(i).zero();
         footstepRecursionMutlipliers.get(i).zero();
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

      footstepCost_H.reshape(numberOfFootstepVariables, numberOfFootstepVariables);
      footstepCost_h.reshape(numberOfFootstepVariables, 1);

      footstepRegularizationCost_H.reshape(numberOfFootstepVariables, numberOfFootstepVariables);
      footstepRegularizationCost_h.reshape(numberOfFootstepVariables, 1);

      solverInput_Aeq.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, numberOfLagrangeMultipliers);
      solverInput_AeqTrans.reshape(numberOfLagrangeMultipliers, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_beq.reshape(numberOfLagrangeMultipliers, 1);

      solverInput_Aineq.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_AineqTrans.reshape(numberOfCMPVertices + numberOfReachabilityVertices, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices);
      solverInput_bineq.reshape(numberOfCMPVertices + numberOfReachabilityVertices, 1);

      dynamics_Aeq.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 2);

      solution.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices + numberOfLagrangeMultipliers, 1);
      freeVariableSolution.reshape(numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 1);
      lagrangeMultiplierSolution.reshape(numberOfLagrangeMultipliers, 1);
      footstepLocationSolution.reshape(numberOfFootstepVariables, 1);
      footstepObjectiveVector.reshape(numberOfFootstepVariables, 1);
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
      MatrixTools.setMatrixBlock(footstepRecursionMutlipliers.get(footstepIndex), 0, 0, identity, 0, 0, 2, 2, recursionMultiplier);
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
   public void setSupportPolygonVertex(int vertexIndex, FramePoint2d vertexLocation, ReferenceFrame frame, double xBuffer, double yBuffer)
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

      cmpVertexLocations.get(vertexIndex).set(0, 0, tmpPoint.getX());
      cmpVertexLocations.get(vertexIndex).set(1, 0, tmpPoint.getY());
   }

   public void setReachabilityVertex(int vertexIndex, FramePoint2d vertexLocation, ReferenceFrame frame)
   {
      tmpPoint.setToZero(frame);
      tmpPoint.setXY(vertexLocation);
      tmpPoint.changeFrame(worldFrame);

      reachabilityVertexLocations.get(vertexIndex).set(0, 0, tmpPoint.getX());
      reachabilityVertexLocations.get(vertexIndex).set(1, 0, tmpPoint.getY());
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

         if (hasFootstepRegularizationTerm)
            addFootstepRegularizationTask();
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

   private final DenseMatrix64F tmpFootstepObjective = new DenseMatrix64F(2, 1);
   protected void addStepAdjustmentTask()
   {
      footstepObjectiveVector.zero();
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.setMatrixBlock(footstepCost_H, 2 * i, 2 * i, footstepWeights.get(i), 0, 0, 2, 2, 1.0);

         tmpFootstepObjective.zero();
         tmpFootstepObjective.set(referenceFootstepLocations.get(i));
         CommonOps.mult(footstepWeights.get(i), tmpFootstepObjective, tmpFootstepObjective);
         CommonOps.multTransA(referenceFootstepLocations.get(i), tmpFootstepObjective, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepCost_h, 2 * i, 0, tmpFootstepObjective, 0, 0, 2, 1, 1.0);
         CommonOps.addEquals(solverInputResidualCost, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepObjectiveVector, 2 * i, 0, referenceFootstepLocations.get(i), 0, 0, 2, 1, 1.0);
      }

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepCost_H, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepCost_h, 0, 0, numberOfFootstepVariables, 1, 1.0);

      if (localDebug)
      {
         footstepReferenceLocation.set(footstepObjectiveVector);
         footstepH.set(footstepCost_H);
         footsteph.set(footstepCost_h);
      }
   }

   private final DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);
   protected void addFootstepRegularizationTask()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.setMatrixBlock(footstepRegularizationCost_H, 2 * i, 2 * i, footstepRegularizationWeight, 0, 0, 2, 2, 1.0);

         tmpObjective.zero();
         tmpObjective.set(previousFootstepLocations.get(i));
         CommonOps.mult(footstepRegularizationWeight, tmpObjective, tmpObjective);
         CommonOps.multTransA(previousFootstepLocations.get(i), tmpObjective, footstepRegularizationResidualCost);

         MatrixTools.setMatrixBlock(footstepRegularizationCost_h, 2 * i, 0, tmpObjective, 0, 0, 2, 1, 1.0);
         CommonOps.addEquals(solverInputResidualCost, footstepRegularizationResidualCost);
      }

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, footstepRegularizationCost_H, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, footstepRegularizationCost_h, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }

   private void addCMPLocationConstraint()
   {
      computeCMPLocationConstraint();

      CommonOps.setIdentity(stanceCMPCost_G);
      CommonOps.scale(betaSmoothing, stanceCMPCost_G);

      MatrixTools.setMatrixBlock(solverInput_H, numberOfFreeVariables, numberOfFreeVariables, stanceCMPCost_G, 0, 0, numberOfCMPVertices, numberOfCMPVertices, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, indexHandler.getFeedbackCMPIndex(), currentEqualityConstraintIndex, stanceCMP_Aeq, 0, 0, 4 + numberOfCMPVertices, 3, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, stanceCMP_beq, 0, 0, 3, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aineq, indexHandler.getCMPConstraintIndex(), currentInequalityConstraintIndex, stanceCMP_Aineq, 0, 0, numberOfCMPVertices, numberOfCMPVertices, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, stanceCMP_bineq, 0, 0, numberOfCMPVertices, 1, 1.0);

      currentEqualityConstraintIndex += 3;
      currentInequalityConstraintIndex += numberOfCMPVertices;
   }

   private void computeCMPLocationConstraint()
   {
      // set up location constraints
      stanceCMPDynamics_Aeq.set(0, 0, -1.0);
      stanceCMPDynamics_Aeq.set(1, 1, -1.0);

      int offset = indexHandler.getCMPConstraintIndex() - indexHandler.getFeedbackCMPIndex();
      for (int i = 0; i < numberOfCMPVertices; i++)
      {
         stanceCMPDynamics_Aeq.set(offset + i, 0, cmpVertexLocations.get(i).get(0, 0));
         stanceCMPDynamics_Aeq.set(offset + i, 1, cmpVertexLocations.get(i).get(1, 0));

         stanceCMPSum_Aeq.set(i, 0, 1.0);

         stanceCMP_Aineq.set(i, i, -1.0);
      }

      stanceCMPDynamics_beq.set(perfectCMP);

      stanceCMPSum_beq.set(0, 0, 1.0);

      MatrixTools.setMatrixBlock(stanceCMP_Aeq, 0, 0, stanceCMPDynamics_Aeq, 0, 0, 4 + numberOfCMPVertices, 2, 1.0);
      MatrixTools.setMatrixBlock(stanceCMP_Aeq, offset, 2, stanceCMPSum_Aeq, 0, 0, numberOfCMPVertices, 1, 1.0);

      MatrixTools.setMatrixBlock(stanceCMP_beq, 0, 0, stanceCMPDynamics_beq, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(stanceCMP_beq, 2, 0, stanceCMPSum_beq, 0, 0, 1, 1, 1.0);
   }

   private void addReachabilityConstraint()
   {
      computeReachabilityConstraint();

      CommonOps.setIdentity(reachabilityCost_G);
      CommonOps.scale(betaSmoothing, reachabilityCost_G);

      int reachabilityConstraintIndex = indexHandler.getReachabilityConstraintIndex();
      MatrixTools.setMatrixBlock(solverInput_H, reachabilityConstraintIndex, reachabilityConstraintIndex, reachabilityCost_G, 0, 0,
            numberOfReachabilityVertices, numberOfReachabilityVertices, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, currentEqualityConstraintIndex, reachability_Aeq, 0, 0, numberOfFreeVariables + numberOfCMPVertices + numberOfReachabilityVertices, 3, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, reachability_beq, 0, 0, 3, 1, 1.0);

      MatrixTools.setMatrixBlock(solverInput_Aineq, reachabilityConstraintIndex, currentInequalityConstraintIndex, reachability_Aineq, 0, 0, numberOfReachabilityVertices, numberOfReachabilityVertices, 1.0);
      MatrixTools.setMatrixBlock(solverInput_bineq, currentInequalityConstraintIndex, 0, reachability_bineq, 0, 0, numberOfReachabilityVertices, 1, 1.0);

      currentEqualityConstraintIndex += 3;
      currentInequalityConstraintIndex += numberOfReachabilityVertices;
   }

   private void computeReachabilityConstraint()
   {
      // set up location constraints
      reachabilityDynamics_Aeq.set(0, 0, -1.0);
      reachabilityDynamics_Aeq.set(1, 1, -1.0);

      int offset = indexHandler.getReachabilityConstraintIndex();
      for (int i = 0; i < numberOfReachabilityVertices; i++)
      {
         reachabilityDynamics_Aeq.set(offset + i, 0, reachabilityVertexLocations.get(i).get(0, 0));
         reachabilityDynamics_Aeq.set(offset + i, 1, reachabilityVertexLocations.get(i).get(1, 0));

         reachabilitySum_Aeq.set(i, 0, 1.0);

         reachability_Aineq.set(i, i, -1.0);
      }

      reachabilitySum_beq.set(0, 0, 1.0);

      MatrixTools.setMatrixBlock(reachability_Aeq, 0, 0, reachabilityDynamics_Aeq, 0, 0, numberOfFootstepVariables + 4 + numberOfCMPVertices + numberOfReachabilityVertices, 2, 1.0);
      MatrixTools.setMatrixBlock(reachability_Aeq, offset, 2, reachabilitySum_Aeq, 0, 0, numberOfReachabilityVertices, 1, 1.0);

      MatrixTools.setMatrixBlock(reachability_beq, 0, 0, reachabilityDynamics_beq, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(reachability_beq, 2, 0, reachabilitySum_beq, 0, 0, 1, 1, 1.0);
   }

   private void addDynamicConstraint()
   {
      computeDynamicConstraint();

      MatrixTools.setMatrixBlock(solverInput_Aeq, 0, currentEqualityConstraintIndex, dynamics_Aeq, 0, 0, numberOfFreeVariables, 2, 1.0);
      MatrixTools.setMatrixBlock(solverInput_beq, currentEqualityConstraintIndex, 0, dynamics_beq, 0, 0, 2, 1, 1.0);

      currentEqualityConstraintIndex += 2;
   }

   private void computeDynamicConstraint()
   {
      addDynamicRelaxationToDynamicConstraint();

      if (useFeedback)
         addFeedbackToDynamicConstraint();
      if (useStepAdjustment)
         addFootstepRecursionsToDynamicConstraint();

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, stanceCMPProjection);
      CommonOps.subtractEquals(currentICP, initialICPProjection);

      if (useTwoCMPs)
         CommonOps.subtractEquals(currentICP, cmpOffsetRecursionEffect);

      MatrixTools.setMatrixBlock(dynamics_beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
   }

   private void addFeedbackToDynamicConstraint()
   {
      CommonOps.invert(feedbackGain);

      MatrixTools.setMatrixBlock(dynamics_Aeq, indexHandler.getFeedbackCMPIndex(), 0, feedbackGain, 0, 0, 2, 2, 1.0);
   }

   private void addDynamicRelaxationToDynamicConstraint()
   {
      CommonOps.setIdentity(identity);
      MatrixTools.setMatrixBlock(dynamics_Aeq, indexHandler.getDynamicRelaxationIndex(), 0, identity, 0, 0, 2, 2, 1.0);
   }

   private void addFootstepRecursionsToDynamicConstraint()
   {
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         MatrixTools.setMatrixBlock(dynamics_Aeq, 2 * i, 0, footstepRecursionMutlipliers.get(i), 0, 0, 2, 2, 1.0);
      }
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
         yoDynamics_Aeq.set(dynamics_Aeq);
         yoDynamics_beq.set(dynamics_beq);
         yoSolver_Aeq.set(solverInput_Aeq);
         yoSolver_beq.set(solverInput_beq);
         yoStanceCMP_Aeq.set(stanceCMP_Aeq);
         yoStanceCMP_beq.set(stanceCMP_beq);
         yoStanceCMPDynamics_Aeq.set(stanceCMPDynamics_Aeq);
         yoStanceCMPDynamics_beq.set(stanceCMPDynamics_beq);
         yoStanceCMPSum_Aeq.set(stanceCMPSum_Aeq);
         yoStanceCMPSum_beq.set(stanceCMPSum_beq);
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

      CommonOps.mult(footstepCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepCostToGo);

      CommonOps.mult(footstepRegularizationCost_H, footstepLocationSolution, tmpFootstepCost);
      CommonOps.multTransA(footstepLocationSolution, tmpFootstepCost, footstepRegularizationCostToGo);

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

      CommonOps.multTransA(-1.0, footstepCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepCostToGo, tmpCostScalar);

      CommonOps.multTransA(-1.0, footstepRegularizationCost_h, footstepLocationSolution, tmpCostScalar);
      CommonOps.addEquals(footstepRegularizationCostToGo, tmpCostScalar);

      CommonOps.multTransA(-1.0, feedbackTaskInput.linearTerm, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackCostToGo, tmpCostScalar);

      /*
      CommonOps.multTransA(-1.0, feedbackRegularizationCost_h, feedbackDeltaSolution, tmpCostScalar);
      CommonOps.addEquals(feedbackRegularizationCostToGo, tmpCostScalar);
      */

      // residual cost
      CommonOps.addEquals(costToGo, solverInputResidualCost);
      CommonOps.addEquals(footstepCostToGo, footstepResidualCost);
      CommonOps.addEquals(footstepRegularizationCostToGo, footstepRegularizationResidualCost);
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
