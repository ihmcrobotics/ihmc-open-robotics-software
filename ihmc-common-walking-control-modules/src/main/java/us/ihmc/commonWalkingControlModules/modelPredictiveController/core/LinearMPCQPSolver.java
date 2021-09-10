package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.TIntList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeC;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This is the wrapper class for the quadratic program solver. It receives all the MPC Commands, converting them to quadratic costs or linear constraints.
 * It then submits these costs and constraints to a quadratic program solver, and computes the optimal solution. It then provides these solution coefficients
 * as an output.
 */
public class LinearMPCQPSolver
{
   private static  final boolean debug = true;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("mpcSolverTimer", 0.5, registry);
   public final MPCQPSolver qpSolver;

   private final YoBoolean addRateRegularization = new YoBoolean("AddRateRegularization", registry);
   private final YoBoolean foundSolution = new YoBoolean("foundSolution", registry);

   protected final NativeMatrix previousSolution;

   public final NativeQPInputTypeA qpInputTypeA = new NativeQPInputTypeA(0);
   public final NativeQPInputTypeC qpInputTypeC = new NativeQPInputTypeC(0);

   protected final NativeMatrix solverOutput;

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveMPCVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfMPCIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfMPCEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfMPCInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfMPCConstraints", registry);

   private final YoDouble comCoefficientRegularization = new YoDouble("comCoefficientRegularization", registry);
   private final YoDouble rhoCoefficientRegularization = new YoDouble("rhoCoefficientRegularization", registry);

   private final YoDouble comRateCoefficientRegularization = new YoDouble("comRateCoefficientRegularization", registry);
   private final YoDouble rhoRateCoefficientRegularization = new YoDouble("rhoRateCoefficientRegularization", registry);

   private int problemSize;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   private final LinearMPCIndexHandler indexHandler;
   private final MPCQPInputCalculator inputCalculator;

   protected final double dt;
   protected final double dt2;

   private final RowMajorNativeMatrixGrower nativeMatrixGrower = new RowMajorNativeMatrixGrower();

   public LinearMPCQPSolver(LinearMPCIndexHandler indexHandler, double dt, double gravityZ, YoRegistry parentRegistry)
   {
      this(indexHandler,
           dt,
           gravityZ,
           new BlockInverseCalculator(indexHandler,
                                      indexHandler::getComCoefficientStartIndex,
                                      i -> indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment),
           parentRegistry);
   }

   public LinearMPCQPSolver(LinearMPCIndexHandler indexHandler, double dt, double gravityZ, InverseMatrixCalculator<NativeMatrix> inverseMatrixCalculator, YoRegistry parentRegistry)
   {
      this.indexHandler = indexHandler;
      this.dt = dt;
      dt2 = dt * dt;

      rhoCoefficientRegularization.set(1e-5);
      comCoefficientRegularization.set(1e-5);

      rhoRateCoefficientRegularization.set(1e-10);
      comRateCoefficientRegularization.set(1e-10);

      qpSolver = new MPCQPSolver();
      qpSolver.setConvergenceThreshold(5e-6);
      qpSolver.setConvergenceThresholdForLagrangeMultipliers(1e-4);
      if (inverseMatrixCalculator != null)
         qpSolver.setInverseHessianCalculator(inverseMatrixCalculator);
      qpSolver.setResetActiveSetOnSizeChange(false);

      inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      int problemSize = 3 * (2 * 4 * LinearMPCIndexHandler.coefficientsPerRho + LinearMPCIndexHandler.comCoefficientsPerSegment);

      previousSolution = new NativeMatrix(0, 0);

      solverOutput = new NativeMatrix(problemSize, 1);

      parentRegistry.addChild(registry);
   }

   public void setComCoefficientRegularizationWeight(double weight)
   {
      this.comCoefficientRegularization.set(weight);
   }

   public void setRhoCoefficientRegularizationWeight(double weight)
   {
      this.rhoCoefficientRegularization.set(weight);
   }

   public void setComRateCoefficientRegularizationWeight(double weight)
   {
      this.comRateCoefficientRegularization.set(weight);
   }

   public void setRhoRateCoefficientRegularizationWeight(double weight)
   {
      this.rhoRateCoefficientRegularization.set(weight);
   }

   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void notifyResetActiveSet()
   {
      this.resetActiveSet = true;
   }

   public void setPreviousSolution(DMatrixRMaj previousSolution)
   {
      this.previousSolution.set(previousSolution);
      addRateRegularization.set(true);
   }

   private boolean pollResetActiveSet()
   {
      boolean ret = resetActiveSet;
      resetActiveSet = false;
      return ret;
   }

   public void initialize()
   {
      problemSize = indexHandler.getTotalProblemSize();

      qpInputTypeA.setNumberOfVariables(problemSize);
      qpInputTypeC.setNumberOfVariables(problemSize);

      solverOutput.reshape(problemSize, 1);

      resetRateRegularization();

      qpSolver.initialize(problemSize);
   }

   public void resetRateRegularization()
   {
      addRateRegularization.set(false);
   }

   private void addCoefficientRegularization()
   {
      addValueRegularization();

      if (addRateRegularization.getBooleanValue())
         addRateRegularization();
   }

   public void addValueRegularization()
   {
      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int start = indexHandler.getComCoefficientStartIndex(segmentId);
         qpSolver.addRegularization(start, LinearMPCIndexHandler.comCoefficientsPerSegment, comCoefficientRegularization.getValue());
         start = indexHandler.getRhoCoefficientStartIndex(segmentId);
         qpSolver.addRegularization(start, indexHandler.getRhoCoefficientsInSegment(segmentId), rhoCoefficientRegularization.getValue());
      }
   }

   public void addRateRegularization()
   {
      double comCoefficientFactor = comRateCoefficientRegularization.getDoubleValue() / dt2;
      double rhoCoefficientFactor = rhoRateCoefficientRegularization.getDoubleValue() / dt2;

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int start = indexHandler.getComCoefficientStartIndex(segmentId);
         qpSolver.addRateRegularization(start, LinearMPCIndexHandler.comCoefficientsPerSegment, comCoefficientFactor, previousSolution);
         start += LinearMPCIndexHandler.comCoefficientsPerSegment;
         qpSolver.addRateRegularization(start, indexHandler.getRhoCoefficientsInSegment(segmentId), rhoCoefficientFactor, previousSolution);
      }
   }

   public void submitMPCCommandList(MPCCommandList commandList)
   {
      for (int i = 0; i < commandList.getNumberOfCommands(); i++)
      {
         MPCCommand<?> command = commandList.getCommand(i);
         submitMPCCommand(command);
      }
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
      switch (command.getCommandType())
      {
         case VALUE:
            submitMPCValueObjective((MPCValueCommand) command);
            break;
         case CONTINUITY:
            submitContinuityObjective((MPCContinuityCommand) command);
            break;
         case LIST:
            submitMPCCommandList((MPCCommandList) command);
            break;
         case RHO_VALUE:
            submitRhoValueCommand((RhoObjectiveCommand) command);
            break;
         case VRP_TRACKING:
            submitVRPTrackingCommand((VRPTrackingCommand) command);
            break;
         case RHO_BOUND:
            submitRhoBoundCommand((RhoBoundCommand) command);
            break;
         case NORMAL_FORCE_BOUND:
            submitNormalForceBoundCommand((NormalForceBoundCommand) command);
            break;
         case FORCE_VALUE:
            submitForceValueCommand((ForceObjectiveCommand) command);
            break;
         case FORCE_TRACKING:
            submitForceTrackingCommand((ForceTrackingCommand) command);
            break;
         case FORCE_RATE_TRACKING:
            submitForceRateTrackingCommand((ForceRateTrackingCommand) command);
            break;
         case RHO_TRACKING:
            submitRhoTrackingCommand((RhoTrackingCommand) command);
            break;
         case RHO_RATE_TRACKING:
            submitRhoRateTrackingCommand((RhoRateTrackingCommand) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled.");
      }
   }

   public void submitRhoValueCommand(RhoObjectiveCommand command)
   {
      int offset = inputCalculator.calculateCompactRhoValueCommand(qpInputTypeA, command);
      if (offset != -1)
         addInput(qpInputTypeA, offset);
   }

   public void submitMPCValueObjective(MPCValueCommand command)
   {
      int offset = inputCalculator.calculateCompactValueObjective(qpInputTypeA, command);
      if (offset != -1)
         addInput(qpInputTypeA, offset);
   }

   public void submitContinuityObjective(MPCContinuityCommand command)
   {
      int offset = inputCalculator.calculateContinuityObjective(qpInputTypeA, command);
      if (offset != -1)
         addInput(qpInputTypeA, offset);
   }

   public void submitVRPTrackingCommand(VRPTrackingCommand command)
   {
      int offset = inputCalculator.calculateCompactVRPTrackingObjective(qpInputTypeC, command);
      if (offset != -1)
         addInput(qpInputTypeC, offset);
   }

   public void submitRhoBoundCommand(RhoBoundCommand command)
   {
      int offset = inputCalculator.calculateRhoBoundCommandCompact(qpInputTypeA, command);
      if (offset != -1)
         addInput(qpInputTypeA, offset, command.getSlackVariableWeight());
   }

   public void submitNormalForceBoundCommand(NormalForceBoundCommand command)
   {
      int offset = inputCalculator.calculateNormalForceBoundCommandCompact(qpInputTypeA, command);
      if (offset != -1)
         addInput(qpInputTypeA, offset);
   }

   public void submitForceValueCommand(ForceObjectiveCommand command)
   {
      boolean success = inputCalculator.calculateForceMinimizationObjective(qpInputTypeC, command);
      if (success)
         addInput(qpInputTypeC);
   }

   public void submitForceTrackingCommand(ForceTrackingCommand command)
   {
      int offset = inputCalculator.calculateForceTrackingObjective(qpInputTypeC, command);
      if (offset != -1)
         addInput(qpInputTypeC);
   }

   public void submitForceRateTrackingCommand(ForceRateTrackingCommand command)
   {
      int offset = inputCalculator.calculateForceRateTrackingObjective(qpInputTypeC, command);
      if (offset != -1)
         addInput(qpInputTypeC);
   }

   public void submitRhoTrackingCommand(RhoTrackingCommand command)
   {
      int offset = inputCalculator.calculateRhoTrackingObjective(qpInputTypeC, command);
      if (offset != -1)
         addInput(qpInputTypeC, offset);
   }

   public void submitRhoRateTrackingCommand(RhoRateTrackingCommand command)
   {
      int offset = inputCalculator.calculateRhoRateTrackingObjective(qpInputTypeC, command);
      if (offset != -1)
         addInput(qpInputTypeC, offset);
   }

   public void addInput(NativeQPInputTypeA input)
   {
      addInput(input, 0);
   }

   public void addInput(NativeQPInputTypeA input, int offset)
   {
      addInput(input, offset, Double.NaN);
   }

   public void addInput(NativeQPInputTypeA input, int offset, double slackVariableWeight)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               qpSolver.addObjective(input.taskJacobian, input.taskObjective, input.getWeightScalar(), offset);
            else
               qpSolver.addObjective(input.taskJacobian, input.taskObjective, input.getTaskWeightMatrix(), offset);
            break;
         case EQUALITY:
            qpSolver.addEqualityConstraint(input.taskJacobian, input.taskObjective, problemSize, offset);
            break;
         case LEQ_INEQUALITY:
            qpSolver.addMotionLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, slackVariableWeight, problemSize, offset);
            break;
         case GEQ_INEQUALITY:
            qpSolver.addMotionGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, slackVariableWeight, problemSize, offset);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addInput(NativeQPInputTypeC input)
   {
      addInput(input, 0);
   }

   public void addInput(NativeQPInputTypeC input, int offset)
   {
      if (!input.useWeightScalar())
         throw new IllegalArgumentException("Not yet implemented.");

      qpSolver.addDirectObjective(input.directCostHessian, input.directCostGradient, input.getWeightScalar(), offset);
   }

   public boolean solve()
   {
      addCoefficientRegularization();

      numberOfEqualityConstraints.set(qpSolver.getNumberOfEqualityConstraints());//solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(qpSolver.getNumberOfInequalityConstraints());
      numberOfConstraints.set(numberOfEqualityConstraints.getIntegerValue() + numberOfInequalityConstraints.getIntegerValue());

      qpSolverTimer.startMeasurement();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      numberOfActiveVariables.set(problemSize);
      
      numberOfIterations.set(qpSolver.solve(solverOutput));

      qpSolverTimer.stopMeasurement();

      if (solverOutput.containsNaN())
      {
         addRateRegularization.set(false);
         numberOfIterations.set(-1);
         foundSolution.set(false);
         return false;
      }

      foundSolution.set(true);

      addRateRegularization.set(true);

      if (debug)
      {
         NativeMatrix constructedB = new NativeMatrix(qpSolver.linearEqualityConstraintsAMatrix.getNumRows(), 1);
         constructedB.mult(qpSolver.linearEqualityConstraintsAMatrix, solverOutput);
         for (int i = 0; i < constructedB.getNumRows(); i++)
         {
            if (!MathTools.epsilonEquals(constructedB.get(i, 0), qpSolver.linearEqualityConstraintsBVector.get(i, 0), 1e-3))
               LogTools.info("The equality constraints weren't satisfied.");
         }
      }

      return true;
   }

   public NativeMatrix getSolution()
   {
      return solverOutput;
   }

   public void setActiveInequalityIndices(TIntList activeInequalityIndices)
   {
      qpSolver.setActiveInequalityIndices(activeInequalityIndices);
   }

   public TIntList getActiveInequalityIndices()
   {
      return qpSolver.getActiveInequalityIndices();
   }
}
