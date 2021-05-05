package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.TIntList;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.BlockInverseCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.LinearMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.NativeQPInputTypeC;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;
import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.lang.annotation.Native;

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
   private final SimpleEfficientActiveSetQPSolver qpSolver;

   private final YoBoolean addRateRegularization = new YoBoolean("AddRateRegularization", registry);
   private final YoBoolean foundSolution = new YoBoolean("foundSolution", registry);

   private final NativeMatrix solverInput_H_previous;
   private final NativeMatrix solverInput_f_previous;

   // These are all public to help with testing. DO NOT USE THEM IN A PUBLIC FASHION
   public final NativeMatrix solverInput_H;
   public final NativeMatrix solverInput_f;

   public final NativeMatrix solverInput_Aeq;
   public final NativeMatrix solverInput_beq;
   public final NativeMatrix solverInput_Ain;
   public final NativeMatrix solverInput_bin;
   public final NativeMatrix solverOutput_beq;
   public final NativeMatrix solverOutput_bin;

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

      qpSolver = new SimpleEfficientActiveSetQPSolver();
      qpSolver.setConvergenceThreshold(5e-6);
      qpSolver.setConvergenceThresholdForLagrangeMultipliers(1e-4);
      if (inverseMatrixCalculator != null)
         qpSolver.setInverseHessianCalculator(inverseMatrixCalculator);
      qpSolver.setResetActiveSetOnSizeChange(false);

      inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      int problemSize = 3 * (2 * 4 * LinearMPCIndexHandler.coefficientsPerRho + LinearMPCIndexHandler.comCoefficientsPerSegment);
      solverInput_H = new NativeMatrix(problemSize, problemSize);
      solverInput_f = new NativeMatrix(problemSize, 1);

      solverInput_H_previous = new NativeMatrix(problemSize, problemSize);
      solverInput_f_previous = new NativeMatrix(problemSize, 1);

      solverInput_Aeq = new NativeMatrix(0, problemSize);
      solverInput_beq = new NativeMatrix(0, 1);
      solverInput_Ain = new NativeMatrix(0, problemSize);
      solverInput_bin = new NativeMatrix(0, 1);
      solverOutput_bin = new NativeMatrix(0, 1);
      solverOutput_beq = new NativeMatrix(0, 1);

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

         solverInput_H.reshape(problemSize, problemSize);
         solverInput_f.reshape(problemSize, 1);

         solverInput_H_previous.reshape(problemSize, problemSize);
         solverInput_f_previous.reshape(problemSize, 1);

         solverOutput.reshape(problemSize, 1);

         resetRateRegularization();

      solverInput_Aeq.zero();
      solverInput_beq.zero();

      solverInput_Ain.zero();
      solverInput_bin.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);

      solverInput_H.zero();
      solverInput_f.zero();
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
         for (int i = 0; i < LinearMPCIndexHandler.comCoefficientsPerSegment; i++)
            solverInput_H.add(start + i, start + i, comCoefficientRegularization.getDoubleValue());

         start = indexHandler.getRhoCoefficientStartIndex(segmentId);
         for (int i = 0; i < indexHandler.getRhoCoefficientsInSegment(segmentId); i++)
            solverInput_H.add(start + i, start + i, rhoCoefficientRegularization.getDoubleValue());
      }
   }

   public void addRateRegularization()
   {
      double comCoefficientFactor = comRateCoefficientRegularization.getDoubleValue() / dt2;
      double rhoCoefficientFactor = rhoRateCoefficientRegularization.getDoubleValue() / dt2;

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int start = indexHandler.getComCoefficientStartIndex(segmentId);
         for (int i = 0; i < LinearMPCIndexHandler.comCoefficientsPerSegment; i++)
         {
            double previousValue = previousSolution.get(start + i, 0);
            if (Double.isNaN(previousValue))
               continue;
            solverInput_H.add(start + i, start + i, comCoefficientFactor);
            solverInput_f.add(start + i, 0, -previousValue * comCoefficientFactor);
         }

         start += LinearMPCIndexHandler.comCoefficientsPerSegment;
         for (int i = 0; i < indexHandler.getRhoCoefficientsInSegment(segmentId); i++)
         {
            double previousValue = previousSolution.get(start + i, 0);
            if (Double.isNaN(previousValue))
               continue;
            solverInput_H.add(start + i, start + i, rhoCoefficientFactor);
            solverInput_f.add(start + i, 0, -previousValue * rhoCoefficientFactor);
         }
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
         addInput(qpInputTypeA, offset);
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

   public void addInput(NativeQPInputTypeA input)
   {
      addInput(input, 0);
   }

   public void addInput(NativeQPInputTypeA input, int offset)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               addObjective(input.taskJacobian, input.taskObjective, input.getWeightScalar(), offset);
            else
               addObjective(input.taskJacobian, input.taskObjective, input.getTaskWeightMatrix(), offset);
            break;
         case EQUALITY:
            addEqualityConstraint(input.taskJacobian, input.taskObjective, offset);
            break;
         case LEQ_INEQUALITY:
            addMotionLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, offset);
            break;
         case GEQ_INEQUALITY:
            addMotionGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective, offset);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, 0);
   }

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, int offset)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, taskJacobian.getNumCols(), offset, solverInput_H, solverInput_f);
   }

   public static void addObjective(NativeMatrix taskJacobian,
                                   NativeMatrix taskObjective,
                                   double taskWeight,
                                   int problemSize,
                                   int offset,
                                   NativeMatrix solverInput_H,
                                   NativeMatrix solverInput_f)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      // TODO figure out an efficient inner product in eigen
      solverInput_H.multAddBlockTransA(taskWeight, taskJacobian, taskJacobian, offset, offset);
      if (debug && solverInput_H.containsNaN())
         throw new RuntimeException("error");

      // Compute: f += - J^T W Objective
      solverInput_f.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, offset, 0);
      if (debug && solverInput_f.containsNaN())
         throw new RuntimeException("error");
   }

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, int offset)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, taskJacobian.getNumCols(), offset, solverInput_H, solverInput_f);
   }

   private final NativeMatrix tempJtW = new NativeMatrix(0, 0);

   private void addObjective(NativeMatrix taskJacobian,
                             NativeMatrix taskObjective,
                             NativeMatrix taskWeight,
                             int problemSize,
                             int offset,
                             NativeMatrix solverInput_H,
                             NativeMatrix solverInput_f)
   {
      int taskSize = taskJacobian.getNumRows();
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      tempJtW.reshape(variables, taskSize);

      // J^T W
      tempJtW.multTransA(taskJacobian, taskWeight);

      // Compute: H += J^T W J
      solverInput_H.multAddBlock(tempJtW, taskJacobian, offset, offset);
      if (debug && solverInput_H.containsNaN())
         throw new RuntimeException("error");

      // Compute: f += - J^T W Objective
      solverInput_f.multAddBlock(-1.0, tempJtW, taskObjective, offset, 0);
      if (debug && solverInput_f.containsNaN())
         throw new RuntimeException("error");
   }

   public void addEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addEqualityConstraint(taskJacobian, taskObjective, 0);
   }

   public void addEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int taskColOffset)
   {
      addEqualityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, taskColOffset, solverInput_Aeq, solverInput_beq);
   }

   public static void addEqualityConstraint(NativeMatrix taskJacobian,
                                            NativeMatrix taskObjective,
                                            int problemSize,
                                            NativeMatrix solverInput_Aeq,
                                            NativeMatrix solverInput_beq)
   {
      addEqualityConstraint(taskJacobian, taskObjective, problemSize, problemSize, 0, solverInput_Aeq, solverInput_beq);
   }

   public static void addEqualityConstraint(NativeMatrix taskJacobian,
                                            NativeMatrix taskObjective,
                                            int problemSize,
                                            int totalProblemSize,
                                            int colOffset,
                                            NativeMatrix solverInput_Aeq,
                                            NativeMatrix solverInput_beq)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }

      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (variables + colOffset > totalProblemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.conservativeReshapeRows(previousSize + taskSize);
      solverInput_beq.conservativeReshapeRows(previousSize + taskSize);

      solverInput_Aeq.insert(taskJacobian, previousSize, colOffset);
      solverInput_beq.insert(taskObjective, previousSize, 0);

      if (debug && solverInput_Aeq.containsNaN())
         throw new RuntimeException("error");
      if (debug && solverInput_beq.containsNaN())
         throw new RuntimeException("error");
   }



   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addMotionLesserOrEqualInequalityConstraint(taskJacobian, taskObjective, 0);
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int colOffset)
   {
      addMotionLesserOrEqualInequalityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, colOffset, solverInput_Ain, solverInput_bin);
   }

   public static void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                                 NativeMatrix taskObjective,
                                                                 int problemSize,
                                                                 int totalProblemSize,
                                                                 int colOffset,
                                                                 NativeMatrix solverInput_Ain,
                                                                 NativeMatrix solverInput_bin)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, 1.0, problemSize, totalProblemSize, colOffset, solverInput_Ain, solverInput_bin);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addMotionGreaterOrEqualInequalityConstraint(taskJacobian, taskObjective, 0);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int colOffset)
   {
      addMotionGreaterOrEqualInequalityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, colOffset, solverInput_Ain, solverInput_bin);
   }

   public static void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                                  NativeMatrix taskObjective,
                                                                  int problemSize,
                                                                  int totalProblemSize,
                                                                  int colOffset,
                                                                  NativeMatrix solverInput_Ain,
                                                                  NativeMatrix solverInput_bin)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, -1.0, problemSize, totalProblemSize, colOffset, solverInput_Ain, solverInput_bin);
   }

   private static void addInequalityConstraintInternal(NativeMatrix taskJacobian,
                                                       NativeMatrix taskObjective,
                                                       double sign,
                                                       int problemSize,
                                                       int totalProblemSize,
                                                       int colOffset,
                                                       NativeMatrix solverInput_Ain,
                                                       NativeMatrix solverInput_bin)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      if (variables > totalProblemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_bin.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Ain.conservativeReshapeRows(previousSize + taskSize);
      solverInput_bin.conservativeReshapeRows(previousSize + taskSize);

      taskJacobian.scale(sign);
      solverInput_Ain.insert(taskJacobian, previousSize, colOffset);
      solverInput_bin.insert(taskObjective, previousSize, 0);
   }

   public void addInput(NativeQPInputTypeC input)
   {
      addInput(input, 0);
   }

   public void addInput(NativeQPInputTypeC input, int offset)
   {
      if (!input.useWeightScalar())
         throw new IllegalArgumentException("Not yet implemented.");

      int size = input.directCostHessian.getNumCols();
      solverInput_H.addBlock(input.directCostHessian, offset, offset, 0, 0, size, size, input.getWeightScalar());
      if (debug && solverInput_H.containsNaN())
         throw new RuntimeException("error");
      solverInput_f.addBlock(input.directCostGradient, offset, 0, 0, 0, size, 1, input.getWeightScalar());
      if (debug && solverInput_f.containsNaN())
         throw new RuntimeException("Error");
   }

   public boolean solve()
   {
      addCoefficientRegularization();

      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      numberOfActiveVariables.set(problemSize);

      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

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

      solverInput_H_previous.set(solverInput_H);
      solverInput_f_previous.set(solverInput_f);

      solverOutput_beq.reshape(numberOfEqualityConstraints.getIntegerValue(), 1);
      solverOutput_bin.reshape(numberOfInequalityConstraints.getIntegerValue(), 1);

      solverOutput_bin.mult(solverInput_Ain, solverOutput);
      solverOutput_beq.mult(solverInput_Aeq, solverOutput);

      return true;
   }

   public DMatrix getSolution()
   {
      return solverOutput;
   }

   public void setActiveInequalityIndices(TIntList activeInequalityIndices)
   {
      qpSolver.setActiveInequalityIndices(activeInequalityIndices);
   }

   public void setActiveLowerBoundIndices(TIntList activeLowerBoundIndices)
   {
      qpSolver.setActiveLowerBoundIndices(activeLowerBoundIndices);
   }

   public void setActiveUpperBoundIndices(TIntList activeUpperBoundIndices)
   {
      qpSolver.setActiveUpperBoundIndices(activeUpperBoundIndices);
   }

   public TIntList getActiveInequalityIndices()
   {
      return qpSolver.getActiveInequalityIndices();
   }

   public TIntList getActiveLowerBoundIndices()
   {
      return qpSolver.getActiveLowerBoundIndices();
   }

   public TIntList getActiveUpperBoundIndices()
   {
      return qpSolver.getActiveUpperBoundIndices();
   }
}
