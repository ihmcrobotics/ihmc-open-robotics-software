package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.convexOptimization.quadraticProgram.NativeActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * It is assumed that the cost function has the form
 *
 * <pre>
 * f(x) = 0.5 * x<sup>T</sup> H x + f<sup>T</sup>x
 * </pre>
 */
public class InverseDynamicsQPSolver
{
   private static final boolean SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final ExecutionTimer qpSolverTimer = new ExecutionTimer("qpSolverTimer", 0.5, registry);

   private final YoFrameVector3D wrenchEquilibriumForceError;
   private final YoFrameVector3D wrenchEquilibriumTorqueError;

   private final YoBoolean addRateRegularization = new YoBoolean("AddRateRegularization", registry);
   private final NativeActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final NativeQPVariableSubstitution accelerationVariablesSubstitution = new NativeQPVariableSubstitution();

   private final NativeMatrix nativeSolverInput_H;
   private final NativeMatrix nativeSolverInput_f;
   private final DMatrixRMaj solverInput_H;
   private final DMatrixRMaj solverInput_f;
   private final NativeMatrix finalSolverInput_H;
   private final NativeMatrix finalSolverInput_f;

   private final NativeMatrix solverInput_H_previous;
   private final NativeMatrix solverInput_f_previous;

   private final NativeMatrix nativeSolverInput_Aeq;
   private final NativeMatrix nativeSolverInput_beq;
   private final NativeMatrix nativeSolverInput_Ain;
   private final NativeMatrix nativeSolverInput_bin;
   private final NativeMatrix temp_A;
   private final NativeMatrix temp_b;

   private final DMatrixRMaj solverInput_Aeq;
   private final DMatrixRMaj solverInput_beq;
   private final DMatrixRMaj solverInput_Ain;
   private final DMatrixRMaj solverInput_bin;

   private final NativeMatrix finalSolverInput_Aeq;
   private final NativeMatrix finalSolverInput_beq;
   private final NativeMatrix finalSolverInput_Ain;
   private final NativeMatrix finalSolverInput_bin;

   private final NativeMatrix solverInput_lb;
   private final NativeMatrix solverInput_ub;

   private final NativeMatrix solverInput_lb_previous;
   private final NativeMatrix solverInput_ub_previous;

   private final NativeMatrix solverInput_activeIndices;

   private final NativeMatrix solverOutput;
   private final NativeMatrix solverOutput_jointAccelerations;
   private final NativeMatrix solverOutput_rhos;

   private final NativeMatrix tempObjective = new NativeMatrix(0, 0);
   private final NativeMatrix tempWeight = new NativeMatrix(0, 0);

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble jointAccelerationRegularization = new YoDouble("jointAccelerationRegularization", registry);
   private final YoDouble jointJerkRegularization = new YoDouble("jointJerkRegularization", registry);
   private final YoDouble jointTorqueWeight = new YoDouble("jointTorqueWeight", registry);
   private final NativeMatrix regularizationMatrix;

   private final DMatrixRMaj tempJtW;
   private final NativeMatrix nativeTempJtW = new NativeMatrix(0, 0);

   private final int numberOfDoFs;
   private final int rhoSize;
   private final int problemSize;
   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   private final double dt;

   public InverseDynamicsQPSolver(NativeActiveSetQPSolverWithInactiveVariablesInterface qpSolver,
                                  int numberOfDoFs,
                                  int rhoSize,
                                  boolean hasFloatingBase,
                                  double dt,
                                  YoRegistry parentRegistry)
   {
      this.qpSolver = qpSolver;
      this.numberOfDoFs = numberOfDoFs;
      this.rhoSize = rhoSize;
      this.hasFloatingBase = hasFloatingBase;
      this.problemSize = numberOfDoFs + rhoSize;
      this.dt = dt;

      addRateRegularization.set(false);

      nativeSolverInput_H = new NativeMatrix(problemSize, problemSize);
      nativeSolverInput_f = new NativeMatrix(problemSize, 1);
      finalSolverInput_H = qpSolver.getCostHessianUnsafe();
      finalSolverInput_f = qpSolver.getCostGradientUnsafe();
      solverInput_H = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f = new DMatrixRMaj(problemSize, 1);

      solverInput_H_previous = new NativeMatrix(problemSize, problemSize);
      solverInput_f_previous = new NativeMatrix(problemSize, 1);

      nativeSolverInput_Aeq = new NativeMatrix(problemSize, problemSize);
      nativeSolverInput_beq = new NativeMatrix(problemSize, 1);
      nativeSolverInput_Ain = new NativeMatrix(problemSize, problemSize);
      nativeSolverInput_bin = new NativeMatrix(problemSize, 1);
      finalSolverInput_Ain = qpSolver.getAinUnsafe();
      finalSolverInput_bin = qpSolver.getBinUnsafe();
      finalSolverInput_Aeq = qpSolver.getAeqUnsafe();
      finalSolverInput_beq = qpSolver.getBeqUnsafe();
      temp_A = new NativeMatrix(problemSize, problemSize);
      temp_b = new NativeMatrix(problemSize, 1);
      solverInput_Aeq = new DMatrixRMaj(0, problemSize);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, problemSize);
      solverInput_bin = new DMatrixRMaj(0, 1);

      solverInput_lb = qpSolver.getLowerBoundsUnsafe();
      solverInput_ub = qpSolver.getUpperBoundsUnsafe();
      solverInput_lb.reshape(problemSize, 1);
      solverInput_ub.reshape(problemSize, 1);

      solverInput_lb_previous = new NativeMatrix(problemSize, 1);
      solverInput_ub_previous = new NativeMatrix(problemSize, 1);

      solverInput_lb.fill(Double.NEGATIVE_INFINITY);
      solverInput_ub.fill(Double.POSITIVE_INFINITY);

      solverInput_activeIndices = new NativeMatrix(problemSize, 1);
      solverInput_activeIndices.fill(1.0);

      solverOutput = new NativeMatrix(problemSize, 1);
      solverOutput_jointAccelerations = new NativeMatrix(numberOfDoFs, 1);
      solverOutput_rhos = new NativeMatrix(rhoSize, 1);

      tempJtW = new DMatrixRMaj(problemSize, problemSize);

      jointAccelerationRegularization.set(0.005);
      jointJerkRegularization.set(0.1);
      jointTorqueWeight.set(0.001);
      regularizationMatrix = new NativeMatrix(problemSize, problemSize);

//      for (int i = 0; i < numberOfDoFs; i++)
//         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());
      double defaultRhoRegularization = 0.00001;
//      for (int i = numberOfDoFs; i < problemSize; i++)
//         regularizationMatrix.set(i, i, defaultRhoRegularization);
      regularizationMatrix.zero();
      regularizationMatrix.addDiagonal(0, 0, numberOfDoFs, jointAccelerationRegularization.getDoubleValue());
      regularizationMatrix.addDiagonal(numberOfDoFs, numberOfDoFs, problemSize - numberOfDoFs, defaultRhoRegularization);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         wrenchEquilibriumForceError = new YoFrameVector3D("wrenchEquilibriumForceError", null, registry);
         wrenchEquilibriumTorqueError = new YoFrameVector3D("wrenchEquilibriumTorqueError", null, registry);
      }
      else
      {
         wrenchEquilibriumForceError = null;
         wrenchEquilibriumTorqueError = null;
      }

      accelerationVariablesSubstitution.setIgnoreBias(true);

      parentRegistry.addChild(registry);
   }

   public void setAccelerationRegularizationWeight(double weight)
   {
      jointAccelerationRegularization.set(weight);
   }

   public void setJerkRegularizationWeight(double weight)
   {
      jointJerkRegularization.set(weight);
   }

   public void setJointTorqueWeight(double weight)
   {
      jointTorqueWeight.set(weight);
   }

   public void setRhoRegularizationWeight(DMatrixRMaj weight)
   {
//      CommonOps_DDRM.insert(weight, regularizationMatrix, numberOfDoFs, numberOfDoFs);

      regularizationMatrix.insert(weight, numberOfDoFs, numberOfDoFs);
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

   private boolean pollResetActiveSet()
   {
      boolean ret = resetActiveSet;
      resetActiveSet = false;
      return ret;
   }

   public void reset()
   {
      regularizationMatrix.fillDiagonal(0, 0, numberOfDoFs, jointAccelerationRegularization.getDoubleValue());

      accelerationVariablesSubstitution.reset();

      solverInput_H.zero();
      solverInput_f.zero();
      nativeSolverInput_H.zero();
      nativeSolverInput_f.zero();

      nativeSolverInput_Aeq.reshape(0, problemSize);
      nativeSolverInput_beq.reshape(0, 1);
      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      nativeSolverInput_Ain.reshape(0, problemSize);
      nativeSolverInput_bin.reshape(0, 1);
      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);

      solverInput_activeIndices.fill(1.0);
   }

   private void addRegularization()
   {
      nativeSolverInput_H.addEquals(regularizationMatrix);

      if (addRateRegularization.getBooleanValue())
      {
         addJointJerkRegularization();
      }
   }

   public void resetRateRegularization()
   {
      addRateRegularization.set(false);
   }

   private void addJointJerkRegularization()
   {
      double factor = dt * dt / jointJerkRegularization.getDoubleValue();
      nativeSolverInput_H.addDiagonal(0, 0, numberOfDoFs, 1.0 / factor);
      nativeSolverInput_f.addBlock(solverOutput_jointAccelerations, 0, 0, 0, 0, numberOfDoFs, 1, -1.0 / factor);
   }

   public void addMotionInput(NativeQPInputTypeA input)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
               addMotionTask(input.taskJacobian, input.taskObjective, input.getWeightScalar());
            else
               addMotionTask(input.taskJacobian, input.taskObjective, input.taskWeightMatrix);
            break;
         case EQUALITY:
            addMotionEqualityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case LEQ_INEQUALITY:
            addMotionLesserOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         case GEQ_INEQUALITY:
            addMotionGreaterOrEqualInequalityConstraint(input.taskJacobian, input.taskObjective);
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }


   public void addMotionInput(QPInputTypeB input)
   {
      if (input.useWeightScalar())
         addMotionTask(input.taskJacobian, input.taskConvectiveTerm, input.getWeightScalar(), input.directCostHessian, input.directCostGradient);
      else
         addMotionTask(input.taskJacobian, input.taskConvectiveTerm, input.taskWeightMatrix, input.directCostHessian, input.directCostGradient);
   }

   public void addRhoInput(NativeQPInputTypeA input)
   {
      switch (input.getConstraintType())
      {
         case OBJECTIVE:
            if (input.useWeightScalar())
            {
               addRhoTask(input.getTaskJacobian(), input.getTaskObjective(), input.getWeightScalar());
            }
            else
            {
               addRhoTask(input.getTaskJacobian(), input.getTaskObjective(), input.getTaskWeightMatrix());
            }
            break;
         case EQUALITY:
            addRhoEqualityConstraint(input.getTaskJacobian(), input.getTaskObjective());
            break;
         case LEQ_INEQUALITY:
            addRhoLesserOrEqualInequalityConstraint(input.getTaskJacobian(), input.getTaskObjective());
            break;
         case GEQ_INEQUALITY:
            addRhoGreaterOrEqualInequalityConstraint(input.getTaskJacobian(), input.getTaskObjective());
            break;
         default:
            throw new RuntimeException("Unexpected constraint type: " + input.getConstraintType());
      }
   }

   public void addMotionTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, 0);
   }

   public void addRhoTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight)
   {
      if (taskJacobian.getNumCols() != rhoSize)
      {
         throw new RuntimeException("Rho task needs to have size macthing the number of rhos of the robot.");
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, numberOfDoFs);
   }

   /**
    * Sets up a motion objective for the joint accelerations (qddot).
    * <p>
    * min (J qddot - b)^T * W * (J qddot - b)
    * </p>
    *
    * @param taskJacobian jacobian to map qddot to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be
    *       diagonal.
    */
   public void addMotionTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, 0);
   }

   public void addMotionTask(DMatrixRMaj taskJacobian,
                             DMatrixRMaj taskConvectiveTerm,
                             double taskWeight,
                             DMatrixRMaj directCostHessian,
                             DMatrixRMaj directCostGradient)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addTaskInternal(taskJacobian, taskConvectiveTerm, taskWeight,directCostHessian, directCostGradient, 0);
   }

   public void addMotionTask(DMatrixRMaj taskJacobian,
                             DMatrixRMaj taskConvectiveTerm,
                             DMatrixRMaj taskWeight,
                             DMatrixRMaj directCostHessian,
                             DMatrixRMaj directCostGradient)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addTaskInternal(taskJacobian, taskConvectiveTerm, taskWeight, directCostHessian, directCostGradient, 0);
   }

   /**
    * Sets up a motion objective for the generalized contact forces (rhos).
    * <p>
    * min (J rho - b)^T * W * (J rho - b)
    * </p>
    *
    * @param taskJacobian jacobian to map rho to the objective space. J in the above equation.
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be
    *       diagonal.
    */
   public void addRhoTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight)
   {
      if (taskJacobian.getNumCols() != rhoSize)
      {
         throw new RuntimeException("Rho task needs to have size macthing the number of rhos of the robot.");
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, numberOfDoFs);
   }

   /**
    * Sets up a motion objective for the generalized contact forces (rhos).
    * <p>
    * min (rho - b)^T * W * (rho - b)
    * </p>
    *
    * @param taskObjective matrix of the desired objective for the rho task. b in the above equation.
    * @param taskWeight weight for the desired objective. W in the above equation. Assumed to be
    *       diagonal.
    */
   public void addRhoTask(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      tempObjective.set(taskObjective);
      tempWeight.set(taskWeight);
      addTaskInternal(tempObjective, tempWeight, numberOfDoFs, rhoSize);
   }

   public void addTaskInternal(NativeMatrix taskObjective, NativeMatrix taskWeight, int offset, int variables)
   {
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      nativeSolverInput_H.addBlock(taskWeight, offset, offset, 0, 0, variables, variables);
      nativeSolverInput_f.multAddBlock(-1.0, taskWeight, taskObjective, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // J^T W
      nativeTempJtW.multTransA(taskJacobian, taskWeight);

      // Compute: H += J^T W J
      nativeSolverInput_H.multAddBlock(nativeTempJtW, taskJacobian, offset, offset);

      // Compute: f += - J^T W Objective
      nativeSolverInput_f.multAddBlock(-1.0, nativeTempJtW, taskObjective, offset, 0);
   }

   private void addTaskInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      nativeSolverInput_H.multAddBlockTransA(taskWeight, taskJacobian, taskJacobian, offset, offset);

      // Compute: f += - J^T W Objective
      nativeSolverInput_f.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, offset, 0);
   }

   private void addTaskInternal(DMatrixRMaj taskJacobian,
                                DMatrixRMaj taskConvectiveTerm,
                                DMatrixRMaj taskWeight,
                                DMatrixRMaj directCostHessian,
                                DMatrixRMaj directCostGradient,
                                int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      tempJtW.reshape(variables, taskSize);

      // J^T Q
      CommonOps_DDRM.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: f += J^T Q g
      MatrixTools.multAddBlock(tempJtW, directCostGradient, solverInput_f, offset, 0);

      // J^T (Q + H)
      CommonOps_DDRM.multAddTransA(taskJacobian, directCostHessian, tempJtW);

      // Compute: H += J^T (H + Q) J
      MatrixTools.multAddBlock(tempJtW, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += J^T (Q + H) b
      MatrixTools.multAddBlock(tempJtW, taskConvectiveTerm, solverInput_f, offset, 0);
   }

   private void addTaskInternal(DMatrixRMaj taskJacobian,
                                DMatrixRMaj taskConvectiveTerm,
                                double taskWeight,
                                DMatrixRMaj directCostHessian,
                                DMatrixRMaj directCostGradient,
                                int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: f += J^T W g
      MatrixTools.multAddBlock(taskWeight, taskJacobian, directCostGradient, solverInput_f, offset, 0);

      // J^T (Q + H)
      CommonOps_DDRM.multTransA(taskWeight, taskJacobian, directCostHessian, tempJtW);

      // Compute: H += J^T (H + Q) J
      MatrixTools.multAddBlock(tempJtW, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += J^T (Q + H) b
      MatrixTools.multAddBlock(tempJtW, taskConvectiveTerm, solverInput_f, offset, 0);
   }

   public void addMotionEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addEqualityConstraintInternal(taskJacobian, taskObjective, 0);
   }

   public void addMotionEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addEqualityConstraintInternal(taskJacobian, taskObjective, 0);
   }

   public void addRhoEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      if (taskJacobian.getNumCols() != rhoSize)
      {
         throw new RuntimeException("Rho task needs to have size macthing the number of rhos of the robot.");
      }
      addEqualityConstraintInternal(taskJacobian, taskObjective, numberOfDoFs);
   }

   public void addRhoEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      if (taskJacobian.getNumCols() != rhoSize)
      {
         throw new RuntimeException("Rho task needs to have size macthing the number of rhos of the robot.");
      }
      addEqualityConstraintInternal(taskJacobian, taskObjective, numberOfDoFs);
   }

   private void addEqualityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Aeq.reshape(previousSize + taskSize, problemSize, true);
      solverInput_beq.reshape(previousSize + taskSize, 1, true);

      CommonOps_DDRM.insert(taskJacobian, solverInput_Aeq, previousSize, offset);
      CommonOps_DDRM.insert(taskObjective, solverInput_beq, previousSize, 0);
   }

   private void addEqualityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = nativeSolverInput_beq.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      temp_A.set(nativeSolverInput_Aeq);
      temp_b.set(nativeSolverInput_beq);
      nativeSolverInput_Aeq.reshape(previousSize + taskSize, problemSize);
      nativeSolverInput_beq.reshape(previousSize + taskSize, 1);

      nativeSolverInput_Aeq.insert(temp_A, 0, 0);
      nativeSolverInput_Aeq.insert(taskJacobian, previousSize, offset);
      nativeSolverInput_Aeq.zeroBlock(previousSize, previousSize + taskSize, 0, offset);
      nativeSolverInput_beq.insert(temp_b, 0, 0);
      nativeSolverInput_beq.insert(taskObjective, previousSize, 0);
   }

   public void addMotionLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addRhoLesserOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addRhoInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   private void addRhoGreaterOrEqualInequalityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
   {
      addRhoInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addMotionInequalityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double sign)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, 0);
   }

   private void addRhoInequalityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double sign)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, numberOfDoFs);
   }

   private void addInequalityConstraintInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double sign, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = solverInput_bin.getNumRows();

      // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
      solverInput_Ain.reshape(previousSize + taskSize, problemSize, true);
      solverInput_bin.reshape(previousSize + taskSize, 1, true);

      MatrixTools.setMatrixBlock(solverInput_Ain, previousSize, offset, taskJacobian, 0, 0, taskSize, variables, sign);
      MatrixTools.setMatrixBlock(solverInput_bin, previousSize, 0, taskObjective, 0, 0, taskSize, 1, sign);
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addMotionInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addRhoLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addRhoInequalityConstraintInternal(taskJacobian, taskObjective, 1.0);
   }

   private void addRhoGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective)
   {
      addRhoInequalityConstraintInternal(taskJacobian, taskObjective, -1.0);
   }

   private void addMotionInequalityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double sign)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, 0);
   }

   private void addRhoInequalityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double sign)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, numberOfDoFs);
   }

   private void addInequalityConstraintInternal(NativeMatrix taskJacobian, NativeMatrix taskObjective, double sign, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      int previousSize = nativeSolverInput_bin.getNumRows();

      temp_A.set(nativeSolverInput_Ain);
      temp_b.set(nativeSolverInput_bin);
      nativeSolverInput_Ain.reshape(previousSize + taskSize, problemSize);
      nativeSolverInput_bin.reshape(previousSize + taskSize, 1);

      nativeSolverInput_Ain.insertScaled(temp_A, 0, 0, sign);
      nativeSolverInput_Ain.insertScaled(taskJacobian, previousSize, offset, sign);
      nativeSolverInput_Ain.zeroBlock(previousSize, previousSize + taskSize, 0, offset);
      nativeSolverInput_bin.insertScaled(temp_b, 0, 0, sign);
      nativeSolverInput_bin.insertScaled(taskObjective, previousSize, 0, sign);
   }

   public void addTorqueMinimizationObjective(DMatrixRMaj torqueJacobian, DMatrixRMaj torqueObjective)
   {
      // Compute: H += J^T W J
      MatrixTools.multAddInner(jointTorqueWeight.getDoubleValue(), torqueJacobian, solverInput_H);

      // Compute: f += - J^T W Objective
      CommonOps_DDRM.multAddTransA(-jointTorqueWeight.getDoubleValue(), torqueJacobian, torqueObjective, solverInput_f);
   }

   public void addTorqueMinimizationObjective(DMatrixRMaj torqueQddotJacobian, DMatrixRMaj torqueRhoJacobian, DMatrixRMaj torqueObjective)
   {
      int taskSize = torqueObjective.getNumRows();

      tempJtW.reshape(taskSize, problemSize);
      CommonOps_DDRM.insert(torqueQddotJacobian, tempJtW, 0, 0);
      CommonOps_DDRM.insert(torqueRhoJacobian, tempJtW, 0, numberOfDoFs);

      addTorqueMinimizationObjective(tempJtW, torqueObjective);
   }

   private final NativeMatrix nativeAdditionalExternalWrench = new NativeMatrix(1, 1);
   private final NativeMatrix nativeGravityWrench = new NativeMatrix(1, 1);

   /**
    * Need to be called before {@link #solve()}. It sets up the constraint that ensures that the
    * solution is dynamically feasible:
    * <p>
    * <li>hDot = &sum;W<sub>ext</sub>
    * <li>A * qDDot + ADot * qDot = Q * &rho; + &sum;W<sub>user</sub> + W<sub>gravity</sub>
    * <li>-A * qDDot - ADot * qDot = - Q * &rho; - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li>-A * qDDot + Q * &rho; = ADot * qDot - &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * <li>[-A Q] * [qDDot<sup>T</sup> &rho;<sup>T</sup>]<sup>T</sup> = ADot * qDot -
    * &sum;W<sub>user</sub> - W<sub>gravity</sub>
    * </p>
    *
    * @param centroidalMomentumMatrix refers to A in the equation.
    * @param rhoJacobian refers to Q in the equation. Q&rho; represents external wrench to
    *       be optimized for.
    * @param convectiveTerm refers to ADot * qDot in the equation.
    * @param additionalExternalWrench refers to &sum;W<sub>user</sub> in the equation. These are
    *       constant wrenches usually used for compensating for the weight of
    *       an object that the robot is holding.
    * @param gravityWrench refers to W<sub>gravity</sub> in the equation. It the wrench
    *       induced by the weight of the robot.
    */
   public void setupWrenchesEquilibriumConstraint(DMatrixRMaj centroidalMomentumMatrix,
                                                  DMatrixRMaj rhoJacobian,
                                                  DMatrixRMaj convectiveTerm,
                                                  DMatrixRMaj additionalExternalWrench,
                                                  DMatrixRMaj gravityWrench)
   {
      if (!hasFloatingBase)
      {
         hasWrenchesEquilibriumConstraintBeenSetup = true;
         return;
      }

      tempWrenchConstraint_RHS.set(convectiveTerm);
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, additionalExternalWrench);
      CommonOps_DDRM.subtractEquals(tempWrenchConstraint_RHS, gravityWrench);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         tempWrenchConstraint_J.reshape(Wrench.SIZE, problemSize);
         MatrixTools.setMatrixBlock(tempWrenchConstraint_J, 0, 0, centroidalMomentumMatrix, 0, 0, Wrench.SIZE, numberOfDoFs, -1.0);
         CommonOps_DDRM.insert(rhoJacobian, tempWrenchConstraint_J, 0, numberOfDoFs);

         double weight = 150.0;
         MatrixTools.multAddInner(weight, tempWrenchConstraint_J, solverInput_H);
         CommonOps_DDRM.multAddTransA(-weight, tempWrenchConstraint_J, tempWrenchConstraint_RHS, solverInput_f);
      }
      else
      {
         int constraintSize = Wrench.SIZE;
         int previousSize = solverInput_beq.getNumRows();

         // Careful on that one, it works as long as matrices are row major and that the number of columns is not changed.
         solverInput_Aeq.reshape(previousSize + constraintSize, problemSize, true);
         solverInput_beq.reshape(previousSize + constraintSize, 1, true);

         MatrixTools.setMatrixBlock(solverInput_Aeq, previousSize, 0, centroidalMomentumMatrix, 0, 0, constraintSize, numberOfDoFs, -1.0);
         CommonOps_DDRM.insert(rhoJacobian, solverInput_Aeq, previousSize, numberOfDoFs);

         CommonOps_DDRM.insert(tempWrenchConstraint_RHS, solverInput_beq, previousSize, 0);
      }

      hasWrenchesEquilibriumConstraintBeenSetup = true;
   }

   public void addAccelerationSubstitution(NativeQPVariableSubstitution substitution)
   {
      this.accelerationVariablesSubstitution.concatenate(substitution);
   }

   private final DMatrixRMaj tempWrenchConstraint_J = new DMatrixRMaj(Wrench.SIZE, 200);
   private final DMatrixRMaj tempWrenchConstraint_LHS = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj tempWrenchConstraint_RHS = new DMatrixRMaj(Wrench.SIZE, 1);

   public boolean solve()
   {
      if (!hasWrenchesEquilibriumConstraintBeenSetup)
         throw new RuntimeException("The wrench equilibrium constraint has to be setup before calling solve().");

      addRegularization();

      numberOfEqualityConstraints.set(solverInput_Aeq.getNumRows());
      numberOfInequalityConstraints.set(solverInput_Ain.getNumRows());
      numberOfConstraints.set(solverInput_Aeq.getNumRows() + solverInput_Ain.getNumRows());

      qpSolverTimer.startMeasurement();

      qpSolver.clear();

      qpSolver.setUseWarmStart(useWarmStart);
      qpSolver.setMaxNumberOfIterations(maxNumberOfIterations);
      if (useWarmStart && pollResetActiveSet())
         qpSolver.resetActiveSet();

      finalSolverInput_H.set(solverInput_H);
      finalSolverInput_f.set(solverInput_f);
      finalSolverInput_H.addEquals(nativeSolverInput_H);
      finalSolverInput_f.addEquals(nativeSolverInput_f);

      if (solverInput_Ain.getNumRows() > 0)
      {
         finalSolverInput_Ain.reshape(solverInput_Ain.getNumRows() + nativeSolverInput_Ain.getNumRows(), problemSize);
         finalSolverInput_bin.reshape(solverInput_bin.getNumRows() + nativeSolverInput_bin.getNumRows(), 1);
         finalSolverInput_Ain.insert(solverInput_Ain, 0, 0);
         finalSolverInput_Ain.insert(nativeSolverInput_Ain, solverInput_Ain.getNumRows(), 0);
         finalSolverInput_bin.insert(solverInput_bin, 0, 0);
         finalSolverInput_bin.insert(nativeSolverInput_bin, solverInput_bin.getNumRows(), 0);
      }
      else
      {
         finalSolverInput_Ain.set(nativeSolverInput_Ain);
         finalSolverInput_bin.set(nativeSolverInput_bin);
      }

      if (solverInput_Aeq.getNumRows() > 0)
      {
         finalSolverInput_Aeq.reshape(solverInput_Aeq.getNumRows() + nativeSolverInput_Aeq.getNumRows(), problemSize);
         finalSolverInput_beq.reshape(solverInput_beq.getNumRows() + nativeSolverInput_beq.getNumRows(), 1);
         finalSolverInput_Aeq.insert(solverInput_Aeq, 0, 0);
         finalSolverInput_Aeq.insert(nativeSolverInput_Aeq, solverInput_Aeq.getNumRows(), 0);
         finalSolverInput_beq.insert(solverInput_beq, 0, 0);
         finalSolverInput_beq.insert(nativeSolverInput_beq, solverInput_beq.getNumRows(), 0);
      }
      else
      {
         finalSolverInput_Aeq.set(nativeSolverInput_Aeq);
         finalSolverInput_beq.set(nativeSolverInput_beq);
      }

      TIntArrayList inactiveIndices = applySubstitution(); // This needs to be done right before configuring the QP and solving.

      if (inactiveIndices != null)
      {
         for (int i = 0; i < inactiveIndices.size(); i++)
         {
            solverInput_activeIndices.set(inactiveIndices.get(i), 0, 0.0);
         }
      }

      numberOfActiveVariables.set((int) solverInput_activeIndices.sum());
      qpSolver.setActiveVariables(solverInput_activeIndices);

      numberOfIterations.set(qpSolver.solve(solverOutput));
      removeSubstitution(); // This needs to be done right after solving.

      qpSolverTimer.stopMeasurement();

      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (solverOutput.containsNaN())
      {
         return false;
      }

      solverOutput_jointAccelerations.insert(solverOutput, 0, numberOfDoFs, 0, 1, 0, 0);
      solverOutput_rhos.insert(solverOutput, numberOfDoFs, problemSize, 0, 1, 0, 0);

      addRateRegularization.set(true);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            // FIXME
            CommonOps_DDRM.mult(tempWrenchConstraint_J, new DMatrixRMaj(solverOutput), tempWrenchConstraint_LHS);
            int index = 0;
            wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         }
      }

      solverInput_H_previous.set(finalSolverInput_H);
      solverInput_f_previous.set(finalSolverInput_f);

      solverInput_lb_previous.set(solverInput_lb);
      solverInput_ub_previous.set(solverInput_ub);

      return true;
   }

   private TIntArrayList applySubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return null;

      accelerationVariablesSubstitution.applySubstitutionToObjectiveFunction(finalSolverInput_H, finalSolverInput_f);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(finalSolverInput_Aeq, finalSolverInput_beq);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(finalSolverInput_Ain, finalSolverInput_bin);
      accelerationVariablesSubstitution.applySubstitutionToBounds(solverInput_lb, solverInput_ub, nativeSolverInput_Ain, nativeSolverInput_bin);
      return accelerationVariablesSubstitution.getInactiveIndices();
   }

   private void removeSubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return;

      accelerationVariablesSubstitution.removeSubstitutionToSolution(solverOutput);
   }

   public NativeMatrix getJointAccelerations()
   {
      return solverOutput_jointAccelerations;
   }

   public NativeMatrix getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinJointAccelerations(double qDDotMin)
   {
      solverInput_lb.fillBlock(0, 0, numberOfDoFs, 1, qDDotMin);
   }

   public void setMinJointAccelerations(DMatrixRMaj qDDotMin)
   {
      solverInput_lb.insert(qDDotMin, 0, 0);
   }

   public void setMaxJointAccelerations(double qDDotMax)
   {
      solverInput_ub.fillBlock(0, 0, numberOfDoFs, 1, qDDotMax);
   }

   public void setMaxJointAccelerations(DMatrixRMaj qDDotMax)
   {
      solverInput_ub.insert(qDDotMax, 0, 0);
   }

   public void setMinRho(double rhoMin)
   {
      solverInput_lb.fillBlock(numberOfDoFs, 0, problemSize - numberOfDoFs, 1, rhoMin);
   }

   public void setMinRho(DMatrixRMaj rhoMin)
   {
      solverInput_lb.insert(rhoMin, numberOfDoFs, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      solverInput_ub.fillBlock(numberOfDoFs, 0, problemSize - numberOfDoFs, 1, rhoMax);
   }

   public void setMaxRho(DMatrixRMaj rhoMax)
   {
      solverInput_ub.insert(rhoMax, numberOfDoFs, 0);
   }

   public void setActiveDoF(int dofIndex, boolean active)
   {
      solverInput_activeIndices.set(dofIndex, 0, active ? 1.0 : 0.0);
   }

   public void setActiveRhos(DMatrixRMaj activeRhoMatrix)
   {
      solverInput_activeIndices.insert(activeRhoMatrix, numberOfDoFs, 0);
   }
}
