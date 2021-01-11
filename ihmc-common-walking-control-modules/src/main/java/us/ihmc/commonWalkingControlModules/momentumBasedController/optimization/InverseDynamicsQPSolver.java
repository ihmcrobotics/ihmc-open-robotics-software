package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.convexOptimization.quadraticProgram.ActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.matrixlib.MatrixTools;
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
   private final ActiveSetQPSolverWithInactiveVariablesInterface qpSolver;

   private final QPVariableSubstitution accelerationVariablesSubstitution = new QPVariableSubstitution();

   private final DMatrixRMaj solverInput_H;
   private final DMatrixRMaj solverInput_f;

   private final DMatrixRMaj solverInput_H_previous;
   private final DMatrixRMaj solverInput_f_previous;

   private final DMatrixRMaj solverInput_Aeq;
   private final DMatrixRMaj solverInput_beq;
   private final DMatrixRMaj solverInput_Ain;
   private final DMatrixRMaj solverInput_bin;

   private final DMatrixRMaj solverInput_lb;
   private final DMatrixRMaj solverInput_ub;

   private final DMatrixRMaj solverInput_lb_previous;
   private final DMatrixRMaj solverInput_ub_previous;

   private final DMatrixRMaj solverInput_activeIndices;

   private final DMatrixRMaj solverOutput;
   private final DMatrixRMaj solverOutput_jointAccelerations;
   private final DMatrixRMaj solverOutput_rhos;

   private final YoInteger numberOfActiveVariables = new YoInteger("numberOfActiveVariables", registry);
   private final YoInteger numberOfIterations = new YoInteger("numberOfIterations", registry);
   private final YoInteger numberOfEqualityConstraints = new YoInteger("numberOfEqualityConstraints", registry);
   private final YoInteger numberOfInequalityConstraints = new YoInteger("numberOfInequalityConstraints", registry);
   private final YoInteger numberOfConstraints = new YoInteger("numberOfConstraints", registry);
   private final YoDouble jointAccelerationRegularization = new YoDouble("jointAccelerationRegularization", registry);
   private final YoDouble jointJerkRegularization = new YoDouble("jointJerkRegularization", registry);
   private final YoDouble jointTorqueWeight = new YoDouble("jointTorqueWeight", registry);
   private final DMatrixRMaj regularizationMatrix;

   private final DMatrixRMaj tempJtW;

   private final int numberOfDoFs;
   private final int rhoSize;
   private final int problemSize;
   private final boolean hasFloatingBase;
   private boolean hasWrenchesEquilibriumConstraintBeenSetup = false;

   private boolean resetActiveSet = false;
   private boolean useWarmStart = false;
   private int maxNumberOfIterations = 100;

   private final double dt;

   public InverseDynamicsQPSolver(ActiveSetQPSolverWithInactiveVariablesInterface qpSolver,
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

      solverInput_H = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f = new DMatrixRMaj(problemSize, 1);

      solverInput_H_previous = new DMatrixRMaj(problemSize, problemSize);
      solverInput_f_previous = new DMatrixRMaj(problemSize, 1);

      solverInput_Aeq = new DMatrixRMaj(0, problemSize);
      solverInput_beq = new DMatrixRMaj(0, 1);
      solverInput_Ain = new DMatrixRMaj(0, problemSize);
      solverInput_bin = new DMatrixRMaj(0, 1);

      solverInput_lb = new DMatrixRMaj(problemSize, 1);
      solverInput_ub = new DMatrixRMaj(problemSize, 1);

      solverInput_lb_previous = new DMatrixRMaj(problemSize, 1);
      solverInput_ub_previous = new DMatrixRMaj(problemSize, 1);

      CommonOps_DDRM.fill(solverInput_lb, Double.NEGATIVE_INFINITY);
      CommonOps_DDRM.fill(solverInput_ub, Double.POSITIVE_INFINITY);

      solverInput_activeIndices = new DMatrixRMaj(problemSize, 1);
      CommonOps_DDRM.fill(solverInput_activeIndices, 1.0);

      solverOutput = new DMatrixRMaj(problemSize, 1);
      solverOutput_jointAccelerations = new DMatrixRMaj(numberOfDoFs, 1);
      solverOutput_rhos = new DMatrixRMaj(rhoSize, 1);

      tempJtW = new DMatrixRMaj(problemSize, problemSize);

      jointAccelerationRegularization.set(0.005);
      jointJerkRegularization.set(0.1);
      jointTorqueWeight.set(0.001);
      regularizationMatrix = new DMatrixRMaj(problemSize, problemSize);

      for (int i = 0; i < numberOfDoFs; i++)
         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());
      double defaultRhoRegularization = 0.00001;
      for (int i = numberOfDoFs; i < problemSize; i++)
         regularizationMatrix.set(i, i, defaultRhoRegularization);

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
      CommonOps_DDRM.insert(weight, regularizationMatrix, numberOfDoFs, numberOfDoFs);
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
      for (int i = 0; i < numberOfDoFs; i++)
         regularizationMatrix.set(i, i, jointAccelerationRegularization.getDoubleValue());

      accelerationVariablesSubstitution.reset();

      solverInput_H.zero();

      solverInput_f.zero();

      solverInput_Aeq.reshape(0, problemSize);
      solverInput_beq.reshape(0, 1);

      solverInput_Ain.reshape(0, problemSize);
      solverInput_bin.reshape(0, 1);
   }

   private void addRegularization()
   {
      CommonOps_DDRM.addEquals(solverInput_H, regularizationMatrix);

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
      for (int i = 0; i < numberOfDoFs; i++)
      {
         solverInput_H.add(i, i, 1.0 / factor);
         solverInput_f.add(i, 0, -solverOutput_jointAccelerations.get(i, 0) / factor);
      }
   }

   public void addMotionInput(QPInputTypeA input)
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

   public void addRhoInput(QPInputTypeA input)
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

   public void addMotionTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double taskWeight)
   {
      if (taskJacobian.getNumCols() != numberOfDoFs)
      {
         throw new RuntimeException("Motion task needs to have size macthing the DoFs of the robot.");
      }
      addTaskInternal(taskJacobian, taskObjective, taskWeight, 0);
   }

   public void addRhoTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double taskWeight)
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
   public void addMotionTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
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
   public void addRhoTask(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
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
      addTaskInternal(taskObjective, taskWeight, numberOfDoFs, rhoSize);
   }

   public void addMotionTask(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight)
   {
      addTaskInternal(taskObjective, taskWeight, 0, numberOfDoFs);
   }

   public void addTaskInternal(DMatrixRMaj taskObjective, DMatrixRMaj taskWeight, int offset, int variables)
   {
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      MatrixTools.addMatrixBlock(solverInput_H, offset, offset, taskWeight, 0, 0, variables, variables, 1.0);
      MatrixTools.multAddBlock(-1.0, taskWeight, taskObjective, solverInput_f, offset, 0);
   }

   private void addTaskInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, DMatrixRMaj taskWeight, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      tempJtW.reshape(variables, taskSize);

      // J^T W
      CommonOps_DDRM.multTransA(taskJacobian, taskWeight, tempJtW);

      // Compute: H += J^T W J
      MatrixTools.multAddBlock(tempJtW, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += - J^T W Objective
      MatrixTools.multAddBlock(-1.0, tempJtW, taskObjective, solverInput_f, offset, 0);
   }

   private void addTaskInternal(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective, double taskWeight, int offset)
   {
      int variables = taskJacobian.getNumCols();
      if (offset + variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      MatrixTools.multAddBlockInner(taskWeight, taskJacobian, solverInput_H, offset, offset);

      // Compute: f += - J^T W Objective
      MatrixTools.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, solverInput_f, offset, 0);
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

   public void addRhoEqualityConstraint(DMatrixRMaj taskJacobian, DMatrixRMaj taskObjective)
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
      CommonOps_DDRM.insert(taskObjective, solverInput_beq, previousSize, offset);
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

   public void addAccelerationSubstitution(QPVariableSubstitution substitution)
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

      TIntArrayList inactiveIndices = applySubstitution(); // This needs to be done right before configuring the QP and solving.

      if (inactiveIndices != null)
      {
         for (int i = 0; i < inactiveIndices.size(); i++)
         {
            solverInput_activeIndices.set(inactiveIndices.get(i), 0, 0.0);
         }
      }

      numberOfActiveVariables.set((int) CommonOps_DDRM.elementSum(solverInput_activeIndices));
      qpSolver.setQuadraticCostFunction(solverInput_H, solverInput_f);
      qpSolver.setVariableBounds(solverInput_lb, solverInput_ub);
      qpSolver.setActiveVariables(solverInput_activeIndices);
      qpSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);
      qpSolver.setLinearEqualityConstraints(solverInput_Aeq, solverInput_beq);

      numberOfIterations.set(qpSolver.solve(solverOutput));
      removeSubstitution(); // This needs to be done right after solving.

      qpSolverTimer.stopMeasurement();

      hasWrenchesEquilibriumConstraintBeenSetup = false;

      if (MatrixTools.containsNaN(solverOutput))
      {
         return false;
      }

      CommonOps_DDRM.extract(solverOutput, 0, numberOfDoFs, 0, 1, solverOutput_jointAccelerations, 0, 0);
      CommonOps_DDRM.extract(solverOutput, numberOfDoFs, problemSize, 0, 1, solverOutput_rhos, 0, 0);

      addRateRegularization.set(true);

      if (SETUP_WRENCHES_CONSTRAINT_AS_OBJECTIVE)
      {
         if (hasFloatingBase)
         {
            CommonOps_DDRM.mult(tempWrenchConstraint_J, solverOutput, tempWrenchConstraint_LHS);
            int index = 0;
            wrenchEquilibriumTorqueError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumTorqueError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setX(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setY(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
            wrenchEquilibriumForceError.setZ(tempWrenchConstraint_LHS.get(index, 0) - tempWrenchConstraint_RHS.get(index++, 0));
         }
      }

      solverInput_H_previous.set(solverInput_H);
      solverInput_f_previous.set(solverInput_f);

      solverInput_lb_previous.set(solverInput_lb);
      solverInput_ub_previous.set(solverInput_ub);

      return true;
   }

   private TIntArrayList applySubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return null;

      accelerationVariablesSubstitution.applySubstitutionToObjectiveFunction(solverInput_H, solverInput_f);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(solverInput_Aeq, solverInput_beq);
      accelerationVariablesSubstitution.applySubstitutionToLinearConstraint(solverInput_Ain, solverInput_bin);
      accelerationVariablesSubstitution.applySubstitutionToBounds(solverInput_lb, solverInput_ub, solverInput_Ain, solverInput_bin);
      return accelerationVariablesSubstitution.getInactiveIndices();
   }

   private void removeSubstitution()
   {
      if (accelerationVariablesSubstitution.isEmpty())
         return;

      accelerationVariablesSubstitution.removeSubstitutionToSolution(solverOutput);
   }

   private void printForJerry()
   {
      MatrixTools.printJavaForConstruction("H", solverInput_H);
      MatrixTools.printJavaForConstruction("f", solverInput_f);
      MatrixTools.printJavaForConstruction("lowerBounds", solverInput_lb);
      MatrixTools.printJavaForConstruction("upperBounds", solverInput_ub);
      MatrixTools.printJavaForConstruction("solution", solverOutput);
   }

   public DMatrixRMaj getJointAccelerations()
   {
      return solverOutput_jointAccelerations;
   }

   public DMatrixRMaj getRhos()
   {
      return solverOutput_rhos;
   }

   public void setMinJointAccelerations(double qDDotMin)
   {
      for (int i = 4; i < numberOfDoFs; i++)
         solverInput_lb.set(i, 0, qDDotMin);
   }

   public void setMinJointAccelerations(DMatrixRMaj qDDotMin)
   {
      CommonOps_DDRM.insert(qDDotMin, solverInput_lb, 0, 0);
   }

   public void setMaxJointAccelerations(double qDDotMax)
   {
      for (int i = 4; i < numberOfDoFs; i++)
         solverInput_ub.set(i, 0, qDDotMax);
   }

   public void setMaxJointAccelerations(DMatrixRMaj qDDotMax)
   {
      CommonOps_DDRM.insert(qDDotMax, solverInput_ub, 0, 0);
   }

   public void setMinRho(double rhoMin)
   {
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_lb.set(i, 0, rhoMin);
   }

   public void setMinRho(DMatrixRMaj rhoMin)
   {
      CommonOps_DDRM.insert(rhoMin, solverInput_lb, numberOfDoFs, 0);
   }

   public void setMaxRho(double rhoMax)
   {
      for (int i = numberOfDoFs; i < problemSize; i++)
         solverInput_ub.set(i, 0, rhoMax);
   }

   public void setMaxRho(DMatrixRMaj rhoMax)
   {
      CommonOps_DDRM.insert(rhoMax, solverInput_ub, numberOfDoFs, 0);
   }

   public void setActiveRhos(DMatrixRMaj activeRhoMatrix)
   {
      CommonOps_DDRM.insert(activeRhoMatrix, solverInput_activeIndices, numberOfDoFs, 0);
   }
}
