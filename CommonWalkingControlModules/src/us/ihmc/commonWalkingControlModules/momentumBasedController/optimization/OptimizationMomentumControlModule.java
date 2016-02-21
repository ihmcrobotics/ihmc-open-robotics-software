package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ActiveSetQPMomentumOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CQPMomentumBasedOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXMomentumOptimizerAdapter;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CompositeActiveSetQPSolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.QuadProgSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PlaneContactStateCommandPool;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.optimization.EqualityConstraintEnforcer;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class OptimizationMomentumControlModule
{
   public enum QPSolverFlavor
   {
      CVX_NULL,
      EIGEN_NULL,
      EIGEN_DIRECT,
      EIGEN_ACTIVESET_NULL,
      EIGEN_ACTIVESET_DIRECT,
      EIGEN_ACTIVESET_DIRECT_JNA,
      CQP_OASES_DIRECT,
      CQP_QUADPROG_DIRECT,
      SIMPLE_ACTIVE_SET,
      SIMPLE_ACTIVE_SET_NULL
   }

   ;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final ExternalWrenchHandler externalWrenchHandler;

   private final EqualityConstraintEnforcer equalityConstraintEnforcer;
   private final MotionConstraintHandler primaryMotionConstraintHandler;
   private final MotionConstraintHandler secondaryMotionConstraintHandler;

   private final PlaneContactWrenchMatrixCalculator wrenchMatrixCalculator;

   private final Map<QPSolverFlavor, MomentumOptimizerInterface> momentumOptimizers = new EnumMap<>(QPSolverFlavor.class);
   private MomentumOptimizerInterface momentumOptimizer;
   private MomentumOptimizationSettings momentumOptimizationSettings;

   private final BooleanYoVariable converged = new BooleanYoVariable("converged", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);
   private final ExecutionTimer probingTimer = new ExecutionTimer("probingTimer", 0, registry);

   private final boolean INSPECT_CONSTRAINTS = false;
   private final DoubleYoVariable primaryResNorm = new DoubleYoVariable("primaryResNorm", registry);
   private final DoubleYoVariable secondaryResNorm = new DoubleYoVariable("secondaryResNorm", registry);

   private final EnumYoVariable<QPSolverFlavor> requestedQPSolver = new EnumYoVariable<>("requestedQPSolver", registry, QPSolverFlavor.class);
   private final EnumYoVariable<QPSolverFlavor> currentQPSolver = new EnumYoVariable<>("currentQPSolver", registry, QPSolverFlavor.class);
   private final IntegerYoVariable qpSolverIteration = new IntegerYoVariable("qpSolverInteration", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final LinearSolver<DenseMatrix64F> hardMotionConstraintSolver;

   private final DenseMatrix64F dampedLeastSquaresFactorMatrix;
   private final DenseMatrix64F bOriginal = new DenseMatrix64F(Momentum.SIZE, 1);
   private final int nDoF;

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double gravityZ, MomentumOptimizationSettings momentumOptimizationSettings,
         TwistCalculator twistCalculator, GeometricJacobianHolder geometricJacobianHolder, List<? extends ContactablePlaneBody> contactablePlaneBodies,
         YoVariableRegistry parentRegistry)
   {
      this.jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint, contactablePlaneBodies);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler("primary", jointsToOptimizeFor, twistCalculator, geometricJacobianHolder, registry);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler("secondary", jointsToOptimizeFor, twistCalculator, geometricJacobianHolder, registry);

      nDoF = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);

      // setup solvers
      momentumOptimizers.put(QPSolverFlavor.CVX_NULL, new CVXMomentumOptimizerAdapter(nDoF, registry));

      final boolean ENABLE_EIGEN_ACTIVESET_QP = false;
      if (ENABLE_EIGEN_ACTIVESET_QP)
      {
         ActiveSetQPMomentumOptimizer eigenQP = new ActiveSetQPMomentumOptimizer(nDoF, false)
         {
            public int solve() throws NoConvergenceException
            {
               return super.solve(true);
            }
         };
         eigenQP.setSaveNoConvergeProblem(false);
         momentumOptimizers.put(QPSolverFlavor.EIGEN_ACTIVESET_NULL, new ActiveSetQPMomentumOptimizer(nDoF, false));
         momentumOptimizers.put(QPSolverFlavor.EIGEN_ACTIVESET_DIRECT, new ActiveSetQPMomentumOptimizer(nDoF, false));
         momentumOptimizers.put(QPSolverFlavor.EIGEN_ACTIVESET_DIRECT_JNA, new ActiveSetQPMomentumOptimizer(nDoF, true));

         momentumOptimizers.put(QPSolverFlavor.EIGEN_NULL, eigenQP);
         momentumOptimizers.put(QPSolverFlavor.EIGEN_DIRECT, eigenQP);
      }
      momentumOptimizers.put(QPSolverFlavor.CQP_OASES_DIRECT, new CQPMomentumBasedOptimizer(nDoF, new OASESConstrainedQPSolver(registry)));
      momentumOptimizers.put(QPSolverFlavor.SIMPLE_ACTIVE_SET, new CQPMomentumBasedOptimizer(nDoF, new CompositeActiveSetQPSolver(registry)));
      momentumOptimizers.put(QPSolverFlavor.SIMPLE_ACTIVE_SET_NULL, new CQPMomentumBasedOptimizer(nDoF, new CompositeActiveSetQPSolver(registry)));
      momentumOptimizers.put(QPSolverFlavor.CQP_QUADPROG_DIRECT, new CQPMomentumBasedOptimizer(nDoF, new QuadProgSolver(registry)));

      //    momentumOptimizers.put(QPSolverFlavor.CQP_JOPT_DIRECT, new CQPMomentumBasedOptimizer(nDoF, new JOptimizerConstrainedQPSolver()));

      // initialize default solver
      final QPSolverFlavor defaultSolver = QPSolverFlavor.CVX_NULL;
      requestedQPSolver.set(defaultSolver);
      currentQPSolver.set(defaultSolver);
      momentumOptimizer = momentumOptimizers.get(defaultSolver);

      int rhoSize = momentumOptimizer.getRhoSize();
      int nPointsPerPlane = momentumOptimizer.getNPointsPerPlane();
      int nSupportVectors = momentumOptimizer.getNSupportVectors();
      double wRhoPlaneContacts = momentumOptimizationSettings.getRhoPlaneContactRegularization();
      double wRhoSmoother = momentumOptimizationSettings.getRateOfChangeOfRhoPlaneContactRegularization();
      double wRhoPenalizer = momentumOptimizationSettings.getPenalizerOfRhoPlaneContactRegularization();

      wrenchMatrixCalculator = new PlaneContactWrenchMatrixCalculator(centerOfMassFrame, rhoSize, nPointsPerPlane, nSupportVectors, wRhoPlaneContacts,
            wRhoSmoother, wRhoPenalizer, contactablePlaneBodies, registry);

      this.momentumOptimizationSettings = momentumOptimizationSettings;

      dampedLeastSquaresFactorMatrix = new DenseMatrix64F(nDoF, nDoF);

      this.hardMotionConstraintSolver = new DampedLeastSquaresSolver(1, momentumOptimizationSettings.getDampedLeastSquaresFactor());
      this.equalityConstraintEnforcer = new EqualityConstraintEnforcer(hardMotionConstraintSolver);

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize()
   {
      centroidalMomentumHandler.initialize();
   }

   public void reset()
   {
      momentumRateCommand.setEmpty();
      primaryMotionConstraintHandler.reset();
      secondaryMotionConstraintHandler.reset();
      externalWrenchHandler.reset();
   }

   public MomentumModuleSolution compute() throws MomentumControlModuleException

   {
      checkQPSolverSwitch();

      switch (currentQPSolver.getEnumValue())
      {
      case EIGEN_ACTIVESET_DIRECT_JNA:
      case EIGEN_ACTIVESET_DIRECT:
      case EIGEN_DIRECT:
      case CQP_OASES_DIRECT:
      case CQP_QUADPROG_DIRECT:
      case SIMPLE_ACTIVE_SET:

         //          case CQP_JOPT_DIRECT:
         return compute(false);

      case CVX_NULL:
      case EIGEN_ACTIVESET_NULL:
      case EIGEN_NULL:
      case SIMPLE_ACTIVE_SET_NULL:
         return compute(true);

      default:
         throw new RuntimeException("Unlisted solverFlavor, please added it in the case above");
      }
   }

   private void checkQPSolverSwitch()
   {
      if (currentQPSolver.getEnumValue() != requestedQPSolver.getEnumValue())
      {
         if (momentumOptimizers.containsKey(requestedQPSolver.getEnumValue()))
         {
            momentumOptimizer = momentumOptimizers.get(requestedQPSolver.getEnumValue());
            if (momentumOptimizer instanceof ActiveSetQPMomentumOptimizer)
               ActiveSetQPMomentumOptimizer.loadNativeLibraries();
            currentQPSolver.set(requestedQPSolver.getEnumValue());
         }
         else
         {
            System.out.println(getClass().getSimpleName() + ": requestedQPSolver is not loaded");
            requestedQPSolver.set(currentQPSolver.getEnumValue());
         }
      }
   }

   private MomentumModuleSolution compute(boolean useNullSpaceProjection) throws MomentumControlModuleException

   {
      wrenchMatrixCalculator.setRhoMinScalar(momentumOptimizationSettings.getRhoMinScalar());

      momentumOptimizer.reset();

      primaryMotionConstraintHandler.compute();
      centroidalMomentumHandler.compute();

      // C1:  J vd = p
      DenseMatrix64F jPrimary = primaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pPrimary = primaryMotionConstraintHandler.getRightHandSide();

      // C2:  A vd = b = hd - Ad v
      DenseMatrix64F a = centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      DenseMatrix64F b = centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateCommand);
      bOriginal.set(b);

      if (useNullSpaceProjection)
      {
         // put C1 into C2, find solution w \in null(J), ie: vd = null(J)w + J\p, from now on, we solve for w instead of vd

         /*
          * A'= A*(I-J^+ J) b'=b- A J^+
          */
         equalityConstraintEnforcer.setConstraint(jPrimary, pPrimary);
         equalityConstraintEnforcer.constrainEquation(a, b);
      }

      // -(\sum Wi) + Ad v
      wrenchMatrixCalculator.computeMatrices();
      DenseMatrix64F centroidalMomentumConvectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();
      DenseMatrix64F wrenchEquationRightHandSide = externalWrenchHandler.computeWrenchEquationRightHandSide(centroidalMomentumConvectiveTerm, bOriginal, b);

      // Js vd = ps
      secondaryMotionConstraintHandler.compute();
      DenseMatrix64F jSecondary = secondaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pSecondary = secondaryMotionConstraintHandler.getRightHandSide();
      DenseMatrix64F weightMatrixSecondary = secondaryMotionConstraintHandler.getWeightMatrix();
      if (useNullSpaceProjection)
      {
         equalityConstraintEnforcer.constrainEquation(jSecondary, pSecondary);
      }

      // C
      momentumOptimizationSettings.getDampedLeastSquaresFactorMatrix(dampedLeastSquaresFactorMatrix);

      // Lambda
      DenseMatrix64F momentumDotWeight = momentumOptimizationSettings.getMomentumDotWeight(momentumRateCommand.getMomentumSubspace());

      if (useNullSpaceProjection)
      {
         momentumOptimizer.setInputs(a, b, momentumDotWeight, jSecondary, pSecondary, weightMatrixSecondary, wrenchMatrixCalculator.getWRho(),
               dampedLeastSquaresFactorMatrix, wrenchMatrixCalculator.getWRhoSmoother(), wrenchMatrixCalculator.getRhoPreviousAverage(),
               wrenchMatrixCalculator.getWRhoPenalizer(), wrenchMatrixCalculator.getQRho(), wrenchEquationRightHandSide, wrenchMatrixCalculator.getRhoMin());
      }
      else
      {
         momentumOptimizer.setInputs(a, b, momentumDotWeight, jPrimary, pPrimary, jSecondary, pSecondary, weightMatrixSecondary,
               wrenchMatrixCalculator.getWRho(), dampedLeastSquaresFactorMatrix, wrenchMatrixCalculator.getWRhoSmoother(),
               wrenchMatrixCalculator.getRhoPreviousAverage(), wrenchMatrixCalculator.getWRhoPenalizer(), wrenchMatrixCalculator.getQRho(),
               wrenchEquationRightHandSide, wrenchMatrixCalculator.getRhoMin());
      }

      NoConvergenceException noConvergenceException = null;
      try
      {
         probingTimer.startMeasurement();
         optimize();
         probingTimer.stopMeasurement();
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenches(momentumOptimizer.getOutputRho());

      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);

      DenseMatrix64F jointAccelerations;
      if (useNullSpaceProjection)
      {
         jointAccelerations = equalityConstraintEnforcer.constrainResult(momentumOptimizer.getOutputJointAccelerations());
      }
      else
      {
         jointAccelerations = momentumOptimizer.getOutputJointAccelerations();
      }

      // check constraint quality
      if (INSPECT_CONSTRAINTS)
      {
         primaryResNorm.set(LinearConstraintResNorm(jPrimary, pPrimary, jointAccelerations));
         secondaryResNorm.set(LinearConstraintResNorm(jSecondary, pSecondary, jointAccelerations));
      }

      ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);

      SpatialForceVector centroidalMomentumRateSolution = centroidalMomentumHandler.getCentroidalMomentumRate();
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenchMap();
      List<RigidBody> rigidBodiesWithExternalWrench = externalWrenchHandler.getRigidBodiesWithExternalWrench();
      MomentumModuleSolution momentumModuleSolution = new MomentumModuleSolution(jointsToOptimizeFor, jointAccelerations, centroidalMomentumRateSolution,
            externalWrenchSolution, rigidBodiesWithExternalWrench);

      if (noConvergenceException != null)
      {
         throw new MomentumControlModuleException(noConvergenceException, momentumModuleSolution);
      }

      return momentumModuleSolution;
   }

   private double LinearConstraintResNorm(DenseMatrix64F J, DenseMatrix64F p, DenseMatrix64F x)
   {
      DenseMatrix64F resVector = p.copy();
      CommonOps.multAdd(-1, J, x, resVector);
      double norm = NormOps.normP1(resVector);

      return norm;
   }

   private void optimize() throws NoConvergenceException
   {
      try
      {
         qpSolverIteration.set(momentumOptimizer.solve());
         converged.set(true);
      }
      catch (NoConvergenceException e)
      {
         qpSolverIteration.set(e.getIter());

         if (!hasNotConvergedInPast.getBooleanValue())
         {
            e.printStackTrace();
            System.err.println("WARNING: Only showing the stack trace of the first " + e.getClass().getSimpleName()
                  + ". This may be happening more than once. See value of YoVariable " + converged.getName() + ".");
         }

         converged.set(false);
         hasNotConvergedInPast.set(true);
         hasNotConvergedCounts.increment();

         throw e;
      }
   }

   public void setInverseDynamicsCommand(InverseDynamicsCommand<?> inverseDynamicsCommand)
   {
      switch (inverseDynamicsCommand.getCommandType())
      {
      case TASKSPACE_MOTION:
         setDesiredSpatialAcceleration((SpatialAccelerationCommand) inverseDynamicsCommand);
         return;
      case TASKSPACE_POINT_MOTION:
         setDesiredPointAcceleration((PointAccelerationCommand) inverseDynamicsCommand);
         return;
      case JOINTSPACE_MOTION:
         setDesiredJointAcceleration((JointspaceAccelerationCommand) inverseDynamicsCommand);
         return;
      case MOMENTUM_RATE:
         setDesiredRateOfChangeOfMomentum((MomentumRateCommand) inverseDynamicsCommand);
         return;
      case EXTERNAL_WRENCH:
         setExternalWrenchToCompensateFor((ExternalWrenchCommand) inverseDynamicsCommand);
         return;
      case PLANE_CONTACT_STATE:
         setPlaneContactStateCommand((PlaneContactStateCommand) inverseDynamicsCommand);
         return;
      case PLANE_CONTACT_STATE_POOL:
         setPlaneContactStateCommandPool((PlaneContactStateCommandPool) inverseDynamicsCommand);
         return;
      case COMMAND_LIST:
         setInverseDynamicsCommandList((InverseDynamicsCommandList) inverseDynamicsCommand);
         return;
      default:
         throw new RuntimeException("The command type: " + inverseDynamicsCommand.getCommandType() + " is not handled.");
      }
   }

   private void setInverseDynamicsCommandList(InverseDynamicsCommandList inverseDynamicsCommandList)
   {
      for (int i = 0; i < inverseDynamicsCommandList.getNumberOfCommands(); i++)
         setInverseDynamicsCommand(inverseDynamicsCommandList.getCommand(i));
   }

   private void setDesiredJointAcceleration(JointspaceAccelerationCommand desiredJointAccelerationCommand)
   {
      if (desiredJointAccelerationCommand.getHasWeight())
      {
         secondaryMotionConstraintHandler.setDesiredJointAcceleration(desiredJointAccelerationCommand);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredJointAcceleration(desiredJointAccelerationCommand); // weight is arbitrary, actually
      }
   }

   private void setDesiredSpatialAcceleration(SpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      if (desiredSpatialAccelerationCommand.getHasWeight())
      {
         secondaryMotionConstraintHandler.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand); // weight is arbitrary,
      }
   }

   private void setDesiredPointAcceleration(PointAccelerationCommand desiredPointAccelerationCommand)
   {
      GeometricJacobian rootToEndEffectorJacobian = desiredPointAccelerationCommand.getRootToEndEffectorJacobian();
      FramePoint bodyFixedPoint = desiredPointAccelerationCommand.getContactPoint();
      FrameVector desiredAccelerationWithRespectToBase = desiredPointAccelerationCommand.getDesiredAcceleration();
      DenseMatrix64F selectionMatrix = desiredPointAccelerationCommand.getSelectionMatrix();

      if (selectionMatrix != null)
      {
         primaryMotionConstraintHandler.setDesiredPointAcceleration(rootToEndEffectorJacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase,
               selectionMatrix, Double.POSITIVE_INFINITY);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredPointAcceleration(rootToEndEffectorJacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase,
               Double.POSITIVE_INFINITY);
      }
   }

   private void setDesiredRateOfChangeOfMomentum(MomentumRateCommand momentumRateCommand)
   {
      this.momentumRateCommand.set(momentumRateCommand);
   }

   private void setPlaneContactStateCommand(PlaneContactStateCommand contactStateCommand)
   {
      wrenchMatrixCalculator.setPlaneContactStateCommand(contactStateCommand);
   }

   private void setPlaneContactStateCommandPool(PlaneContactStateCommandPool contactStateCommandPool)
   {
      wrenchMatrixCalculator.setPlaneContactStateCommandPool(contactStateCommandPool);
   }

   private void setExternalWrenchToCompensateFor(ExternalWrenchCommand externalWrenchCommand)
   {
      RigidBody rigidBody = externalWrenchCommand.getRigidBody();
      Wrench wrench = externalWrenchCommand.getExternalWrench();
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }
}
