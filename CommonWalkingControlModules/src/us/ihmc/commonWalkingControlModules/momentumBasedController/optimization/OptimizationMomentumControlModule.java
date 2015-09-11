package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.Collection;
import java.util.EnumMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ActiveSetQPMomentumOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CQPMomentumBasedOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXMomentumOptimizerAdapter;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CompositeActiveSetQPSolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerInterface;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.OASESConstrainedQPSolver;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.QuadProgSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.optimization.EqualityConstraintEnforcer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.utilities.exceptions.NoConvergenceException;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public class OptimizationMomentumControlModule implements MomentumControlModule
{
   public enum QPSolverFlavor
   {
      CVX_NULL, EIGEN_NULL, EIGEN_DIRECT, EIGEN_ACTIVESET_NULL, EIGEN_ACTIVESET_DIRECT, EIGEN_ACTIVESET_DIRECT_JNA, CQP_OASES_DIRECT, CQP_QUADPROG_DIRECT,
      SIMPLE_ACTIVE_SET, SIMPLE_ACTIVE_SET_NULL
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
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final LinearSolver<DenseMatrix64F> hardMotionConstraintSolver;

   private final DenseMatrix64F dampedLeastSquaresFactorMatrix;
   private final DenseMatrix64F bOriginal = new DenseMatrix64F(Momentum.SIZE, 1);
   private final int nDoF;

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT, double gravityZ,
           MomentumOptimizationSettings momentumOptimizationSettings, TwistCalculator twistCalculator, GeometricJacobianHolder geometricJacobianHolder,
           Collection<? extends PlaneContactState> planeContactStates, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.jointsToOptimizeFor = momentumOptimizationSettings.getJointsToOptimizeFor();
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, controlDT, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint, planeContactStates);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler("primary", jointsToOptimizeFor, twistCalculator, geometricJacobianHolder, registry);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler("secondary", jointsToOptimizeFor, twistCalculator, geometricJacobianHolder, registry);

      nDoF = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);

      // setup solvers
      momentumOptimizers.put(QPSolverFlavor.CVX_NULL, new CVXMomentumOptimizerAdapter(nDoF, registry));
      
      final boolean ENABLE_EIGEN_ACTIVESET_QP = false;
      if(ENABLE_EIGEN_ACTIVESET_QP)
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
              wRhoSmoother, wRhoPenalizer, planeContactStates, registry);

      this.momentumOptimizationSettings = momentumOptimizationSettings;

      dampedLeastSquaresFactorMatrix = new DenseMatrix64F(nDoF, nDoF);

      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);

      this.hardMotionConstraintSolver = new DampedLeastSquaresSolver(1,momentumOptimizationSettings.getDampedLeastSquaresFactor());
      this.equalityConstraintEnforcer = new EqualityConstraintEnforcer(hardMotionConstraintSolver);

      parentRegistry.addChild(registry);
      reset();
   }

   public void setPrimaryMotionConstraintListener(MotionConstraintListener motionConstraintListener)
   {
      primaryMotionConstraintHandler.setMotionConstraintListener(motionConstraintListener);
   }

   public void setSecondaryMotionConstraintListener(MotionConstraintListener motionConstraintListener)
   {
      secondaryMotionConstraintHandler.setMotionConstraintListener(motionConstraintListener);
   }

   public void initialize()
   {
      centroidalMomentumHandler.initialize();
   }

   public void reset()
   {
      momentumRateOfChangeData.setEmpty();
      primaryMotionConstraintHandler.reset();
      secondaryMotionConstraintHandler.reset();
      externalWrenchHandler.reset();
   }

   private MomentumControlModuleSolverListener momentumControlModuleSolverListener;

   public void setMomentumControlModuleSolverListener(MomentumControlModuleSolverListener momentumControlModuleSolverListener)
   {
      if (this.momentumControlModuleSolverListener != null)
         throw new RuntimeException("MomentumControlModuleSolverListener is already set!");
      this.momentumControlModuleSolverListener = momentumControlModuleSolverListener;
   }

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
           throws MomentumControlModuleException

   {
      checkQPSolverSwitch();

      switch (currentQPSolver.getEnumValue())
      {
         case EIGEN_ACTIVESET_DIRECT_JNA :
         case EIGEN_ACTIVESET_DIRECT :
         case EIGEN_DIRECT :
         case CQP_OASES_DIRECT :
         case CQP_QUADPROG_DIRECT :
         case SIMPLE_ACTIVE_SET :

//          case CQP_JOPT_DIRECT:
            return compute(contactStates, upcomingSupportLeg, false);

         case CVX_NULL :
         case EIGEN_ACTIVESET_NULL :
         case EIGEN_NULL :
         case SIMPLE_ACTIVE_SET_NULL:
            return compute(contactStates, upcomingSupportLeg, true);

         default :
            throw new RuntimeException("Unlisted solverFlavor, please added it in the case above");
      }
   }

   public void checkQPSolverSwitch()
   {
      if (currentQPSolver.getEnumValue() != requestedQPSolver.getEnumValue())
      {
         if(momentumOptimizers.containsKey(requestedQPSolver.getEnumValue()))
         {
                 momentumOptimizer = momentumOptimizers.get(requestedQPSolver.getEnumValue());
                 if (momentumOptimizer instanceof ActiveSetQPMomentumOptimizer)
                    ((ActiveSetQPMomentumOptimizer) momentumOptimizer).loadNativeLibraries();
                 currentQPSolver.set(requestedQPSolver.getEnumValue());
         }
         else
         {
            System.out.println(getClass().getSimpleName() + ": requestedQPSolver is not loaded");
            requestedQPSolver.set(currentQPSolver.getEnumValue());
         }
      }
   }

   private MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg,
           boolean useNullSpaceProjection)
           throws MomentumControlModuleException

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
      DenseMatrix64F b = centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData);
      bOriginal.set(b);


      if (useNullSpaceProjection)
      {
         // put C1 into C2, find solution w \in null(J), ie: vd = null(J)w + J\p, from now on, we solve for w instead of vd

         /*
          *  A'= A*(I-J^+ J)
          *  b'=b- A J^+
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
      momentumOptimizationSettings.packDampedLeastSquaresFactorMatrix(dampedLeastSquaresFactorMatrix);

      // Lambda
      DenseMatrix64F momentumDotWeight = momentumOptimizationSettings.getMomentumDotWeight(momentumRateOfChangeData.getMomentumSubspace());

      if (useNullSpaceProjection)
      {
         momentumOptimizer.setInputs(a, b, momentumDotWeight, jSecondary, pSecondary, weightMatrixSecondary, wrenchMatrixCalculator.getWRho(),
                                     dampedLeastSquaresFactorMatrix, wrenchMatrixCalculator.getWRhoSmoother(), wrenchMatrixCalculator.getRhoPreviousAverage(),
                                     wrenchMatrixCalculator.getWRhoPenalizer(), wrenchMatrixCalculator.getQRho(), wrenchEquationRightHandSide,
                                     wrenchMatrixCalculator.getRhoMin(), wrenchMatrixCalculator.getQFeetCoP());
      }
      else
      {
         momentumOptimizer.setInputs(a, b, momentumDotWeight, jPrimary, pPrimary, jSecondary, pSecondary, weightMatrixSecondary,
                                     wrenchMatrixCalculator.getWRho(), dampedLeastSquaresFactorMatrix, wrenchMatrixCalculator.getWRhoSmoother(),
                                     wrenchMatrixCalculator.getRhoPreviousAverage(), wrenchMatrixCalculator.getWRhoPenalizer(),
                                     wrenchMatrixCalculator.getQRho(), wrenchEquationRightHandSide, wrenchMatrixCalculator.getRhoMin(),
                                     wrenchMatrixCalculator.getQFeetCoP());
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


      updateMomentumControlModuleSolverListener(jPrimary, pPrimary, a, b, jSecondary, pSecondary, weightMatrixSecondary, jointAccelerations);

      ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);

      SpatialForceVector centroidalMomentumRateSolution = centroidalMomentumHandler.getCentroidalMomentumRate();
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenches();
      MomentumModuleSolution momentumModuleSolution = new MomentumModuleSolution(jointsToOptimizeFor, jointAccelerations, centroidalMomentumRateSolution,
                                                         externalWrenchSolution);

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


   private void updateMomentumControlModuleSolverListener(DenseMatrix64F jPrimary, DenseMatrix64F pPrimary, DenseMatrix64F a, DenseMatrix64F b,
           DenseMatrix64F jSecondary, DenseMatrix64F pSecondary, DenseMatrix64F weightMatrixSecondary, DenseMatrix64F jointAccelerations)
   {
      if (momentumControlModuleSolverListener != null)
      {
         momentumControlModuleSolverListener.setPrimaryMotionConstraintJMatrix(jPrimary);
         momentumControlModuleSolverListener.setPrimaryMotionConstraintPVector(pPrimary);
         momentumControlModuleSolverListener.setCentroidalMomentumMatrix(a, b, momentumRateOfChangeData.getMomentumSubspace());

         DenseMatrix64F checkJQEqualsZeroAfterSetConstraint = equalityConstraintEnforcer.checkJQEqualsZeroAfterSetConstraint();
         momentumControlModuleSolverListener.setCheckJQEqualsZeroAfterSetConstraint(checkJQEqualsZeroAfterSetConstraint);

//       equalityConstraintEnforcer.computeCheck();
//       DenseMatrix64F checkCopy = equalityConstraintEnforcer.getCheckCopy();
//       momentumControlModuleSolverListener.setPrimaryMotionConstraintCheck(checkCopy);

         momentumControlModuleSolverListener.setSecondaryMotionConstraintJMatrix(jSecondary);
         momentumControlModuleSolverListener.setSecondaryMotionConstraintPVector(pSecondary);
         momentumControlModuleSolverListener.setSecondaryMotionConstraintWeightMatrix(weightMatrixSecondary);

         momentumControlModuleSolverListener.setJointAccelerationSolution(jointsToOptimizeFor, jointAccelerations);
         momentumControlModuleSolverListener.setOptimizationValue(momentumOptimizer.getOutputOptVal());
         momentumControlModuleSolverListener.reviewSolution();
      }
   }


   public void resetGroundReactionWrenchFilter()
   {
      // empty for now
   }


   public void setDesiredJointAcceleration(DesiredJointAccelerationCommand desiredJointAccelerationCommand)
   {
      if (desiredJointAccelerationCommand.getHasWeight())
      {
         secondaryMotionConstraintHandler.setDesiredJointAcceleration(desiredJointAccelerationCommand);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredJointAcceleration(desiredJointAccelerationCommand);    // weight is arbitrary, actually
      }
   }


   public void setDesiredSpatialAcceleration(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand)
   {
      if (desiredSpatialAccelerationCommand.getHasWeight())
      {
         secondaryMotionConstraintHandler.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);    // weight is arbitrary,
      }
   }


   public void setDesiredPointAcceleration(DesiredPointAccelerationCommand desiredPointAccelerationCommand)
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


   public void setDesiredRateOfChangeOfMomentum(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand)
   {
      this.momentumRateOfChangeData.set(desiredRateOfChangeOfMomentumCommand.getMomentumRateOfChangeData());
   }


   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void setFootCoPControlData(RobotSide side, ReferenceFrame frame)
   {
      wrenchMatrixCalculator.setFootCoPControlData(side, frame);
   }


// public SpatialForceVector getDesiredCentroidalMomentumRate()
// {
//    return centroidalMomentumHandler.getCentroidalMomentumRate();
// }
//
// public Map<RigidBody, Wrench> getExternalWrenches()
// {
//    return externalWrenchHandler.getExternalWrenches();
// }

   public void setWRhoSmoother(double wRhoSmoother)
   {
      wrenchMatrixCalculator.setWRhoSmoother(wRhoSmoother);
   }
}
