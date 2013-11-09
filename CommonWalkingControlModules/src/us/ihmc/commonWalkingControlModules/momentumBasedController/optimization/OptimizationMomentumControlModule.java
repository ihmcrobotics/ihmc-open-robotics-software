package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXGenMomentumOptimizerBridge;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXGenMomentumOptimizerBridge.MomentumOptimizer;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXWithCylinderNative;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactMatrixCalculatorAdapter;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactMatrixCalculatorAdapterwRhowPhiObtainedFromContactStates;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.optimization.EqualityConstraintEnforcer;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

/**
 * @author twan
 *         Date: 4/25/13
 */
public class OptimizationMomentumControlModule implements MomentumControlModule
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final ExternalWrenchHandler externalWrenchHandler;

   private final EqualityConstraintEnforcer equalityConstraintEnforcer;
   private final MotionConstraintHandler primaryMotionConstraintHandler;
   private final MotionConstraintHandler secondaryMotionConstraintHandler;

// private final ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator;

   private final CylinderAndPlaneContactMatrixCalculatorAdapter wrenchMatrixCalculator;

   private final CVXGenMomentumOptimizerBridge momentumOptimizer;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final BooleanYoVariable converged = new BooleanYoVariable("converged", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final IntegerYoVariable hasNotConvergedCounts = new IntegerYoVariable("hasNotConvergedCounts", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final DampedLeastSquaresSolver hardMotionConstraintSolver;
   
   private final DenseMatrix64F dampedLeastSquaresFactorMatrix;
   private final DenseMatrix64F bOriginal = new DenseMatrix64F(Momentum.SIZE, 1);

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT,
           InverseDynamicsJoint[] jointsToOptimizeFor, MomentumOptimizationSettings momentumOptimizationSettings, double gravityZ,
           TwistCalculator twistCalculator, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, controlDT, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler("primary", jointsToOptimizeFor, twistCalculator, registry);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler("secondary", jointsToOptimizeFor, twistCalculator, registry);

      int rhoSize = CVXWithCylinderNative.rhoSize;
      int phiSize = CVXWithCylinderNative.phiSize;
      dynamicGraphicObjectsListRegistry = null; // don't visualize vectors
      double wRhoCylinderContacts = momentumOptimizationSettings.getRhoCylinderContactRegularization();
      double wPhiCylinderContacts = momentumOptimizationSettings.getPhiCylinderContactRegularization();
      double wRhoPlaneContacts = momentumOptimizationSettings.getRhoPlaneContactRegularization();
      double wRhoSmoother = momentumOptimizationSettings.getRateOfChangeOfRhoPlaneContactRegularization();
      
      if (momentumOptimizationSettings.isContactRegularizationWeightObtainedFromContactState())
      {
         //TODO (Sylvain): get rid of that class (used for CarIngressEgressController)
         this.wrenchMatrixCalculator = new CylinderAndPlaneContactMatrixCalculatorAdapterwRhowPhiObtainedFromContactStates(centerOfMassFrame, registry,
               dynamicGraphicObjectsListRegistry, rhoSize, phiSize, wRhoSmoother);
      }
      else
      {
         this.wrenchMatrixCalculator = new CylinderAndPlaneContactMatrixCalculatorAdapter(centerOfMassFrame, registry, dynamicGraphicObjectsListRegistry, rhoSize,
               phiSize, wRhoCylinderContacts, wPhiCylinderContacts, wRhoPlaneContacts, wRhoSmoother);
      }

      int nDoF = ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor);

      //TODO Make EnumYoVariable so it can be change online
      MomentumOptimizer momentumOptimizerToUse = MomentumOptimizer.NO_GRF_SMOOTHER;
      momentumOptimizer = new CVXGenMomentumOptimizerBridge(nDoF, momentumOptimizerToUse, momentumOptimizationSettings);
      this.momentumOptimizationSettings = momentumOptimizationSettings;
      
      dampedLeastSquaresFactorMatrix = new DenseMatrix64F(nDoF, nDoF);

      this.jointsToOptimizeFor = jointsToOptimizeFor;

      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);

      this.hardMotionConstraintSolver = new DampedLeastSquaresSolver(1);
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
      if (this.momentumControlModuleSolverListener != null) throw new RuntimeException("MomentumControlModuleSolverListener is already set!");
      this.momentumControlModuleSolverListener = momentumControlModuleSolverListener;
   }

   public MomentumModuleSolution compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates,
                       Map<ContactableCylinderBody, ? extends CylindricalContactState> cylinderContactStates, RobotSide upcomingSupportLeg) throws MomentumControlModuleException

   {
      LinkedHashMap<RigidBody, Set<PlaneContactState>> planeContactStates = convertContactStates(contactStates);
      LinkedHashMap<RigidBody, CylindricalContactState> cylinderContactStatesConverted;
      if (cylinderContactStates != null)
      {
         cylinderContactStatesConverted = convertCylindricalContactStates(cylinderContactStates);
      }
      else
      {
         cylinderContactStatesConverted = null;
      }

      wrenchMatrixCalculator.setRhoMinScalar(momentumOptimizationSettings.getRhoMinScalar());

      hardMotionConstraintSolver.setAlpha(momentumOptimizationSettings.getDampedLeastSquaresFactor());
      momentumOptimizer.reset();

//      if (EqualityConstraintEnforcer.TEST_CONSTRAINT_CONSISTENCY)
//         hardMotionConstraintSolver.setAlpha(0.0);

      primaryMotionConstraintHandler.compute();

      centroidalMomentumHandler.compute();

      DenseMatrix64F jPrimary = primaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pPrimary = primaryMotionConstraintHandler.getRightHandSide();

      DenseMatrix64F a = centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      DenseMatrix64F b = centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData); 
      bOriginal.set(b);

      equalityConstraintEnforcer.setConstraint(jPrimary, pPrimary);
      equalityConstraintEnforcer.constrainEquation(a, b);

      wrenchMatrixCalculator.computeMatrices(planeContactStates, cylinderContactStatesConverted);

      DenseMatrix64F centroidalMomentumConvectiveTerm = centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm();
      DenseMatrix64F wrenchEquationRightHandSide = externalWrenchHandler.computeWrenchEquationRightHandSide(centroidalMomentumConvectiveTerm, bOriginal, b);

      /*
       * TODO:
       * IMPORTANT: the implementation below doesn't work properly, because lambda is supposed to act on the actual joint accelerations, not on the vector that is left
       * after the hard constraints have been applied. We really need a solver that can handle hard constraints in addition to soft constraints without being
       * too computationally expensive!
       */
      momentumOptimizationSettings.packDampedLeastSquaresFactorMatrix(dampedLeastSquaresFactorMatrix);
      
      secondaryMotionConstraintHandler.compute();
      DenseMatrix64F jSecondary = secondaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pSecondary = secondaryMotionConstraintHandler.getRightHandSide();
      DenseMatrix64F weightMatrixSecondary = secondaryMotionConstraintHandler.getWeightMatrix();

      equalityConstraintEnforcer.constrainEquation(jSecondary, pSecondary);

      DenseMatrix64F momentumDotWeight = momentumOptimizationSettings.getMomentumDotWeight(momentumRateOfChangeData.getMomentumSubspace());
      
      momentumOptimizer.setInputs(a, b, wrenchMatrixCalculator, wrenchEquationRightHandSide, momentumDotWeight, dampedLeastSquaresFactorMatrix, jSecondary, pSecondary, weightMatrixSecondary);
      
      NoConvergenceException noConvergenceException = null;
      try
      {
         optimize();
      }
      catch (NoConvergenceException e)
      {
         noConvergenceException = e;
      }

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenches(momentumOptimizer.getOutputRho(), momentumOptimizer.getOutputPhi());

      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);


      DenseMatrix64F jointAccelerations = equalityConstraintEnforcer.constrainResult(momentumOptimizer.getOutputJointAccelerations());
      
      updateMomentumControlModuleSolverListener(jPrimary, pPrimary, a, b, jSecondary, pSecondary, weightMatrixSecondary, jointAccelerations);
      
      ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);

      SpatialForceVector centroidalMomentumRateSolution = centroidalMomentumHandler.getCentroidalMomentumRate();
      Map<RigidBody, Wrench> externalWrenchSolution = externalWrenchHandler.getExternalWrenches();
      MomentumModuleSolution momentumModuleSolution = new MomentumModuleSolution(jointsToOptimizeFor, jointAccelerations, centroidalMomentumRateSolution, externalWrenchSolution);
      
      if (noConvergenceException != null)
      {
         throw new MomentumControlModuleException(noConvergenceException, momentumModuleSolution);
      }
       
      return momentumModuleSolution;
   }

   // TODO Conversion nastiness!! To be refactored
   private static LinkedHashMap<RigidBody, Set<PlaneContactState>> convertContactStates(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates)
   {
      LinkedHashMap<RigidBody, Set<PlaneContactState>> ret = new LinkedHashMap<RigidBody, Set<PlaneContactState>>();
      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         Set<PlaneContactState> set = ret.get(rigidBody);
         if (set == null)
         {
            set = new LinkedHashSet<PlaneContactState>();
            ret.put(rigidBody, set);
         }
         set.add(contactStates.get(contactablePlaneBody));
      }

      return ret;
   }

   // TODO Conversion nastiness!! To be refactored
   private static LinkedHashMap<RigidBody, CylindricalContactState> convertCylindricalContactStates(
         Map<ContactableCylinderBody, ? extends CylindricalContactState> contactStates)
   {
      LinkedHashMap<RigidBody, CylindricalContactState> ret = new LinkedHashMap<RigidBody, CylindricalContactState>();
      for (ContactableCylinderBody contactableCylinderBody : contactStates.keySet())
      {
         ret.put(contactableCylinderBody.getRigidBody(), contactStates.get(contactableCylinderBody));
      }
      return ret;
   }

   private void optimize() throws NoConvergenceException
   {
      try
      {
         momentumOptimizer.solve();
         converged.set(true);
      }
      catch (NoConvergenceException e)
      {
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
         
//         equalityConstraintEnforcer.computeCheck();
//         DenseMatrix64F checkCopy = equalityConstraintEnforcer.getCheckCopy();
//         momentumControlModuleSolverListener.setPrimaryMotionConstraintCheck(checkCopy);
         
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
         primaryMotionConstraintHandler.setDesiredPointAcceleration(rootToEndEffectorJacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, selectionMatrix, Double.POSITIVE_INFINITY);
      }
      else
      {
         primaryMotionConstraintHandler.setDesiredPointAcceleration(rootToEndEffectorJacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, Double.POSITIVE_INFINITY);
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
   

//   public SpatialForceVector getDesiredCentroidalMomentumRate()
//   {
//      return centroidalMomentumHandler.getCentroidalMomentumRate();
//   }
//
//   public Map<RigidBody, Wrench> getExternalWrenches()
//   {
//      return externalWrenchHandler.getExternalWrenches();
//   }
   
}
