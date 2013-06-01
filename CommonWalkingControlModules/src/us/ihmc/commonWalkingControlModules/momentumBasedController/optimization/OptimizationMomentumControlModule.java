package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXWithCylinderNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXWithCylinderNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.CVXWithCylinderNativeOutput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactMatrixCalculatorAdapter;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

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

   private final HardMotionConstraintEnforcer hardMotionConstraintEnforcer;
   private final MotionConstraintHandler primaryMotionConstraintHandler;
   private final MotionConstraintHandler secondaryMotionConstraintHandler;

// private final ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator;

   private final CylinderAndPlaneContactMatrixCalculatorAdapter wrenchMatrixCalculator;

   private final CVXWithCylinderNativeInput momentumOptimizerNativeInput = new CVXWithCylinderNativeInput();
   private final CVXWithCylinderNative momentumOptimizerNative;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final BooleanYoVariable converged = new BooleanYoVariable("converged", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final MomentumRateOfChangeData momentumRateOfChangeData;
   private final DampedLeastSquaresSolver hardMotionConstraintSolver;

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT,
           InverseDynamicsJoint[] jointsToOptimizeFor, MomentumOptimizationSettings momentumOptimizationSettings, double gravityZ,
           TwistCalculator twistCalculator, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, controlDT, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor, twistCalculator);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor, twistCalculator);

      int rhoSize = CVXWithCylinderNative.rhoSize;
      int phiSize = CVXWithCylinderNative.phiSize;
      dynamicGraphicObjectsListRegistry = null; // don't visualize vectors
      this.wrenchMatrixCalculator = new CylinderAndPlaneContactMatrixCalculatorAdapter(centerOfMassFrame, registry, dynamicGraphicObjectsListRegistry, rhoSize,
              phiSize);

      this.momentumOptimizerNative = new CVXWithCylinderNative(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor), rhoSize, phiSize);
      this.momentumOptimizationSettings = momentumOptimizationSettings;

      this.jointsToOptimizeFor = jointsToOptimizeFor;

      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);

      this.hardMotionConstraintSolver = new DampedLeastSquaresSolver(1);
      this.hardMotionConstraintEnforcer = new HardMotionConstraintEnforcer(hardMotionConstraintSolver);

      parentRegistry.addChild(registry);
      reset();
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

   public void compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates,
                       Map<ContactableCylinderBody, ? extends CylindricalContactState> cylinderContactStates, RobotSide upcomingSupportLeg)
   {
      LinkedHashMap<RigidBody, PlaneContactState> planeContactStates = convertContactStates(contactStates);
      LinkedHashMap<RigidBody, CylindricalContactState> cylinderContactStatesConverted;
      if (cylinderContactStates != null)
      {
         cylinderContactStatesConverted = convertCylindricalContactStates(cylinderContactStates);
      }
      else
      {
         cylinderContactStatesConverted = null;
      }

      hardMotionConstraintSolver.setAlpha(momentumOptimizationSettings.getDampedLeastSquaresFactor());

      if (HardMotionConstraintEnforcer.TEST_CONSTRAINT_CONSISTENCY)
         hardMotionConstraintSolver.setAlpha(0.0);

      primaryMotionConstraintHandler.compute();

      centroidalMomentumHandler.compute();

      DenseMatrix64F jPrimary = primaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pPrimary = primaryMotionConstraintHandler.getRightHandSide();

      DenseMatrix64F a = centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      DenseMatrix64F b = centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData);

      DenseMatrix64F bOriginal = new DenseMatrix64F(b);

      hardMotionConstraintEnforcer.set(jPrimary, pPrimary);
      hardMotionConstraintEnforcer.constrainEquation(a, b);

      momentumOptimizerNativeInput.setCentroidalMomentumMatrix(a);
      momentumOptimizerNativeInput.setMomentumDotEquationRightHandSide(b);

      wrenchMatrixCalculator.computeMatrices(planeContactStates, cylinderContactStatesConverted);

      momentumOptimizerNativeInput.setContactPointWrenchMatrix(wrenchMatrixCalculator.getQRho());
      momentumOptimizerNativeInput.setContactPointWrenchMatrixForBoundedCylinderVariables(wrenchMatrixCalculator.getQPhi());
      momentumOptimizerNativeInput.setPhiMin(wrenchMatrixCalculator.getPhiMin());
      momentumOptimizerNativeInput.setPhiMax(wrenchMatrixCalculator.getPhiMax());
      momentumOptimizerNativeInput.setRhoMin(wrenchMatrixCalculator.getRhoMin());

      DenseMatrix64F wrenchEquationRightHandSide =
         externalWrenchHandler.computeWrenchEquationRightHandSide(centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm(), bOriginal, b);
      momentumOptimizerNativeInput.setWrenchEquationRightHandSide(wrenchEquationRightHandSide);

      momentumOptimizerNativeInput.setMomentumDotWeight(momentumOptimizationSettings.getMomentumDotWeight(momentumRateOfChangeData.getMomentumSubspace()));
      momentumOptimizerNativeInput.setJointAccelerationRegularization(
          momentumOptimizationSettings.getDampedLeastSquaresFactorMatrix(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor)));

      secondaryMotionConstraintHandler.compute();
      DenseMatrix64F jSecondary = secondaryMotionConstraintHandler.getJacobian();
      DenseMatrix64F pSecondary = secondaryMotionConstraintHandler.getRightHandSide();

      hardMotionConstraintEnforcer.constrainEquation(jSecondary, pSecondary);

      momentumOptimizerNativeInput.setSecondaryConstraintJacobian(jSecondary);
      momentumOptimizerNativeInput.setSecondaryConstraintRightHandSide(pSecondary);
      momentumOptimizerNativeInput.setSecondaryConstraintWeight(secondaryMotionConstraintHandler.getWeightMatrix());

      momentumOptimizerNativeInput.setGroundReactionForceRegularization(momentumOptimizationSettings.getGroundReactionForceRegularization());
      momentumOptimizerNativeInput.setPhiRegularization(momentumOptimizationSettings.getPhiRegularization());

      optimize(momentumOptimizerNativeInput);

      CVXWithCylinderNativeOutput output = momentumOptimizerNative.getOutput();

      Map<RigidBody, Wrench> groundReactionWrenches = wrenchMatrixCalculator.computeWrenches(output.getRho(), output.getPhi());

      externalWrenchHandler.computeExternalWrenches(groundReactionWrenches);


      DenseMatrix64F jointAccelerations = hardMotionConstraintEnforcer.constrainResult(output.getJointAccelerations());
      ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);
   }

   private static LinkedHashMap<RigidBody, PlaneContactState> convertContactStates(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates)
   {
      LinkedHashMap<RigidBody, PlaneContactState> ret = new LinkedHashMap<RigidBody, PlaneContactState>();
      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         ret.put(contactablePlaneBody.getRigidBody(), contactStates.get(contactablePlaneBody));
      }

      return ret;
   }

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

   private void optimize(CVXWithCylinderNativeInput momentumOptimizerNativeInput)
   {
      try
      {
         momentumOptimizerNative.solve(momentumOptimizerNativeInput);
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
      }
   }

   public void resetGroundReactionWrenchFilter()
   {
      // empty for now
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      primaryMotionConstraintHandler.setDesiredJointAcceleration(joint, jointAcceleration, Double.POSITIVE_INFINITY);    // weight is arbitrary, actually
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      primaryMotionConstraintHandler.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData, Double.POSITIVE_INFINITY);    // weight is arbitrary,

      // actually
   }

   public void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      primaryMotionConstraintHandler.setDesiredPointAcceleration(jacobian, bodyFixedPoint, desiredAccelerationWithRespectToBase, Double.POSITIVE_INFINITY);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData.set(momentumRateOfChangeData);
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchHandler.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight)
   {
      secondaryMotionConstraintHandler.setDesiredJointAcceleration(joint, jointAcceleration, weight);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
   {
      secondaryMotionConstraintHandler.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData, weight);
   }

   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return centroidalMomentumHandler.getCentroidalMomentumRate();
   }

   public Map<RigidBody, Wrench> getExternalWrenches()
   {
      return externalWrenchHandler.getExternalWrenches();
   }
}
