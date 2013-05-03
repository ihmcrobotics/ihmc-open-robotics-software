package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNativeOutput;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
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
   private final HardMotionConstraintEnforcer hardMotionConstraintEnforcer = new HardMotionConstraintEnforcer();
   private final MotionConstraintHandler primaryMotionConstraintHandler;
   private final MotionConstraintHandler secondaryMotionConstraintHandler;
   private final ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator;
   private final MomentumOptimizerNativeInput momentumOptimizerNativeInput = new MomentumOptimizerNativeInput();
   private final MomentumOptimizerNative momentumOptimizerNative;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final BooleanYoVariable converged = new BooleanYoVariable("converged", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final Map<ContactablePlaneBody, Wrench> wrenches = new LinkedHashMap<ContactablePlaneBody, Wrench>();
   private final InverseDynamicsJoint rootJoint;    // TODO: make this not be special

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT,
           YoVariableRegistry parentRegistry, InverseDynamicsJoint[] jointsToOptimizeFor, MomentumOptimizationSettings momentumOptimizationSettings,
           double gravityZ)
   {
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, controlDT, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, null);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor);
      this.contactPointWrenchMatrixCalculator = new ContactPointWrenchMatrixCalculator(centerOfMassFrame, MomentumOptimizerNative.nSupportVectors,
              MomentumOptimizerNative.rhoSize);

      this.momentumOptimizerNative = new MomentumOptimizerNative();
      this.momentumOptimizationSettings = momentumOptimizationSettings;

      this.jointsToOptimizeFor = jointsToOptimizeFor;

      this.rootJoint = rootJoint;

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize()
   {
      centroidalMomentumHandler.initialize();
   }

   public void reset()
   {
      primaryMotionConstraintHandler.reset();
      secondaryMotionConstraintHandler.reset();
   }

   public void compute(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData momentumRateOfChangeData,
                       LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
   {
      DenseMatrix64F accelerationSubspace = rootJointAccelerationData.getAccelerationSubspace();
      DenseMatrix64F accelerationMultipliers = rootJointAccelerationData.getAccelerationMultipliers();
      DenseMatrix64F momentumSubspace = momentumRateOfChangeData.getMomentumSubspace();
      DenseMatrix64F momentumMultipliers = momentumRateOfChangeData.getMomentumMultipliers();

      // TODO: make this not be special
      if (accelerationSubspace.getNumCols() > 0)
      {
         TaskspaceConstraintData rootJointTaskspaceConstraintData = new TaskspaceConstraintData();
         DenseMatrix64F rootJointAccelerationMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
         CommonOps.mult(accelerationSubspace, accelerationMultipliers, rootJointAccelerationMatrix);
         SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(),
                                                            rootJoint.getFrameAfterJoint(), rootJointAccelerationMatrix);
         spatialAcceleration.changeBodyFrameNoRelativeAcceleration(rootJoint.getSuccessor().getBodyFixedFrame());
         spatialAcceleration.changeFrameNoRelativeMotion(rootJoint.getSuccessor().getBodyFixedFrame());
         DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 1);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(accelerationSubspace.getNumCols(), accelerationSubspace.getNumRows());
         CommonOps.transpose(accelerationSubspace, selectionMatrix);
         rootJointTaskspaceConstraintData.set(spatialAcceleration, nullspaceMultipliers, selectionMatrix);

//       setDesiredSpatialAcceleration(rootJoint.getMotionSubspace(), rootJointTaskspaceConstraintData);
         secondaryMotionConstraintHandler.setDesiredSpatialAcceleration(rootJoint.getMotionSubspace().getJointsInOrder(), rootJoint.getMotionSubspace(),
                 rootJointTaskspaceConstraintData);
      }

      // TODO: hard constraint enforcement, using momentum selection matrix


      centroidalMomentumHandler.compute();
      momentumOptimizerNativeInput.setCentroidalMomentumMatrix(centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor));    // FIXME: hard constraint enforcement
      momentumOptimizerNativeInput.setMomentumDotEquationRightHandSide(
          centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData));

      momentumOptimizerNativeInput.setRhoMin(contactPointWrenchMatrixCalculator.getRhoMin(contactStates.values(),
              momentumOptimizationSettings.getRhoMinScalar()));

      contactPointWrenchMatrixCalculator.computeMatrix(contactStates.values());
      momentumOptimizerNativeInput.setContactPointWrenchMatrix(contactPointWrenchMatrixCalculator.getMatrix());
      momentumOptimizerNativeInput.setWrenchEquationRightHandSide(
          externalWrenchHandler.computeWrenchEquationRightHandSide(centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm()));

//      momentumOptimizerNativeInput.setMomentumDotWeight(momentumOptimizationSettings.getMomentumDotWeight());
      momentumOptimizerNativeInput.setJointAccelerationRegularization(
          momentumOptimizationSettings.getDampedLeastSquaresFactorMatrix(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor)));

      // TODO: hard constraints?
      secondaryMotionConstraintHandler.compute();
      momentumOptimizerNativeInput.setSecondaryConstraintJacobian(secondaryMotionConstraintHandler.getJacobian());
      momentumOptimizerNativeInput.setSecondaryConstraintRightHandSide(secondaryMotionConstraintHandler.getRightHandSide());
      momentumOptimizerNativeInput.setNullspaceMatrix(secondaryMotionConstraintHandler.getNullspaceTransposeMatrix());    // TODO: check: transpose?
      momentumOptimizerNativeInput.setNullspaceMultipliers(secondaryMotionConstraintHandler.getNullspaceMultipliers());
//      momentumOptimizerNativeInput.setSecondaryConstraintWeight();


      optimize();

      MomentumOptimizerNativeOutput output = momentumOptimizerNative.getOutput();

      contactPointWrenchMatrixCalculator.computeWrenches(contactStates.values(), output.getRho());

      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         wrenches.put(contactablePlaneBody, contactPointWrenchMatrixCalculator.getWrench(contactStates.get(contactablePlaneBody)));
      }

      // TODO: hard constraint enforcement

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, output.getJointAccelerations());
   }

   private void optimize()
   {
      try
      {
         momentumOptimizerNative.solve(momentumOptimizerNativeInput);
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
      primaryMotionConstraintHandler.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      primaryMotionConstraintHandler.setDesiredSpatialAcceleration(jacobian.getJointsInOrder(), jacobian, taskspaceConstraintData);
   }

   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return centroidalMomentumHandler.getCentroidalMomentumRate();
   }

   public Map<ContactablePlaneBody, Wrench> getExternalWrenches()
   {
      return wrenches;
   }
}
