package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.CylinderAndContactPointWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BlindWalkingToDestinationDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingUpdater;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScript;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.ManualDesiredVelocityControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.RateBasedDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.SimpleDesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointGroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter.ErrorType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class HighLevelHumanoidControllerFactoryHelper
{
   private static final boolean USE_NEW_OPTIMIZATION_MOMENTUM_CONTROL_MODULE = false;
   private static final boolean USE_PLANE_ONLY_FORCE_DISTRIBUTOR = false;

   public static BlindWalkingToDestinationDesiredFootstepCalculator getBlindWalkingToDestinationDesiredFootstepCalculator(
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<ContactablePlaneBody> bipedFeet, YoVariableRegistry registry)
   {
      BlindWalkingToDestinationDesiredFootstepCalculator desiredFootstepCalculator =
         new BlindWalkingToDestinationDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getFootReferenceFrames(),
            bipedFeet, registry);

      desiredFootstepCalculator.setInPlaceWidth(walkingControllerParameters.getInPlaceWidth());
      desiredFootstepCalculator.setDesiredStepForward(walkingControllerParameters.getDesiredStepForward());
      desiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getMaxStepLength());
      desiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getMinStepWidth());
      desiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getMaxStepWidth());
      desiredFootstepCalculator.setStepPitch(walkingControllerParameters.getStepPitch());

      return desiredFootstepCalculator;
   }


   public static ComponentBasedDesiredFootstepCalculator getDesiredFootstepCalculator(WalkingControllerParameters walkingControllerParameters,
           CommonWalkingReferenceFrames referenceFrames, SideDependentList<ContactablePlaneBody> bipedFeet, double controlDT, YoVariableRegistry registry,
           ArrayList<Updatable> updatables, boolean useHeadingAndVelocityScript)
   {
      ComponentBasedDesiredFootstepCalculator desiredFootstepCalculator;
      ManualDesiredVelocityControlModule desiredVelocityControlModule;

      DesiredHeadingControlModule desiredHeadingControlModule;
      if (useHeadingAndVelocityScript)
      {
         desiredVelocityControlModule = new ManualDesiredVelocityControlModule(ReferenceFrame.getWorldFrame(), registry);
         desiredVelocityControlModule.setDesiredVelocity(new FrameVector2d(ReferenceFrame.getWorldFrame(), 1.0, 0.0));

         SimpleDesiredHeadingControlModule simpleDesiredHeadingControlModule = new SimpleDesiredHeadingControlModule(0.0, controlDT, registry);
         simpleDesiredHeadingControlModule.setMaxHeadingDot(0.4);
         simpleDesiredHeadingControlModule.updateDesiredHeadingFrame();
         HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(true, controlDT,
                                                                                    simpleDesiredHeadingControlModule, desiredVelocityControlModule, registry);
         updatables.add(headingAndVelocityEvaluationScript);
         desiredHeadingControlModule = simpleDesiredHeadingControlModule;
      }
      else
      {
         desiredHeadingControlModule = new RateBasedDesiredHeadingControlModule(0.0, controlDT, registry);
         desiredVelocityControlModule = new ManualDesiredVelocityControlModule(desiredHeadingControlModule.getDesiredHeadingFrame(), registry);
      }

      updatables.add(new DesiredHeadingUpdater(desiredHeadingControlModule));

      desiredFootstepCalculator = new ComponentBasedDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(),
              referenceFrames.getFootReferenceFrames(), bipedFeet, desiredHeadingControlModule, desiredVelocityControlModule, registry);

      desiredFootstepCalculator.setInPlaceWidth(walkingControllerParameters.getInPlaceWidth());
      desiredFootstepCalculator.setMaxStepLength(walkingControllerParameters.getMaxStepLength());
      desiredFootstepCalculator.setMinStepWidth(walkingControllerParameters.getMinStepWidth());
      desiredFootstepCalculator.setMaxStepWidth(walkingControllerParameters.getMaxStepWidth());
      desiredFootstepCalculator.setStepPitch(walkingControllerParameters.getStepPitch());

      return desiredFootstepCalculator;
   }

   public static WalkingStatusReporter getWalkingStatusReporter(ObjectCommunicator objectCommunicator, YoVariableRegistry registry)
   {
      WalkingStatusReporter walkingStatusReporter = new WalkingStatusReporter(objectCommunicator);

      DoubleYoVariable icpErrorX = (DoubleYoVariable) registry.getVariable("icpErrorX");
      DoubleYoVariable icpErrorY = (DoubleYoVariable) registry.getVariable("icpErrorY");
      Pair<Double, Double> icpXYBounds = new Pair<Double, Double>(-0.06, 0.06);
      walkingStatusReporter.setErrorAndBounds(ErrorType.ICP_X, icpErrorX, icpXYBounds);
      walkingStatusReporter.setErrorAndBounds(ErrorType.ICP_Y, icpErrorY, icpXYBounds);

      DoubleYoVariable pelvisOrientationError = (DoubleYoVariable) registry.getVariable("pelvisOrientationError");
      Pair<Double, Double> pelvisOrientationBounds = new Pair<Double, Double>(-0.2, 0.2);
      walkingStatusReporter.setErrorAndBounds(ErrorType.PELVIS_ORIENTATION, pelvisOrientationError, pelvisOrientationBounds);

      return walkingStatusReporter;
   }
   
   
   public static InverseDynamicsJoint[] computeJointsToOptimizeFor(FullRobotModel fullRobotModel, InverseDynamicsJoint lidarJoint)
   {
      List<InverseDynamicsJoint> joints = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      joints.addAll(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         List<InverseDynamicsJoint> fingerJoints = Arrays.asList(ScrewTools.computeSubtreeJoints(fullRobotModel.getHand(robotSide)));
         joints.removeAll(fingerJoints);
      }

      joints.remove(lidarJoint);

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }
   
   public static MomentumControlModule createMomentumControlModule(WalkingControllerParameters walkingControllerParameters, FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, double gravityZ,
         TwistCalculator twistCalculator, double controlDT, LidarControllerInterface lidarControllerInterface, YoVariableRegistry registry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      MomentumControlModule momentumControlModule;
      if (USE_NEW_OPTIMIZATION_MOMENTUM_CONTROL_MODULE)
      {
         InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarControllerInterface.getLidarJoint());
         MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(registry);
         momentumOptimizationSettings.setDampedLeastSquaresFactor(5e-2);
         momentumOptimizationSettings.setGroundReactionForceRegularization(0.001);
         momentumOptimizationSettings.setPhiRegularization(0.001);
         momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 10.0, 10.0);
         momentumOptimizationSettings.setRhoMin(0.0);
         momentumControlModule = new OptimizationMomentumControlModule(fullRobotModel.getRootJoint(), referenceFrames.getCenterOfMassFrame(), controlDT,
                 jointsToOptimizeFor, momentumOptimizationSettings, gravityZ, twistCalculator, dynamicGraphicObjectsListRegistry, registry);
      }
      else
      {
         GroundReactionWrenchDistributor groundReactionWrenchDistributor = null;
         if (USE_PLANE_ONLY_FORCE_DISTRIBUTOR)
         {
            ContactPointGroundReactionWrenchDistributor contactPointGroundReactionWrenchDistributor =
                  new ContactPointGroundReactionWrenchDistributor(referenceFrames.getCenterOfMassFrame(), registry);
            double[] diagonalCWeights = new double[]
                  {
                        1.0, 1.0, 1.0, 10.0, 10.0, 10.0
                  };
            contactPointGroundReactionWrenchDistributor.setWeights(diagonalCWeights, 5.0, 0.001);
            groundReactionWrenchDistributor = contactPointGroundReactionWrenchDistributor;
         }
         else
         {
            CylinderAndContactPointWrenchDistributor optimizationBasedWrenchDistributor =
                  new CylinderAndContactPointWrenchDistributor(referenceFrames.getCenterOfMassFrame(), registry, dynamicGraphicObjectsListRegistry);
            groundReactionWrenchDistributor = optimizationBasedWrenchDistributor;
         }


         DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
         jacobianSolver.setAlpha(5e-2);

         OldMomentumControlModule oldMomentumControlModule = new OldMomentumControlModule(fullRobotModel.getRootJoint(), gravityZ,
                                                                groundReactionWrenchDistributor, referenceFrames.getCenterOfMassFrame(), controlDT,
                                                                twistCalculator, jacobianSolver, registry, dynamicGraphicObjectsListRegistry);
         oldMomentumControlModule.setGroundReactionWrenchBreakFrequencyHertz(walkingControllerParameters.getGroundReactionWrenchBreakFrequencyHertz());

         momentumControlModule = oldMomentumControlModule;
      }
      return momentumControlModule;
   }
  
}
