package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter;
import com.yobotics.simulationconstructionset.util.errorHandling.WalkingStatusReporter.ErrorType;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BlindWalkingToDestinationDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ComponentBasedDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointGroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributor;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.streamingData.GlobalDataProducer;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class HighLevelHumanoidControllerFactoryHelper
{
   public static BlindWalkingToDestinationDesiredFootstepCalculator getBlindWalkingToDestinationDesiredFootstepCalculator(
           WalkingControllerParameters walkingControllerParameters, CommonWalkingReferenceFrames referenceFrames,
           SideDependentList<ContactablePlaneBody> bipedFeet, YoVariableRegistry registry)
   {
      BlindWalkingToDestinationDesiredFootstepCalculator desiredFootstepCalculator =
         new BlindWalkingToDestinationDesiredFootstepCalculator(referenceFrames.getAnkleZUpReferenceFrames(), referenceFrames.getFootReferenceFrames(),
            bipedFeet, registry);

      desiredFootstepCalculator.setDesiredStepWidth(walkingControllerParameters.getInPlaceWidth());
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
         boolean cycleThroughAllEvents = true;
         HeadingAndVelocityEvaluationScript headingAndVelocityEvaluationScript = new HeadingAndVelocityEvaluationScript(cycleThroughAllEvents, controlDT,
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

   public static WalkingStatusReporter getWalkingStatusReporter(GlobalDataProducer dataProducer, YoVariableRegistry registry)
   {
      WalkingStatusReporter walkingStatusReporter = new WalkingStatusReporter(dataProducer);

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
         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            List<InverseDynamicsJoint> fingerJoints = Arrays.asList(ScrewTools.computeSubtreeJoints(hand));
            joints.removeAll(fingerJoints);
         }
      }

      if (lidarJoint != null)
      {
         joints.remove(lidarJoint);
      }

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }

   public static OldMomentumControlModule createOldMomentumControlModule(WalkingControllerParameters walkingControllerParameters,
           FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, double gravityZ, TwistCalculator twistCalculator, double controlDT,
           LidarControllerInterface lidarControllerInterface, YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      GroundReactionWrenchDistributor groundReactionWrenchDistributor = null;
      ContactPointGroundReactionWrenchDistributor contactPointGroundReactionWrenchDistributor =
         new ContactPointGroundReactionWrenchDistributor(referenceFrames.getCenterOfMassFrame(), registry);
      double[] diagonalCWeights = new double[]
      {
         1.0, 1.0, 1.0, 10.0, 10.0, 10.0
      };
      contactPointGroundReactionWrenchDistributor.setWeights(diagonalCWeights, 5.0, 0.001);
      groundReactionWrenchDistributor = contactPointGroundReactionWrenchDistributor;

      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      OldMomentumControlModule oldMomentumControlModule = new OldMomentumControlModule(fullRobotModel.getRootJoint(), gravityZ,
                                                             groundReactionWrenchDistributor, referenceFrames.getCenterOfMassFrame(), controlDT,
                                                             twistCalculator, jacobianSolver, registry, dynamicGraphicObjectsListRegistry);
      oldMomentumControlModule.setGroundReactionWrenchBreakFrequencyHertz(walkingControllerParameters.getGroundReactionWrenchBreakFrequencyHertz());

      return oldMomentumControlModule;
   }

   public static MomentumOptimizationSettings createMomentumOptimizationSettings(FullRobotModel fullRobotModel,
           LidarControllerInterface lidarControllerInterface, YoVariableRegistry registry)
   {
      OneDoFJoint lidarJoint = null;
      if (lidarControllerInterface != null)
         lidarJoint = lidarControllerInterface.getLidarJoint();

      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, registry);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(0.05);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(0.001);
      momentumOptimizationSettings.setMomentumWeight(1.0, 1.0, 10.0, 10.0);
      momentumOptimizationSettings.setRhoMin(4.0);
      momentumOptimizationSettings.setRateOfChangeOfRhoPlaneContactRegularization(0.01);
      momentumOptimizationSettings.setRhoPenalizerPlaneContactRegularization(0.01);

      return momentumOptimizationSettings;
   }

}
