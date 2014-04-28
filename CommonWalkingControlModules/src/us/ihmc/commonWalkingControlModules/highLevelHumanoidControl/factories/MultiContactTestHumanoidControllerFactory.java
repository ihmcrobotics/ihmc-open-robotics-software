package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MultiContactTestHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredArmJointAngleProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredComHeightProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ReinitializeWalkingControllerProvider;
import us.ihmc.commonWalkingControlModules.packetProviders.ControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.SystemErrControlStatusProducer;
import us.ihmc.commonWalkingControlModules.packets.DesiredHighLevelStateProvider;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FingerForceSensors;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointGroundReactionWrenchDistributor;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.TorusManipulationProvider;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.TorusPoseProvider;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;

public class MultiContactTestHumanoidControllerFactory implements HighLevelHumanoidControllerFactory
{
   private final SideDependentList<String> namesOfJointsBeforeHands;
   private final SideDependentList<Transform3D> handContactPointTransforms;
   private final SideDependentList<List<Point2d>> handContactPoints;
   private final RobotSide[] footContactSides;
   private final RobotSide[] handContactSides;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;

   public MultiContactTestHumanoidControllerFactory(SideDependentList<String> namesOfJointsBeforeHands,
           SideDependentList<Transform3D> handContactPointTransforms, SideDependentList<List<Point2d>> handContactPoints, RobotSide[] footContactSides,
           RobotSide[] handContactSides, WalkingControllerParameters walkingControllerParameters, ArmControllerParameters armControllerParameters)
   {
      this.namesOfJointsBeforeHands = namesOfJointsBeforeHands;
      this.handContactPointTransforms = handContactPointTransforms;
      this.handContactPoints = handContactPoints;
      this.footContactSides = footContactSides;
      this.handContactSides = handContactSides;
      this.walkingControllerParameters = walkingControllerParameters;
      this.armControllerParameters = armControllerParameters;
   }

   public RobotController create(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
                                 Map<OneDoFJoint, Double> initialPositionControlKpGains, Map<OneDoFJoint, Double> initialPositionControlKdGains,
                                 CommonWalkingReferenceFrames referenceFrames, FingerForceSensors fingerForceSensors, DoubleYoVariable yoTime, double gravityZ,
                                 TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, SideDependentList<ContactablePlaneBody> feet,
                                 double controlDT, SideDependentList<FootSwitchInterface> footSwitches,
                                 LidarControllerInterface lidarControllerInterface, StateEstimationDataFromController stateEstimationDataFromControllerSink,
                                 DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                 YoVariableRegistry registry, GUISetterUpperRegistry guiSetterUpperRegistry,
                                 ProcessedOutputsInterface processedOutputs, ForceSensorDataHolder forceSensorDataHolder)
   {
      LinkedHashMap<ContactablePlaneBody, RigidBody> contactablePlaneBodiesAndBases = new LinkedHashMap<ContactablePlaneBody, RigidBody>();

      // TODO: code duplication from driving controller
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      SideDependentList<ContactablePlaneBody> hands = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         InverseDynamicsJoint[] jointBeforeHandArray = ScrewTools.findJointsWithNames(allJoints, namesOfJointsBeforeHands.get(robotSide));
         if (jointBeforeHandArray.length != 1)
            throw new RuntimeException("Incorrect number of joints before hand found: " + jointBeforeHandArray.length);

         RigidBody handBody = jointBeforeHandArray[0].getSuccessor();

         ReferenceFrame afterHipFrame = handBody.getParentJoint().getFrameAfterJoint();
         ReferenceFrame handContactsFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression()
                                               + "HandContact", afterHipFrame, handContactPointTransforms.get(robotSide));

         ContactablePlaneBody hand = new ListOfPointsContactablePlaneBody(handBody, handContactsFrame, handContactPoints.get(robotSide));
         hands.put(robotSide, hand);
      }

      for (ContactablePlaneBody contactablePlaneBody : feet.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getPelvis());
      }

      for (ContactablePlaneBody contactablePlaneBody : hands.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getChest());
      }

      ContactPointGroundReactionWrenchDistributor groundReactionWrenchDistributor =
         new ContactPointGroundReactionWrenchDistributor(referenceFrames.getCenterOfMassFrame(), registry);
      double[] diagonalCWeights = new double[]
      {
         1.0, 1.0, 1.0, 1.0, 1.0, 1.0
      };
      groundReactionWrenchDistributor.setWeights(diagonalCWeights, 1.0, 0.001);

      CoMBasedMomentumRateOfChangeControlModule momentumRateOfChangeControlModule =
         new CoMBasedMomentumRateOfChangeControlModule(controlDT, referenceFrames.getCenterOfMassFrame(), centerOfMassJacobian, registry);

      double comProportionalGain = 100.0;
      double dampingRatio = 1.0;
      double totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      double comDerivativeGain = GainCalculator.computeDampingForSecondOrderSystem(totalMass, comProportionalGain, dampingRatio);
      momentumRateOfChangeControlModule.setProportionalGains(comProportionalGain, comProportionalGain, comProportionalGain);
      momentumRateOfChangeControlModule.setDerivativeGains(comDerivativeGain, comDerivativeGain, comDerivativeGain);

      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

//      OldMomentumControlModule oldMomentumControlModule = new OldMomentumControlModule(fullRobotModel.getRootJoint(), gravityZ, groundReactionWrenchDistributor,
//                                                          referenceFrames.getCenterOfMassFrame(), controlDT, twistCalculator, jacobianSolver, registry,
//                                                          dynamicGraphicObjectsListRegistry);
//      double groundReactionWrenchBreakFrequencyHertz = 7.0;
//      oldMomentumControlModule.setGroundReactionWrenchBreakFrequencyHertz(groundReactionWrenchBreakFrequencyHertz);
      OldMomentumControlModule oldMomentumControlModule = null;

      MomentumOptimizationSettings momentumOptimizationSettings = HighLevelHumanoidControllerFactoryHelper.createMomentumOptimizationSettings(fullRobotModel, lidarControllerInterface, registry);
      
      // The controllers do not extend the MomentumBasedController anymore. Instead, it is passed through the constructor.
      MomentumBasedController momentumBasedController = new MomentumBasedController(estimationLink, estimationFrame, fullRobotModel, centerOfMassJacobian,
                                                           referenceFrames, footSwitches, yoTime, gravityZ, twistCalculator, feet, hands, null, null, null, controlDT,
                                                           processedOutputs, momentumOptimizationSettings , oldMomentumControlModule, null, stateEstimationDataFromControllerSink,
                                                           dynamicGraphicObjectsListRegistry);

      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters, registry);
      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = null;
      FootstepProvider footstepProvider = null;
      HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters = null;
      DesiredHeadOrientationProvider headOrientationProvider = null;
      DesiredComHeightProvider comHeightProvider = null;
      DesiredPelvisPoseProvider pelvisPoseProvider = null;
      DesiredChestOrientationProvider chestOrientationProvider = null;
      DesiredFootStateProvider footLoadBearingProvider = null;
      DesiredThighLoadBearingProvider thighLoadBearingProvider = null;
      DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider = null;
      DesiredHighLevelStateProvider highLevelStateProvider = null;
      ReinitializeWalkingControllerProvider reinitializeWalkingController = null;
      
      DesiredArmJointAngleProvider armJointAngleProvider = null;
      
      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();
     
      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, chestOrientationProvider, footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider,
            pelvisLoadBearingProvider, armJointAngleProvider, reinitializeWalkingController, controlStatusProducer);

      VariousWalkingManagers variousWalkingManagers = VariousWalkingManagers.create(momentumBasedController, yoTime, variousWalkingProviders, walkingControllerParameters,
            armControllerParameters, registry, dynamicGraphicObjectsListRegistry);
      
      MultiContactTestHumanoidController multiContactBehavior = new MultiContactTestHumanoidController(variousWalkingProviders, variousWalkingManagers,
                                                                   momentumRateOfChangeControlModule,
                                                                   momentumBasedController, walkingControllerParameters, null,
                                                                   dynamicGraphicObjectsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         multiContactBehavior.setFootInContact(robotSide, false);
         multiContactBehavior.setHandInContact(robotSide, false);
      }

      for (RobotSide robotSide : footContactSides)
      {
         multiContactBehavior.setFootInContact(robotSide, true);
      }

      for (RobotSide robotSide : handContactSides)
      {
         multiContactBehavior.setHandInContact(robotSide, true);
      }

      ArrayList<Pair<State<HighLevelState>, YoVariableRegistry>> highLevelBehaviors = new ArrayList<>();
      highLevelBehaviors.add(new Pair<State<HighLevelState>, YoVariableRegistry>(multiContactBehavior, multiContactBehavior.getYoVariableRegistry()));

      // This is the "highest level" controller that enables switching between the different controllers (walking, multi-contact, driving, etc.)
      HighLevelHumanoidControllerManager ret = new HighLevelHumanoidControllerManager(HighLevelState.MULTI_CONTACT, highLevelBehaviors, momentumBasedController, null);

      return ret;
   }

}
