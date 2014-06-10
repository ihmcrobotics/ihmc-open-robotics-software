package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelBehavior;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MultiContactTestHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CoMBasedMomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OldMomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
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
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryParameters;

public class MultiContactTestHumanoidControllerFactory implements HighLevelHumanoidControllerFactory
{
   private final ContactableBodiesFactory contactableBodiesFactory;
   private final RobotSide[] footContactSides;
   private final RobotSide[] handContactSides;
   private final WalkingControllerParameters walkingControllerParameters;
   private final ArmControllerParameters armControllerParameters;

   public MultiContactTestHumanoidControllerFactory(ContactableBodiesFactory contactableBodiesFactory, RobotSide[] footContactSides,
           RobotSide[] handContactSides, WalkingControllerParameters walkingControllerParameters, ArmControllerParameters armControllerParameters)
   {
      this.contactableBodiesFactory = contactableBodiesFactory;
      this.footContactSides = footContactSides;
      this.handContactSides = handContactSides;
      this.walkingControllerParameters = walkingControllerParameters;
      this.armControllerParameters = armControllerParameters;
   }

   public RobotController create(FullRobotModel fullRobotModel, Map<OneDoFJoint, Double> initialPositionControlKpGains, Map<OneDoFJoint, Double> initialPositionControlKdGains,
                                 CommonWalkingReferenceFrames referenceFrames, FingerForceSensors fingerForceSensors, DoubleYoVariable yoTime, double gravityZ,
                                 TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian, SideDependentList<ContactablePlaneBody> feet,
                                 double controlDT, SideDependentList<FootSwitchInterface> footSwitches, LidarControllerInterface lidarControllerInterface,
                                 DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry, ForceSensorDataHolder forceSensorDataHolder)
   {
      LinkedHashMap<ContactablePlaneBody, RigidBody> contactablePlaneBodiesAndBases = new LinkedHashMap<ContactablePlaneBody, RigidBody>();

      // TODO: code duplication from driving controller
      SideDependentList<ContactablePlaneBody> hands = contactableBodiesFactory.createHandContactableBodies(fullRobotModel.getRootJoint().getSuccessor());

      for (ContactablePlaneBody contactablePlaneBody : feet.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getPelvis());
      }

      for (ContactablePlaneBody contactablePlaneBody : hands.values())
      {
         contactablePlaneBodiesAndBases.put(contactablePlaneBody, fullRobotModel.getChest());
      }

//      ContactPointGroundReactionWrenchDistributor groundReactionWrenchDistributor =
//         new ContactPointGroundReactionWrenchDistributor(referenceFrames.getCenterOfMassFrame(), registry);
//      double[] diagonalCWeights = new double[]
//      {
//         1.0, 1.0, 1.0, 1.0, 1.0, 1.0
//      };
//      groundReactionWrenchDistributor.setWeights(diagonalCWeights, 1.0, 0.001);

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
      MomentumBasedController momentumBasedController = new MomentumBasedController(fullRobotModel, centerOfMassJacobian,
                                                           referenceFrames, footSwitches, yoTime, gravityZ, twistCalculator, feet, hands, null, null, null, controlDT,
                                                           momentumOptimizationSettings, oldMomentumControlModule , null, dynamicGraphicObjectsListRegistry);

      DesiredHandPoseProvider handPoseProvider = new DesiredHandPoseProvider(fullRobotModel, walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame());
//      TorusPoseProvider torusPoseProvider = new TorusPoseProvider();
//      TorusManipulationProvider torusManipulationProvider = new TorusManipulationProvider();
      DesiredFootPoseProvider footPoseProvider = new DesiredFootPoseProvider();

      DesiredHandLoadBearingProvider handLoadBearingProvider = new DesiredHandLoadBearingProvider();
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
      
      ControlStatusProducer controlStatusProducer = new SystemErrControlStatusProducer();
     
      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footstepProvider, mapFromFootstepsToTrajectoryParameters,
            headOrientationProvider, comHeightProvider, pelvisPoseProvider, handPoseProvider, handLoadBearingProvider, chestOrientationProvider, footPoseProvider, footLoadBearingProvider, highLevelStateProvider, thighLoadBearingProvider,
            pelvisLoadBearingProvider, reinitializeWalkingController, controlStatusProducer);

      VariousWalkingManagers variousWalkingManagers = VariousWalkingManagers.create(momentumBasedController, yoTime, variousWalkingProviders, walkingControllerParameters,
            armControllerParameters, registry);
      variousWalkingManagers.initializeManagers();

      MultiContactTestHumanoidController multiContactBehavior = new MultiContactTestHumanoidController(variousWalkingProviders, variousWalkingManagers,
                                                                   momentumRateOfChangeControlModule,
                                                                   momentumBasedController, walkingControllerParameters, null,
                                                                   dynamicGraphicObjectsListRegistry);

      SideDependentList<Boolean> areHandsInContact = new SideDependentList<Boolean>(false, false);
      SideDependentList<Boolean> areFeetInContact = new SideDependentList<>(false, false);
      for (RobotSide robotSide : footContactSides)
         areFeetInContact.put(robotSide, true);
      for (RobotSide robotSide : handContactSides)
         areHandsInContact.put(robotSide, true);
      multiContactBehavior.initializeContactStates(areHandsInContact, areFeetInContact);

      ArrayList<HighLevelBehavior> highLevelBehaviors = new ArrayList<>();
      highLevelBehaviors.add(multiContactBehavior);

      // This is the "highest level" controller that enables switching between the different controllers (walking, multi-contact, driving, etc.)
      HighLevelHumanoidControllerManager ret = new HighLevelHumanoidControllerManager(HighLevelState.MULTI_CONTACT, highLevelBehaviors, momentumBasedController, null);

      return ret;
   }

}
