package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.atlas.initialSetup.MultiContactDRCRobotInitialSetup;
import us.ihmc.atlas.initialSetup.PushUpDRCRobotInitialSetup;
import us.ihmc.atlas.parameters.AtlasArmControllerParameters;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MultiContactTestHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.UserDesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.MultiContactTestEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class AtlasMultiContact
{
   private static final AtlasRobotVersion ATLAS_ROBOT_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS;

   private final DRCSimulationFactory drcSimulation;
   private final MultiContactTestEnvironment environment;
   private final SimulationConstructionSet simulationConstructionSet;

   public static enum MultiContactTask
   {
      DEFAULT, PUSHUP
   };

   public AtlasMultiContact(AtlasRobotModel robotModel, DRCGuiInitialSetup guiInitialSetup, MultiContactTask task)
   {
      RobotSide[] footContactSides;
      RobotSide[] handContactSides;
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup;
      switch (task)
      {
      case PUSHUP:
         footContactSides = RobotSide.values;
         handContactSides = RobotSide.values;
         robotInitialSetup = new PushUpDRCRobotInitialSetup();
         break;
      default:
         footContactSides = RobotSide.values;
         handContactSides = new RobotSide[] {RobotSide.LEFT};
         robotInitialSetup = new MultiContactDRCRobotInitialSetup();
         break;

      }

      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createInvisibleHandContactPoints();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();

      environment = new MultiContactTestEnvironment(robotInitialSetup, robotModel, footContactSides, handContactSides,
            contactPointParameters.getHandContactPointTransforms());
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setRunMultiThreaded(true);

      double dt = scsInitialSetup.getDT();
      int recordFrequency = (int) Math.round(robotModel.getControllerDT() / dt);
      if (recordFrequency < 1)
         recordFrequency = 1;
      scsInitialSetup.setRecordFrequency(recordFrequency);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      ArmControllerParameters armControllerParameters = new AtlasArmControllerParameters()
      {
         public Map<OneDoFJoint, Double> getDefaultArmJointPositions(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
         {
            Map<OneDoFJoint, Double> jointPositions = new LinkedHashMap<OneDoFJoint, Double>();
            EnumMap<ArmJointName, Double> defaultArmPosition = MultiContactDRCRobotInitialSetup.getDefaultArmPositionForMultiContactSimulation().get(robotSide);

            for (ArmJointName armJointName : defaultArmPosition.keySet())
            {
               double position = defaultArmPosition.get(armJointName);
               OneDoFJoint joint = fullRobotModel.getArmJoint(robotSide, armJointName);
               jointPositions.put(joint, position);
            }

            return jointPositions;
         }
      };

      WalkingControllerParameters controllerParameters = robotModel.getWalkingControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, controllerParameters, armControllerParameters, capturePointPlannerParameters,
            HighLevelState.DO_NOTHING_BEHAVIOR);

      controllerFactory
            .addHighLevelBehaviorFactory(new MultiContactTestHumanoidControllerFactory(controllerParameters, footContactSides, handContactSides, true));
      VariousWalkingProviderFactory variousWalkingProviderFactory = createVariousWalkingProviderFactory();
      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);

      drcSimulation = new DRCSimulationFactory(robotModel, controllerFactory, environment, robotInitialSetup, scsInitialSetup, guiInitialSetup, null);

      simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      MidiSliderBoard sliderBoard = new MidiSliderBoard(simulationConstructionSet);
      sliderBoard.setSlider(1, "desiredCoMX", simulationConstructionSet, -0.2, 0.2);
      sliderBoard.setSlider(2, "desiredCoMY", simulationConstructionSet, -0.2, 0.2);
      sliderBoard.setSlider(3, "desiredCoMZ", simulationConstructionSet, 0.5, 1.5);
      sliderBoard.setSlider(4, "userDesiredPelvisYaw", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setSlider(5, "userDesiredPelvisPitch", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setSlider(6, "userDesiredPelvisRoll", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(1, "desiredChestYaw", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(2, "desiredChestPitch", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);
      sliderBoard.setKnob(3, "desiredChestRoll", simulationConstructionSet, -Math.PI / 8.0, Math.PI / 8.0);

      simulationConstructionSet.setCameraPosition(6.0, -2.0, 4.5);
      simulationConstructionSet.setCameraFix(-0.44, -0.17, 0.75);

      drcSimulation.start();
   }

   private VariousWalkingProviderFactory createVariousWalkingProviderFactory()
   {
      return new VariousWalkingProviderFactory()
      {
         @Override
         public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullHumanoidRobotModel fullRobotModel,
               WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
               SideDependentList<ContactablePlaneBody> feet, ConstantTransferTimeCalculator transferTimeCalculator,
               ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, YoVariableRegistry registry,
               YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
         {
            PelvisPoseProvider desiredPelvisPoseProvider = new UserDesiredPelvisPoseProvider(registry);
            HandPoseProvider desiredHandPoseProvider = new DesiredHandPoseProvider(referenceFrames, fullRobotModel,
                  walkingControllerParameters.getDesiredHandPosesWithRespectToChestFrame(), null);
            HandLoadBearingProvider desiredHandLoadBearingProvider = new DesiredHandLoadBearingProvider();
            return new VariousWalkingProviders(null, null, null, null, null, null, null, null, null, desiredPelvisPoseProvider, desiredHandPoseProvider, null,
                  null, desiredHandLoadBearingProvider, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null);
         }
      };
   }

   public DRCSimulationFactory getDRCSimulation()
   {
      return drcSimulation;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public static void main(String[] args) throws JSAPException
   {

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, true);

      new AtlasMultiContact(new AtlasRobotModel(ATLAS_ROBOT_VERSION, DRCRobotModel.RobotTarget.SCS, false), guiInitialSetup, MultiContactTask.DEFAULT);
   }

}
