package us.ihmc.avatar.jumpingSimulation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.*;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.*;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.InputStream;
import java.util.*;

public class JumpingSimulationController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final JumpingControllerState controller;
//   private final StateEstimatorController estimator;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidFloatingRootJointRobot humanoidRobotModel;
   private final JumpingControllerToolbox controllerToolbox;
   private final YoDouble time;

   private final SDFPerfectSimulatedSensorReader sensorReader;
   private final PerfectSimulatedOutputWriter outputWriter;

   public JumpingSimulationController(CommandInputManager commandInputManager,
                                      DRCRobotModel robotModel,
                                      FullHumanoidRobotModel fullRobotModel,
                                      HumanoidFloatingRootJointRobot humanoidRobotModel,
                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                      double gravityZ,
                                      InputStream parameterFile)
   {
      this.fullRobotModel = fullRobotModel;
      this.humanoidRobotModel = humanoidRobotModel;
      time = humanoidRobotModel.getYoTime();

      // set up the contactable bodies factory
      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      SegmentDependentList<RobotSide, ArrayList<Point2D>> footContactPoints = contactPointParameters.getFootContactPoints();
      SegmentDependentList<RobotSide, Point2D> toeContactPoints = contactPointParameters.getControllerToeContactPoints();
      SegmentDependentList<RobotSide, LineSegment2D> toeContactLines = contactPointParameters.getControllerToeContactLines();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(footContactPoints);
      contactableBodiesFactory.setToeContactParameters(toeContactPoints, toeContactLines);

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(this.fullRobotModel.getForceSensorDefinitions()));

      // Make the sensor reader
      List<WrenchCalculatorInterface> forceSensors = new ArrayList<>();
      List<IMUMount> imuMounts = new ArrayList<>();
      humanoidRobotModel.getForceSensors(forceSensors);
      humanoidRobotModel.getIMUMounts(imuMounts);

      sensorReader = new SDFPerfectSimulatedSensorReader(humanoidRobotModel, this.fullRobotModel, forceSensorDataHolder, null);
      if (this.fullRobotModel.getIMUDefinitions() != null)
      {
         for (IMUDefinition imuDefinition : this.fullRobotModel.getIMUDefinitions())
         {
            for (IMUMount imuMount : imuMounts)
            {
               if (imuDefinition.getName().equals(imuMount.getName()))
               {
                  sensorReader.addIMUSensor(imuDefinition, imuMount);
                  break;
               }
            }
         }
      }
      if (this.fullRobotModel.getForceSensorDefinitions() != null)
      {
         for (ForceSensorDefinition forceSensorDefinition : this.fullRobotModel.getForceSensorDefinitions())
         {
            for (WrenchCalculatorInterface forceSensor : forceSensors)
            {
               if (forceSensorDefinition.getSensorName().equals(forceSensor.getName()))
               {
                  sensorReader.addForceTorqueSensorPort(forceSensorDefinition, forceSensor);
                  break;
               }
            }
         }
      }


      // Create DRC Estimator:
      KinematicsBasedStateEstimatorFactory estimatorFactory = new KinematicsBasedStateEstimatorFactory();
      estimatorFactory.setEstimatorFullRobotModel(this.fullRobotModel);
      estimatorFactory.setSensorInformation(robotModel.getSensorInformation());
      estimatorFactory.setSensorOutputMapReadOnly(sensorReader);
      estimatorFactory.setGravity(gravityZ);
      estimatorFactory.setStateEstimatorParameters(robotModel.getStateEstimatorParameters());
      estimatorFactory.setContactableBodiesFactory(contactableBodiesFactory);
      estimatorFactory.setEstimatorForceSensorDataHolder(forceSensorDataHolder);
      estimatorFactory.setCenterOfPressureDataHolderFromController(new CenterOfPressureDataHolder(this.fullRobotModel));
      estimatorFactory.setRobotMotionStatusFromController(new RobotMotionStatusHolder());

//      YoRegistry stateEstimatorRegistry = new YoRegistry("stateEstimatorRegistry");
//      estimator = estimatorFactory.createStateEstimator(stateEstimatorRegistry, yoGraphicsListRegistry);
//      registry.addChild(stateEstimatorRegistry);
//      registry.addChild(estimator.getYoRegistry());

      // Create the jumping controller
      CommonHumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(this.fullRobotModel);

      contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFullRobotModel(this.fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      contactableBodiesFactory.setFootContactPoints(footContactPoints);
      contactableBodiesFactory.setToeContactParameters(toeContactPoints, toeContactLines);

      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());

      double totalMass = TotalMassCalculator.computeSubTreeMass(this.fullRobotModel.getElevator());
      double totalRobotWeight = totalMass * gravityZ;
      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet,
                                                                               forceSensorDataHolder,
                                                                               robotModel.getWalkingControllerParameters(),
                                                                               totalRobotWeight,
                                                                               robotModel.getSensorInformation().getFeetForceSensorNames(),
                                                                               yoGraphicsListRegistry,
                                                                               registry);
      JointBasics[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(this.fullRobotModel, robotModel, robotModel.getSensorInformation());

      controllerToolbox = new JumpingControllerToolbox(this.fullRobotModel,
                                                                                referenceFrames,
                                                                                robotModel.getWalkingControllerParameters(),
                                                                                footSwitches,
                                                                                humanoidRobotModel.getYoTime(),
                                                                                gravityZ,
                                                                                robotModel.getWalkingControllerParameters().getOmega0(),
                                                                                feet,
                                                                                robotModel.getControllerDT(),
                                                                                new ArrayList<>(),
                                                                                yoGraphicsListRegistry,
                                                                                jointsToIgnore);
      registry.addChild(controllerToolbox.getYoVariableRegistry());
//      controllerToolbox.attachControllerFailureListener(fallingDirection -> hasControllerFailed.set(true));

      JumpingCoPTrajectoryParameters jumpingCoPTrajectoryParameters = new JumpingCoPTrajectoryParameters(registry);
      JumpingParameters jumpingParameters = new JumpingParameters(registry);
      JumpingControlManagerFactory managerFactory = new JumpingControlManagerFactory(registry);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setCoPTrajectoryParameters(robotModel.getCoPTrajectoryParameters());
      managerFactory.setWalkingControllerParameters(robotModel.getWalkingControllerParameters());
      managerFactory.setJumpingCoPTrajectoryParameters(jumpingCoPTrajectoryParameters);

      controller = new JumpingControllerState(commandInputManager,
                                              managerFactory,
                                              controllerToolbox,
                                              robotModel.getHighLevelControllerParameters(),
                                              robotModel.getWalkingControllerParameters(),
                                              jumpingCoPTrajectoryParameters,
                                              jumpingParameters);
      registry.addChild(controller.getYoRegistry());

      // Set up the output writer
      outputWriter = new PerfectSimulatedOutputWriter(humanoidRobotModel, this.fullRobotModel, controller.getOutputForLowLevelController());

      ParameterLoaderHelper.loadParameters(this, parameterFile, registry);
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<? extends ContactablePlaneBody> bipedFeet,
                                                                     ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                                     WalkingControllerParameters walkingControllerParameters,
                                                                     double totalRobotWeight,
                                                                     SideDependentList<String> footSensorNames,
                                                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                                                     YoRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>();

      FootSwitchFactory footSwitchFactory = walkingControllerParameters.getFootSwitchFactory();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = bipedFeet.get(robotSide).getName();
         ForceSensorDataReadOnly footForceSensor = forceSensorDataHolder.getByName(footSensorNames.get(robotSide));
         FootSwitchInterface footSwitch = footSwitchFactory.newFootSwitch(footName, bipedFeet.get(robotSide),
                                                                          Collections.singleton(bipedFeet.get(robotSide.getOppositeSide())), footForceSensor,
                                                                          totalRobotWeight, yoGraphicsListRegistry, registry);
         footSwitches.put(robotSide, footSwitch);
      }

      return footSwitches;
   }

   public void attachControllerFailureListener(ControllerFailureListener controllerFailureListener)
   {
      controllerToolbox.attachControllerFailureListener(controllerFailureListener);
   }

   private boolean initialized = false;

   @Override
   public void initialize()
   {
      sensorReader.initialize();
//      estimator.initialize();
      controller.initialize();
      outputWriter.initialize();

      initialized = true;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }


   @Override
   public void doControl()
   {
      if (!initialized)
         initialize();

      sensorReader.read();

//      estimator.doControl();
      controller.doAction(time.getDoubleValue());

      outputWriter.write();
   }

}
