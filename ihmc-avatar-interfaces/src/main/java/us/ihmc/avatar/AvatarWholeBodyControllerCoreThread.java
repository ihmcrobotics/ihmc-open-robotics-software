package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutPutDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HumanoidWholeBodyControllerCoreManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.wholeBodyController.CenterOfMassCalibrationTool;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import static us.ihmc.avatar.AvatarControllerThread.createListOfJointsToIgnore;

public class AvatarWholeBodyControllerCoreThread implements AvatarControllerThreadInterface
{
   private static final boolean CREATE_COM_CALIBRATION_TOOL = false;

   private final YoRegistry registry = new YoRegistry("WholeBodyControllerCoreThread");
   private final YoDouble wholeBodyControllerCoreTime = new YoDouble("WholeBodyControllerCoreTime", registry);
   private final FullHumanoidRobotModel controllerCoreFullRobotModel;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", registry);
   private final YoLong timeStamp = new YoLong("TimeStampWholeBodyControllerCore", registry);
   private final YoLong timeStampOffset = new YoLong("TimestampOffsetWholeBodyControllerCore", registry);
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final YoBoolean runWholeBodyControllerCore = new YoBoolean("RunWholeBodyControllerCore", registry);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final List<Supplier<YoGraphicDefinition>> scs2YoGraphicHolders = new ArrayList<>();
   private ModularRobotController wholeBodyControllerCoreCalculator = new ModularRobotController("WBCC");
   private final ExecutionTimer wholeBodyControllerCoreThreadTimer;

   public AvatarWholeBodyControllerCoreThread(String robotName,
                                              HumanoidRobotContextDataFactory contextDataFactory,
                                              StatusMessageOutputManager walkingOutputManager,
                                              DRCRobotModel robotModel,
                                              HumanoidRobotSensorInformation sensorInformation,
                                              HighLevelHumanoidControllerFactory controllerFactory,
                                              DRCOutputProcessor outputProcessor,
                                              RealtimeROS2Node realtimeROS2Node,
                                              double gravity,
                                              boolean kinematicSimulation)
   {
      this.controllerCoreFullRobotModel = robotModel.createFullRobotModel();

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(controllerCoreFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForWholeBodyControllerCore = new ForceSensorDataHolder(Arrays.asList(controllerCoreFullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForWholeBodyControllerCore = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForWholeBodyControllerCore = new CenterOfPressureDataHolder(controllerCoreFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllerCoreFullRobotModel.getControllableOneDoFJoints());
      LowLevelOneDoFJointDesiredDataHolder wholeBodyControllerCoreDesiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllerCoreFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      ControllerCoreOutPutDataHolder controllerCoreOutPutDataHolder = new ControllerCoreOutPutDataHolder(controllerCoreFullRobotModel.getControllableOneDoFJoints());
      ControllerCoreCommandDataHolder controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForWholeBodyControllerCore);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setWBCCJointDesiredOutputList(wholeBodyControllerCoreDesiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setControllerCoreOutputDataHolder(controllerCoreOutPutDataHolder);
      contextDataFactory.setControllerCoreCommandDataHolder(controllerCoreCommandDataHolder);
      contextDataFactory.setSensorDataContext(new SensorDataContext(controllerCoreFullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      /**
       * Something related to realtimeROS2Node should be here?
       */ // TODO check crashNotificationPublisher should be here which is called and set in the AvatarControllerThread
      // WholeBodyControllerCoreThread has not been yet made the API.

      JointBasics[] arrayOfJointsToIgnore = createListOfJointsToIgnore(controllerCoreFullRobotModel, robotModel, sensorInformation);

      //TODO I'm not sure this can be defined here. This was used to be written in the AvatarControllerThread.

      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(processedJointData, desiredJointDataHolder);
         outputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForWholeBodyControllerCore);
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }

      // similar role with the robotController in AvatarControllerThread
      wholeBodyControllerCoreCalculator = createWholeBodyControllerCoreCalculator(controllerCoreFullRobotModel,
                                                                                  controllerFactory,
                                                                                  wholeBodyControllerCoreTime,
                                                                                  robotModel.getControllerDT(),
                                                                                  gravity,
                                                                                  forceSensorDataHolderForWholeBodyControllerCore,
                                                                                  centerOfMassDataHolderForWholeBodyControllerCore,
                                                                                  centerOfPressureDataHolderForWholeBodyControllerCore,
                                                                                  sensorInformation,
                                                                                  wholeBodyControllerCoreDesiredJointDataHolder,
                                                                                  desiredJointDataHolder,
                                                                                  controllerCoreOutPutDataHolder,
                                                                                  controllerCoreCommandDataHolder,
                                                                                  registry,
                                                                                  kinematicSimulation,
                                                                                  arrayOfJointsToIgnore);
      //      desiredJointDataHolder.set(wholeBodyControllerCoreDesiredJointDataHolder);

      wholeBodyControllerCoreThreadTimer = new ExecutionTimer("WholeBodyControllerCoreTimer111", registry);

      firstTick.set(true);
      registry.addChild(wholeBodyControllerCoreCalculator.getYoRegistry());
   }

   private ModularRobotController createWholeBodyControllerCoreCalculator(FullHumanoidRobotModel controllerCoreModel,
                                                                          HighLevelHumanoidControllerFactory controllerFactory,
                                                                          YoDouble yoTime,
                                                                          double controllerCoreDT,
                                                                          double gravity,
                                                                          ForceSensorDataHolderReadOnly forceSensorDataHolderForControllerCore,
                                                                          CenterOfMassDataHolderReadOnly centerOfMassDataHolderForControllerCore,
                                                                          CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                                          HumanoidRobotSensorInformation sensorInformation,
                                                                          JointDesiredOutputListBasics wholeBodyControllerCoreOutput,
                                                                          JointDesiredOutputListBasics lowLevelControllerOutput,
                                                                          ControllerCoreOutPutDataHolder controllerCoreOutPutDataHolder,
                                                                          ControllerCoreCommandDataHolder controllerCoreCommandDataHolder,
                                                                          YoRegistry registry,
                                                                          boolean kinematicsSimulation,
                                                                          JointBasics... jointsToIgnore)
   {
      // same with the controllerThread call the estimated CoM from the context
      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerCoreModel,
                                                                                                      forceSensorDataHolderForControllerCore,
                                                                                                      null,
                                                                                                      registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch (Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }

      // similar with the controllerFactory.getController from ControllerThread.
      // TODO Is it possible to make controller with the same way of ControllerThread.
      // Tried other way by making separate manager without using the controllerFactory. But, states are needed to be shared between the controllerThread and wholeBodyControllerCoreThread.
      // Each state has its own set for the controllerCore constructor, and these were saved to the controllerCoreToolbox, which is defined from the controlToolbox.
      // Assume that the controllerFactory.getController should be defined here again.
      // Guess two separate thread generate in-defendant constructors.

      //first try to generate the controllerManager without using the controllerFactory.
      HumanoidWholeBodyControllerCoreManager controllerCoreManager = new HumanoidWholeBodyControllerCoreManager(controllerCoreModel,
                                                                                                                      controllerCoreDT,
                                                                                                                      gravity,
                                                                                                                      kinematicsSimulation,
                                                                                                                      yoTime,
                                                                                                                      forceSensorDataHolderForControllerCore,
                                                                                                                      centerOfMassDataHolderForControllerCore,
                                                                                                                      wholeBodyControllerCoreOutput,
                                                                                                                      lowLevelControllerOutput,
                                                                                                                      controllerCoreOutPutDataHolder,
                                                                                                                      controllerCoreCommandDataHolder,
                                                                                                                      controllerFactory.getWholeBodyControllerCoreFactory()
                                                                                                                                       .getWholeBodyControllerCore(),
                                                                                                                      jointsToIgnore);

      // new try using the controllerFactory to keep the same stateMachine structure of controllerThread and use it.
      // use the same constructor is not possible. The registry values can not be generated with the same name.
      // Generating new one can't be. When set the feet, contactableBodiesFactory should createFootContactableFeet, but this is cannot be called twice.
      // ControllerCoreFactory should be shared through the context.
      // If so, how the stateMachine is shared?
//      HumanoidHighLevelControllerManager controllerCoreManager = controllerFactory.getControllerCoreCalculator(controllerCoreModel,
//                                                                                                               controllerCoreDT,
//                                                                                                               gravity,
//                                                                                                               kinematicsSimulation,
//                                                                                                               yoTime,
//                                                                                                               yoGraphicsListRegistry,
//                                                                                                               sensorInformation,
//                                                                                                               forceSensorDataHolderForControllerCore,
//                                                                                                               centerOfMassDataHolderForControllerCore,
//                                                                                                               centerOfPressureDataHolderForEstimator,
//                                                                                                               wholeBodyControllerCoreOutput,
//                                                                                                               controllerCoreOutPutDataHolder,
//                                                                                                               controllerCoreCommandDataHolder,
//                                                                                                               jointsToIgnore);

//      scs2YoGraphicHolders.add(() -> controllerCoreManager.getSCS2YoGraphics());

      ModularRobotController modularRobotController = new ModularRobotController("WholeBodyControllerCoreThread1");
      modularRobotController.addRobotController(controllerCoreManager);

      return modularRobotController;
   }

   public void initialize()
   {
      firstTick.set(true);
      humanoidRobotContextData.setWholeBodyControllerCoreRan(false);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);

      //TODO This should be called in here
      // This is previously called in AvatarControllerThread
      // The controllerThread and enstimationThread have this, respectively.
      // This seems to be able to be duplicated.
      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList = humanoidRobotContextData.getJointDesiredOutputList();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         jointDesiredOutputList.getJointDesiredOutput(i).clear();
      }
   }

   private void runOnFirstTick()
   {

   }

   @Override
   public void run()
   {
      //      wholeBodyControllerCoreThreadTimer.startMeasurement();
      // TODO getWholeBodyControllerCoreRan() should be called in the controllerThread.
      // This tells the controlThread that the wholeBodyControllerCore runs
      runWholeBodyControllerCore.set(humanoidRobotContextData.getEstimatorRan());
      if (!runWholeBodyControllerCore.getValue())
      {
         return;
      }

      try
      {
         HumanoidRobotContextTools.updateRobot(controllerCoreFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         timeStamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            initialize();
            timeStampOffset.set(timeStamp.getValue());
         }
         wholeBodyControllerCoreTime.set(Conversions.nanosecondsToSeconds(timeStamp.getValue() - timeStampOffset.getValue()));
         if (firstTick.getValue())
         {
            wholeBodyControllerCoreCalculator.initialize();
            firstTick.set(false);
         }
         wholeBodyControllerCoreCalculator.doControl();
         humanoidRobotContextData.setWholeBodyControllerCoreRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
//            wholeBodyControllerCoreThreadTimer.stopMeasurement();
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return controllerCoreFullRobotModel;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   public JointDesiredOutputListBasics getDesiredJointDataHolder()
   {
      return humanoidRobotContextData.getJointDesiredOutputList();
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (int i = 0; i < scs2YoGraphicHolders.size(); i++)
      {
         group.addChild(scs2YoGraphicHolders.get(i).get());
      }
      return group;
   }

   public YoGraphicsListRegistry getSCS1YoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }
}
