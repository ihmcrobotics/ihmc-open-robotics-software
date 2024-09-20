package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
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

   private final YoRegistry registry = new YoRegistry("WholeBodyControllerCore");
   private final YoDouble wholeBodyControllerCoreTime = new YoDouble("WholeBodyControllerCoreTime", registry);
   private final FullHumanoidRobotModel controllerCoreFullRobotModel;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", registry);
   private final YoLong timeStamp = new YoLong("TimeStampWholeBodyControllerCore", registry);
   private final YoLong timeStampOffset = new YoLong("TimestampOffsetWholeBodyControllerCore", registry);
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final YoBoolean runWholeBodyControllerCore = new YoBoolean("RunWholeBodyControllerCore", registry);
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final List<Supplier<YoGraphicDefinition>> scs2YoGraphicHolders = new ArrayList<>();
   private final ModularRobotController wholeBodyControllerCoreCalculator = new ModularRobotController("WBCC");
   //   private final ExecutionTimer wholeBodyControllerCoreThreadTimer;

   public AvatarWholeBodyControllerCoreThread(HumanoidRobotContextDataFactory contextDataFactory,
                                              StatusMessageOutputManager walkingOutputManager,
                                              DRCRobotModel robotModel,
                                              HumanoidRobotSensorInformation sensorInformation,
                                              DRCOutputProcessor outputProcessor,
                                              RealtimeROS2Node realtimeROS2Node)
   {
      this.controllerCoreFullRobotModel = robotModel.createFullRobotModel();
      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(controllerCoreFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForWholeBodyControllerCore = new ForceSensorDataHolder(Arrays.asList(controllerCoreFullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForWholeBodyControllerCore = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForWholeBodyControllerCore = new CenterOfPressureDataHolder(controllerCoreFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controllerCoreFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForWholeBodyControllerCore);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(controllerCoreFullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      JointBasics[] arrayOfJointsToIgnore = createListOfJointsToIgnore(controllerCoreFullRobotModel, robotModel, sensorInformation);
      //      wholeBodyControllerCoreCalculator = createWholeBodyControllerCoreCalculator();
      if (outputProcessor != null)
      {
         outputProcessor.setLowLevelControllerCoreOutput(processedJointData, desiredJointDataHolder);
         outputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForWholeBodyControllerCore);
         registry.addChild(outputProcessor.getControllerYoVariableRegistry());
      }
   }

   private ModularRobotController createWholeBodyControllerCoreCalculator(FullHumanoidRobotModel controllerCoreModel,
                                                                          HighLevelHumanoidControllerFactory controllerFactory,
                                                                          YoDouble yoTime,
                                                                          double wbccDT,
                                                                          double gravity,
                                                                          ForceSensorDataHolderReadOnly forceSensorDataHolderForControllerCore,
                                                                          CenterOfMassDataHolderReadOnly centerOfMassDataHolderForControllerCore,
                                                                          CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                                                          HumanoidRobotSensorInformation sensorInformation,
                                                                          JointDesiredOutputListBasics lowLevelControllerOutput,
                                                                          YoRegistry registry,
                                                                          boolean kinematicsSimulation,
                                                                          JointBasics... jointToIgnore)
   {
      if (CREATE_COM_CALIBRATION_TOOL)
      {
         try
         {
            CenterOfMassCalibrationTool centerOfMassCalibrationTool = new CenterOfMassCalibrationTool(controllerCoreModel,
                                                                                                      forceSensorDataHolderForControllerCore,
                                                                                                      yoGraphicsListRegistry,
                                                                                                      registry);
            controllerFactory.addUpdatable(centerOfMassCalibrationTool);
         }
         catch (Exception e)
         {
            System.err.println("Couldn't create CenterOfMassCalibrationTool");
         }
      }
      ModularRobotController modularRobotController = new ModularRobotController("WholeBodyControllerCore");

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
      //runWholeBodyControllerCore.set(humanoidRobotContextData.getWholeBodyControllerCoreRan());
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
         humanoidRobotContextData.setWholeBodyControllerCoreRan(false);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
      //      wholeBodyControllerCoreThreadTimer.stopMeasurement();
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
