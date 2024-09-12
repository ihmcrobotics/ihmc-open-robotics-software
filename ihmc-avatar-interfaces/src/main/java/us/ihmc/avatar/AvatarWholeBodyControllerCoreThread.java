package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Arrays;

public class AvatarWholeBodyControllerCoreThread implements AvatarControllerThreadInterface
{
   private final YoRegistry wbccRegistry = new YoRegistry("WholeBodyControllerCore");
   private final YoDouble wholeBodyControllerCoreTime = new YoDouble("WholeBodyControllerCoreTime", wbccRegistry);
   private final FullHumanoidRobotModel wholeBodyControllerCoreFullRobotModel;
   private final YoBoolean firstTick = new YoBoolean("FirstTick", wbccRegistry);
   private final YoLong timeStamp = new YoLong("TimeStampWholeBodyControllerCore", wbccRegistry);
   private final YoLong timeStampOffset = new YoLong("TimestampOffsetWholeBodyControllerCore", wbccRegistry);
   private final HumanoidRobotContextData humanoidRobotContextData;
   private final YoBoolean runWholeBodyControllerCore = new YoBoolean("RunWholeBodyControllerCore", wbccRegistry);
   private final ModularRobotController robotController;
   private final ExecutionTimer wholeBodyControllerCoreThreadTimer;

   public AvatarWholeBodyControllerCoreThread(HumanoidRobotContextDataFactory contextDataFactory,
                                              StatusMessageOutputManager walkingOutputManager,
                                              DRCRobotModel drcRobotModel,
                                              RealtimeROS2Node ros2Node)
   {
      this.wholeBodyControllerCoreFullRobotModel = drcRobotModel.createFullRobotModel();
      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(wholeBodyControllerCoreFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolderForWholeBodyControllerCore = new ForceSensorDataHolder(Arrays.asList(wholeBodyControllerCoreFullRobotModel.getForceSensorDefinitions()));
      CenterOfMassDataHolder centerOfMassDataHolderForWholeBodyControllerCore = new CenterOfMassDataHolder();
      CenterOfPressureDataHolder centerOfPressureDataHolderForWholeBodyControllerCore = new CenterOfPressureDataHolder(wholeBodyControllerCoreFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(wholeBodyControllerCoreFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfMassDataHolder(centerOfMassDataHolderForWholeBodyControllerCore);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderForWholeBodyControllerCore);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusHolder);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(wholeBodyControllerCoreFullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();
   }

   public void initialize()
   {
      firstTick.set(true);
      humanoidRobotContextData.setWholeBodyControllerCoreRan(false);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);

      //TODO This should be called in here
      // This is previously called in AvatarControllerThread
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
      wholeBodyControllerCoreThreadTimer.startMeasurement();
      // TODO getWholeBodyControllerCoreRan() should be called in the controllerThread.
      // This tells the controlThread that the wholeBodyControllerCore runs
      //runWholeBodyControllerCore.set(humanoidRobotContextData.getWholeBodyControllerCoreRan());
      if (!runWholeBodyControllerCore.getValue())
      {
         return;
      }

      try
      {
         timeStamp.set(humanoidRobotContextData.getTimestamp());
         if (firstTick.getValue())
         {
            initialize();
            timeStampOffset.set(timeStamp.getValue());
         }
         wholeBodyControllerCoreTime.set(Conversions.nanosecondsToSeconds(timeStamp.getValue() - timeStampOffset.getValue()));
         if(firstTick.getValue())
         {
            firstTick.set(false);
         }
         humanoidRobotContextData.setWholeBodyControllerCoreRan(true);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
      wholeBodyControllerCoreThreadTimer.stopMeasurement();
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      return null;
   }

   @Override
   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return null;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
