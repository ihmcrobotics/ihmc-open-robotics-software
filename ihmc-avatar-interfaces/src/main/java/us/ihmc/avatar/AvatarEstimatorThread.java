package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import org.apache.commons.lang3.tuple.ImmutablePair;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarEstimatorThread extends ModularRobotController implements SCS2YoGraphicHolder
{
   private final YoRegistry estimatorRegistry;
   private final YoBoolean firstTick;

   private final SensorReader sensorReader;
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final StateEstimatorController mainStateEstimator;
   private final PairList<BooleanSupplier, StateEstimatorController> secondaryStateEstimators;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();

   private final HumanoidRobotContextData humanoidRobotContextData;

   private final List<Runnable> runnables = new ArrayList<>();
   private final ROS2PublisherBasics<ControllerCrashNotificationPacket> controllerCrashPublisher;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final ExecutionTimer estimatorThreadTimer;

   public AvatarEstimatorThread(SensorReader sensorReader,
                                FullHumanoidRobotModel estimatorFullRobotModel,
                                HumanoidRobotContextData humanoidRobotContextData,
                                StateEstimatorController mainStateEstimator,
                                PairList<BooleanSupplier, StateEstimatorController> secondaryStateEstimators,
                                ROS2PublisherBasics<ControllerCrashNotificationPacket> controllerCrashPublisher,
                                YoRegistry estimatorRegistry,
                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super("EstimatorController");

      this.sensorReader = sensorReader;
      this.estimatorFullRobotModel = estimatorFullRobotModel;
      this.humanoidRobotContextData = humanoidRobotContextData;
      this.mainStateEstimator = mainStateEstimator;
      this.secondaryStateEstimators = secondaryStateEstimators;
      this.controllerCrashPublisher = controllerCrashPublisher;
      this.estimatorRegistry = estimatorRegistry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      // This is to preserve the registry hierarchy for the parameters to work
      estimatorRegistry.addChild(super.getYoRegistry());

      if (mainStateEstimator != null)
         addRobotController(mainStateEstimator);

      for (int i = 0; i < secondaryStateEstimators.size(); i++)
      {
         addRobotController(secondaryStateEstimators.get(i).getRight());
      }

      estimatorThreadTimer = new ExecutionTimer("estimatorThreadTimer", super.getYoRegistry());

      firstTick = new YoBoolean("firstTick", estimatorRegistry);
      firstTick.set(true);
   }

   public void run()
   {
      estimatorThreadTimer.startMeasurement();

      try
      {
         // In the case the SensorReader.compute() does fill the sensor data, it has to be called before initialize of the state estimator.
         sensorReader.compute(humanoidRobotContextData.getTimestamp(), humanoidRobotContextData.getSensorDataContext());

         if (firstTick.getBooleanValue())
         {
            initialize();

            firstTick.set(false);
         }

         for (int i = 0; i < secondaryStateEstimators.size(); i++)
         {
            estimatorFullRobotModel.getRootJoint().getJointConfiguration(rootToWorldTransform);
            if (secondaryStateEstimators.get(i).getLeft().getAsBoolean())
               secondaryStateEstimators.get(i).getRight().initializeEstimator(rootToWorldTransform);
         }

         doControl();

         for (int i = 0; i < runnables.size(); i++)
         {
            runnables.get(i).run();
         }

         HumanoidRobotContextTools.updateContext(estimatorFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         humanoidRobotContextData.setEstimatorRan(!firstTick.getValue());
      }
      catch (Throwable e)
      {
         if (controllerCrashPublisher != null)
         {
            controllerCrashPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.ESTIMATOR_RUN, e));
         }
         throw new RuntimeException(e);
      }

      estimatorThreadTimer.stopMeasurement();
   }

   public void initializeStateEstimators(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      sensorReader.initialize();

      if (mainStateEstimator != null)
         mainStateEstimator.initializeEstimator(rootJointTransform, jointPositions);

      for (int i = 0; i < secondaryStateEstimators.size(); i++)
      {
         secondaryStateEstimators.get(i).getRight().initializeEstimator(rootJointTransform, jointPositions);
      }

      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);

      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList = humanoidRobotContextData.getJointDesiredOutputList();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         jointDesiredOutputList.getJointDesiredOutput(i).clear();
      }
   }

   public void setupHighLevelControllerCallback(HighLevelHumanoidControllerFactory controllerFactory,
                                                Map<HighLevelControllerName, StateEstimatorMode> stateModeMap)
   {
      controllerFactory.attachControllerStateChangedListener(new ControllerStateChangedListener()
      {
         @Override
         public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
         {
            StateEstimatorMode requestedMode = stateModeMap.get(newState);
            if (requestedMode != null)
               mainStateEstimator.requestStateEstimatorMode(requestedMode);
         }
      });
   }

   public void addEstimatorRunnable(Runnable runnable)
   {
      this.runnables.add(runnable);
   }

   @Deprecated // TODO: Split up the sensor reader and move the part needed outside out of this class!
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return estimatorRegistry;
   }

   public YoGraphicsListRegistry getSCS1YoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      if (mainStateEstimator != null)
         group.addChild(mainStateEstimator.getSCS2YoGraphics());
      for (ImmutablePair<BooleanSupplier, StateEstimatorController> entry : secondaryStateEstimators)
      {
         group.addChild(entry.getRight().getSCS2YoGraphics());
      }
      return group;
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return mainStateEstimator.getForceSensorCalibrationModule();
   }
}
