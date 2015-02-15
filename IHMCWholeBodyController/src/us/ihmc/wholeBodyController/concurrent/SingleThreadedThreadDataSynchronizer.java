package us.ihmc.wholeBodyController.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;

public class SingleThreadedThreadDataSynchronizer implements ThreadDataSynchronizerInterface
{
   private final SDFFullRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;

   private final SDFFullRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;

   private final LongYoVariable timestamp;
   private final LongYoVariable estimatorClockStartTime;
   private final LongYoVariable estimatorTick;
   
   private final FullRobotModelRootJointRewinder fullRobotModelRewinder;
   
   /**
    * SingleThreadedThreadDataSynchronizer is an alternative to ThreadDataSynchronizer when you want to run on a single thread and have 
    * deterministic rewindable execution. The FullRobotModels and other objects are just shared between the estimator and the controller.
    * @param wholeBodyControlParameters
    * @param registry
    */
   public SingleThreadedThreadDataSynchronizer(SimulationConstructionSet scs, WholeBodyControllerParameters wholeBodyControlParameters, YoVariableRegistry registry)
   {
      timestamp = new LongYoVariable(getClass().getSimpleName() + "Timestamp", registry);
      estimatorClockStartTime = new LongYoVariable(getClass().getSimpleName() + "EstimatorClockStartTime", registry);
      estimatorTick = new LongYoVariable(getClass().getSimpleName() + "EstimatorTick", registry);
      
      estimatorFullRobotModel = wholeBodyControlParameters.createFullRobotModel();
      estimatorForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()));
      estimatorRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(estimatorFullRobotModel);
      estimatorCenterOfPressureDataHolder = new CenterOfPressureDataHolder(estimatorFullRobotModel.getSoleFrames());

      controllerFullRobotModel = estimatorFullRobotModel;
      controllerForceSensorDataHolder = estimatorForceSensorDataHolder;
      controllerRawJointSensorDataHolderMap = estimatorRawJointSensorDataHolderMap;
      controllerCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      
      this.fullRobotModelRewinder = new FullRobotModelRootJointRewinder(estimatorFullRobotModel, registry);
      scs.attachSimulationRewoundListener(fullRobotModelRewinder);
   }

   public boolean receiveEstimatorStateForController()
   {
      return true;
   }

   public void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      this.timestamp.set(timestamp);
      this.estimatorTick.set(estimatorTick);
      this.estimatorClockStartTime.set(estimatorClockStartTime); 
      
      // Record full robot model here for rewindability.
      fullRobotModelRewinder.recordCurrentState();
   }

   public SDFFullRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   public SDFFullRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }

   public RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap()
   {
      return estimatorRawJointSensorDataHolderMap;
   }

   public RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap()
   {
      return controllerRawJointSensorDataHolderMap;
   }

   public CenterOfPressureDataHolder getEstimatorCenterOfPressureDataHolder()
   {
      return estimatorCenterOfPressureDataHolder;
   }

   public CenterOfPressureDataHolder getControllerCenterOfPressureDataHolder()
   {
      return controllerCenterOfPressureDataHolder;
   }

   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime.getLongValue();
   }

   public long getEstimatorTick()
   {
      return estimatorTick.getLongValue();
   }

   public void publishControllerData()
   {      
   }

   public boolean receiveControllerDataForEstimator()
   {
      return true;
   }

}
