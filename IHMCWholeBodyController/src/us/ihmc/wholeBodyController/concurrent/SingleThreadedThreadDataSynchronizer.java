package us.ihmc.wholeBodyController.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class SingleThreadedThreadDataSynchronizer implements ThreadDataSynchronizerInterface
{
   private final SDFFullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;
   private final ContactSensorHolder estimatorContactSensorHolder;
   private final DesiredJointDataHolder estimatorDesiredJointDataHolder;

   private final SDFFullHumanoidRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   private final ContactSensorHolder controllerContactSensorHolder;
   private final DesiredJointDataHolder controllerDesiredJointDataHolder;

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
      estimatorContactSensorHolder = new ContactSensorHolder(Arrays.asList(estimatorFullRobotModel.getContactSensorDefinitions()));
      estimatorRobotMotionStatusHolder = new RobotMotionStatusHolder();
      estimatorDesiredJointDataHolder = new DesiredJointDataHolder(estimatorFullRobotModel.getOneDoFJoints());

      controllerFullRobotModel = estimatorFullRobotModel;
      controllerForceSensorDataHolder = estimatorForceSensorDataHolder;
      controllerRawJointSensorDataHolderMap = estimatorRawJointSensorDataHolderMap;
      controllerCenterOfPressureDataHolder = estimatorCenterOfPressureDataHolder;
      controllerRobotMotionStatusHolder = estimatorRobotMotionStatusHolder;
      controllerContactSensorHolder = estimatorContactSensorHolder;
      controllerDesiredJointDataHolder = estimatorDesiredJointDataHolder;
      
      this.fullRobotModelRewinder = new FullRobotModelRootJointRewinder(estimatorFullRobotModel, registry);
      if(scs != null)
      {
    	  scs.attachSimulationRewoundListener(fullRobotModelRewinder);
      }
   }

   @Override
   public boolean receiveEstimatorStateForController()
   {
      return true;
   }

   @Override
   public void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      this.timestamp.set(timestamp);
      this.estimatorTick.set(estimatorTick);
      this.estimatorClockStartTime.set(estimatorClockStartTime); 
      
      // Record full robot model here for rewindability.
      fullRobotModelRewinder.recordCurrentState();
   }

   @Override
   public SDFFullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   @Override
   public SDFFullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }

   @Override
   public RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap()
   {
      return estimatorRawJointSensorDataHolderMap;
   }

   @Override
   public RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap()
   {
      return controllerRawJointSensorDataHolderMap;
   }

   @Override
   public CenterOfPressureDataHolder getEstimatorCenterOfPressureDataHolder()
   {
      return estimatorCenterOfPressureDataHolder;
   }

   @Override
   public CenterOfPressureDataHolder getControllerCenterOfPressureDataHolder()
   {
      return controllerCenterOfPressureDataHolder;
   }

   @Override
   public RobotMotionStatusHolder getEstimatorRobotMotionStatusHolder()
   {
      return estimatorRobotMotionStatusHolder;
   }

   @Override
   public RobotMotionStatusHolder getControllerRobotMotionStatusHolder()
   {
      return controllerRobotMotionStatusHolder;
   }

   @Override
   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }

   @Override
   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime.getLongValue();
   }

   @Override
   public long getEstimatorTick()
   {
      return estimatorTick.getLongValue();
   }

   @Override
   public void publishControllerData()
   {      
      controllerDesiredJointDataHolder.updateFromModel();
   }

   @Override
   public boolean receiveControllerDataForEstimator()
   {
      return true;
   }

   @Override
   public ContactSensorHolder getControllerContactSensorHolder()
   {
      return controllerContactSensorHolder;
   }

   @Override
   public ContactSensorHolder getEstimatorContactSensorHolder()
   {
      return estimatorContactSensorHolder;
   }

   @Override
   public DesiredJointDataHolder getEstimatorDesiredJointDataHolder()
   {
      return estimatorDesiredJointDataHolder;
   }

}
