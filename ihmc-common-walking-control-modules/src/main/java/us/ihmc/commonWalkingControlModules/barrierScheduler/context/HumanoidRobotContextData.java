package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.SensorDataContext;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@SuppressWarnings("serial")
public class HumanoidRobotContextData implements InPlaceCopyable<HumanoidRobotContextData>, Settable<HumanoidRobotContextData>
{
   /**
    * Serves to synchronize the time across threads.
    * TODO: should be set by the main robot thread
    */
   private long timestamp = Long.MIN_VALUE;

   /**
    * The measured force sensor data.
    * TODO: should be set by the main robot thread
    */
   private final ForceSensorDataHolder forceSensorDataHolder;

   /**
    * The joint measurements.
    * TODO: should be set by the main robot thread
    */
   private final SensorDataContext sensorDataContext;

   /**
    * Serves to inform the controller that the estimator ran and populated the estimated values in this context.
    * Set by the estimator.
    */
   private boolean estimatorRan = false;

   /**
    * Estimated state of the robot.
    * Set by the estimator.
    */
   private final HumanoidRobotContextJointData processedJointData;

   /**
    * Serves to inform the estimator that the controller ran and populated the desired values in this context.
    * Set by the controller.
    */
   private boolean controllerRan = false;

   /**
    * The controller desired center of pressure.
    * Set by the controller.
    */
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;

   /**
    * The motion status of the robot.
    * Set by the controller.
    */
   private final RobotMotionStatusHolder robotMotionStatusHolder;

   /**
    * The desired joint data to be set on the robot.
    * Set by the controller.
    */
   private final LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;

   public HumanoidRobotContextData()
   {
      processedJointData = new HumanoidRobotContextJointData();
      forceSensorDataHolder = new ForceSensorDataHolder();
      centerOfPressureDataHolder = new CenterOfPressureDataHolder();
      robotMotionStatusHolder = new RobotMotionStatusHolder();
      jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder();
      sensorDataContext = new SensorDataContext();
   }

   public HumanoidRobotContextData(HumanoidRobotContextJointData processedJointData, ForceSensorDataHolder forceSensorDataHolder,
                                   CenterOfPressureDataHolder centerOfPressureDataHolder, RobotMotionStatusHolder robotMotionStatusHolder,
                                   LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList, SensorDataContext sensorDataContext)
   {
      this.processedJointData = processedJointData;
      this.forceSensorDataHolder = forceSensorDataHolder;
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.sensorDataContext = sensorDataContext;
   }

   public HumanoidRobotContextJointData getProcessedJointData()
   {
      return processedJointData;
   }

   public ForceSensorDataHolder getForceSensorDataHolder()
   {
      return forceSensorDataHolder;
   }

   public CenterOfPressureDataHolder getCenterOfPressureDataHolder()
   {
      return centerOfPressureDataHolder;
   }

   public RobotMotionStatusHolder getRobotMotionStatusHolder()
   {
      return robotMotionStatusHolder;
   }

   public LowLevelOneDoFJointDesiredDataHolder getJointDesiredOutputList()
   {
      return jointDesiredOutputList;
   }

   public SensorDataContext getSensorDataContext()
   {
      return sensorDataContext;
   }

   @Override
   public void set(HumanoidRobotContextData other)
   {
      copyFrom(other);
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      this.timestamp = src.timestamp;
      this.controllerRan = src.controllerRan;
      this.estimatorRan = src.estimatorRan;
      this.processedJointData.set(src.processedJointData);
      this.forceSensorDataHolder.set(src.forceSensorDataHolder);
      this.centerOfPressureDataHolder.set(src.centerOfPressureDataHolder);
      this.robotMotionStatusHolder.set(src.robotMotionStatusHolder);
      this.jointDesiredOutputList.set(src.jointDesiredOutputList);
      this.sensorDataContext.set(src.sensorDataContext);
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public void setControllerRan(boolean controllerRan)
   {
      this.controllerRan = controllerRan;
   }

   public boolean getControllerRan()
   {
      return controllerRan;
   }

   public void setEstimatorRan(boolean estimatorRan)
   {
      this.estimatorRan = estimatorRan;
   }

   public boolean getEstimatorRan()
   {
      return estimatorRan;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof HumanoidRobotContextData)
      {
         HumanoidRobotContextData other = (HumanoidRobotContextData) obj;
         if (timestamp != other.timestamp)
            return false;
         if (controllerRan ^ other.controllerRan)
            return false;
         if (estimatorRan ^ other.estimatorRan)
            return false;
         if (!processedJointData.equals(other.processedJointData))
            return false;
         if (!forceSensorDataHolder.equals(other.forceSensorDataHolder))
            return false;
         if (!centerOfPressureDataHolder.equals(other.centerOfPressureDataHolder))
            return false;
         if (!robotMotionStatusHolder.equals(other.robotMotionStatusHolder))
            return false;
         if (!jointDesiredOutputList.equals(other.jointDesiredOutputList))
            return false;
         if (!sensorDataContext.equals(other.sensorDataContext))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
