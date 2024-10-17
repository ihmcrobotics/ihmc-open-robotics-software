package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutPutDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

import java.util.Arrays;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@SuppressWarnings("serial")
public class HumanoidRobotContextData implements InPlaceCopyable<HumanoidRobotContextData>, Settable<HumanoidRobotContextData>
{
   /**
    * Serves to synchronize the time across threads. Set by the scheduler thread.
    */
   private long timestamp = Long.MIN_VALUE;

   /**
    * Serves to keep track of skipped ticks. Set by the scheduler thread.
    */
   private long schedulerTick = Long.MIN_VALUE;

   /**
    * The robot measurements. Set by the scheduler thread.
    */
   private final SensorDataContext sensorDataContext;

   /**
    * Serves to inform the controller that the estimator ran and populated the estimated values in this
    * context. Set by the estimator.
    */
   private boolean estimatorRan = false;

   /**
    * Estimated state of the robot. Set by the estimator.
    */
   private final HumanoidRobotContextJointData processedJointData;

   /**
    * The processed force sensor data. Set by the estimator.
    */
   private final ForceSensorDataHolder forceSensorDataHolder;
   /**
    * The processed center of mass state. Set by the estimator.
    */
   private final CenterOfMassDataHolder centerOfMassDataHolder;

   /**
    * Serves to inform the estimator that the controller ran and populated the desired values in this
    * context. Set by the controller.
    */
   private boolean controllerRan = false;
   /**
    * Serves to inform the controller and estimator that the wholeBodyControllerCore ran and populated the desired values in
    * this context. Set by the wholeBodyControllerCore
    */
   private boolean wholeBodyControllerCoreRan = false;
   /**
    * Serves to inform the estimator and controller that the perception ran. Set by the perception.
    */
   private boolean perceptionRan = false;

   /**
    * The controller desired center of pressure. Set by the controller.
    */
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;

   /**
    * The motion status of the robot. Set by the controller.
    */
   private final RobotMotionStatusHolder robotMotionStatusHolder;

   /**
    * The output of the wholebodyControllerCore. Set by the wholebodyControllerCore thread.
    */
   private final ControllerCoreOutPutDataHolder controllerCoreOutPutDataHolder;

   /**
    * The desired joint data to be set on the robot. Set by the controller.
    */
   private final LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;
   /**
    * The output joint data from the WBCC. set by the WholeBodyController.
    * TODO This will be deleted after finishing moving WBCC from controllerThread to WBCCThread.
    */
   private final LowLevelOneDoFJointDesiredDataHolder wholeBodyControllerCoreDesiredOutPutList;
   private final ControllerCoreCommandDataHolder controllerCoreCommandDataHolder;

   public HumanoidRobotContextData()
   {
      processedJointData = new HumanoidRobotContextJointData();
      forceSensorDataHolder = new ForceSensorDataHolder();
      centerOfMassDataHolder = new CenterOfMassDataHolder();
      centerOfPressureDataHolder = new CenterOfPressureDataHolder();
      robotMotionStatusHolder = new RobotMotionStatusHolder();
      jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder();
      sensorDataContext = new SensorDataContext();
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder();
      controllerCoreOutPutDataHolder = new ControllerCoreOutPutDataHolder(null);
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public HumanoidRobotContextData(HumanoidRobotContextJointData processedJointData,
                                   ForceSensorDataHolder forceSensorDataHolder,
                                   CenterOfMassDataHolder centerOfMassDataHolder,
                                   CenterOfPressureDataHolder centerOfPressureDataHolder,
                                   RobotMotionStatusHolder robotMotionStatusHolder,
                                   LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList,
                                   SensorDataContext sensorDataContext,
                                   LowLevelOneDoFJointDesiredDataHolder wbccJointDesiredOutputList,
                                   ControllerCoreCommandDataHolder controllerCoreCommandDataHolder,
                                   ControllerCoreOutPutDataHolder controllerCoreOutPutDataHolder)
   {
      this.processedJointData = processedJointData;
      this.forceSensorDataHolder = forceSensorDataHolder;
      this.centerOfMassDataHolder = centerOfMassDataHolder;
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      this.robotMotionStatusHolder = robotMotionStatusHolder;
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.sensorDataContext = sensorDataContext;
      this.wholeBodyControllerCoreDesiredOutPutList = wbccJointDesiredOutputList;
      this.controllerCoreOutPutDataHolder = controllerCoreOutPutDataHolder;
      this.controllerCoreCommandDataHolder = controllerCoreCommandDataHolder;
   }

   public HumanoidRobotContextData(FullHumanoidRobotModel fullRobotModel)
   {
      processedJointData = new HumanoidRobotContextJointData(fullRobotModel.getOneDoFJoints().length);
      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      centerOfMassDataHolder = new CenterOfMassDataHolder();
      centerOfPressureDataHolder = new CenterOfPressureDataHolder(fullRobotModel);
      robotMotionStatusHolder = new RobotMotionStatusHolder();
      jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      sensorDataContext = new SensorDataContext(fullRobotModel);
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder(fullRobotModel.getControllableOneDoFJoints());
      controllerCoreOutPutDataHolder = new ControllerCoreOutPutDataHolder(fullRobotModel.getControllableOneDoFJoints());
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public HumanoidRobotContextData(List<OneDoFJointBasics> joints)
   {
      processedJointData = new HumanoidRobotContextJointData();
      forceSensorDataHolder = new ForceSensorDataHolder();
      centerOfMassDataHolder = new CenterOfMassDataHolder();
      centerOfPressureDataHolder = new CenterOfPressureDataHolder();
      robotMotionStatusHolder = new RobotMotionStatusHolder();
      jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder(joints.toArray(new OneDoFJointBasics[0]));
      sensorDataContext = new SensorDataContext(joints);
      wholeBodyControllerCoreDesiredOutPutList = new LowLevelOneDoFJointDesiredDataHolder(joints.toArray(new OneDoFJointBasics[0]));
      controllerCoreOutPutDataHolder = new ControllerCoreOutPutDataHolder(joints.toArray(new OneDoFJointBasics[0]));
      controllerCoreCommandDataHolder = new ControllerCoreCommandDataHolder();
   }

   public HumanoidRobotContextJointData getProcessedJointData()
   {
      return processedJointData;
   }

   public ForceSensorDataHolder getForceSensorDataHolder()
   {
      return forceSensorDataHolder;
   }

   public CenterOfMassDataHolder getCenterOfMassDataHolder()
   {
      return centerOfMassDataHolder;
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

   public LowLevelOneDoFJointDesiredDataHolder getWholeBodyControllerCoreDesiredOutPutList()
   {
      return wholeBodyControllerCoreDesiredOutPutList;
   }

   public ControllerCoreOutPutDataHolder getControllerCoreOutPutDataHolder()
   {
      return controllerCoreOutPutDataHolder;
   }

   public SensorDataContext getSensorDataContext()
   {
      return sensorDataContext;
   }

   public ControllerCoreCommandDataHolder getControllerCoreCommandDataHolder()
   {
      return controllerCoreCommandDataHolder;
   }


   @Override
   public void set(HumanoidRobotContextData other)
   {
      copyFrom(other);
   }

   @Override
   public void copyFrom(HumanoidRobotContextData src)
   {
      timestamp = src.timestamp;
      schedulerTick = src.schedulerTick;
      controllerRan = src.controllerRan;
      estimatorRan = src.estimatorRan;
      perceptionRan = src.perceptionRan;
      wholeBodyControllerCoreRan = src.wholeBodyControllerCoreRan;
      processedJointData.set(src.processedJointData);
      forceSensorDataHolder.set(src.forceSensorDataHolder);
      centerOfMassDataHolder.set(src.centerOfMassDataHolder);
      centerOfPressureDataHolder.set(src.centerOfPressureDataHolder);
      robotMotionStatusHolder.set(src.robotMotionStatusHolder);
      jointDesiredOutputList.set(src.jointDesiredOutputList);
      wholeBodyControllerCoreDesiredOutPutList.set(src.wholeBodyControllerCoreDesiredOutPutList);
      sensorDataContext.set(src.sensorDataContext);
      controllerCoreOutPutDataHolder.set(src.controllerCoreOutPutDataHolder);
      controllerCoreCommandDataHolder.setControllerCoreMode(src.controllerCoreCommandDataHolder.getControllerCoreMode());
      controllerCoreCommandDataHolder.set(src.controllerCoreCommandDataHolder);
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public long getSchedulerTick()
   {
      return schedulerTick;
   }

   public void setSchedulerTick(long schedulerTick)
   {
      this.schedulerTick = schedulerTick;
   }

   public void setControllerRan(boolean controllerRan)
   {
      this.controllerRan = controllerRan;
   }

   public boolean getControllerRan()
   {
      return controllerRan;
   }

   public boolean getWholeBodyControllerCoreRan()
   {
      return wholeBodyControllerCoreRan;
   }

   public void setWholeBodyControllerCoreRan(boolean wholeBodyControllerCoreRan)
   {
      this.wholeBodyControllerCoreRan = wholeBodyControllerCoreRan;
   }

   public void setPerceptionRan(boolean perceptionRan)
   {
      this.perceptionRan = perceptionRan;
   }

   public boolean getPerceptionRan()
   {
      return perceptionRan;
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
      else if (obj instanceof HumanoidRobotContextData other)
      {
         if (timestamp != other.timestamp)
            return false;
         if (schedulerTick != other.schedulerTick)
            return false;
         if (controllerRan ^ other.controllerRan)
            return false;
         if (estimatorRan ^ other.estimatorRan)
            return false;
         if (perceptionRan ^ other.perceptionRan)
            return false;
         if (wholeBodyControllerCoreRan ^ other.wholeBodyControllerCoreRan)
            return false;
         if (!processedJointData.equals(other.processedJointData))
            return false;
         if (!forceSensorDataHolder.equals(other.forceSensorDataHolder))
            return false;
         if (!centerOfMassDataHolder.equals(other.centerOfMassDataHolder))
            return false;
         if (!centerOfPressureDataHolder.equals(other.centerOfPressureDataHolder))
            return false;
         if (!robotMotionStatusHolder.equals(other.robotMotionStatusHolder))
            return false;
         if (!jointDesiredOutputList.equals(other.jointDesiredOutputList))
            return false;
         if (!sensorDataContext.equals(other.sensorDataContext))
            return false;
         if (!wholeBodyControllerCoreDesiredOutPutList.equals(other.wholeBodyControllerCoreDesiredOutPutList))
            return false;
         if(!controllerCoreCommandDataHolder.equals(other.controllerCoreCommandDataHolder))
            return false;
         return controllerCoreOutPutDataHolder.equals(other.controllerCoreOutPutDataHolder);
      }
      else
      {
         return false;
      }
   }
}
