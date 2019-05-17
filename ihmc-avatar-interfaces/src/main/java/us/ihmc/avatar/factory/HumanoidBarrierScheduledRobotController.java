package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.Task;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public class HumanoidBarrierScheduledRobotController extends BarrierScheduledRobotController<HumanoidRobotContextData>
{
   private final SensorReader sensorReader;
   private final HumanoidRobotContextData masterContext;

   public HumanoidBarrierScheduledRobotController(String name, List<? extends Task<HumanoidRobotContextData>> tasks, HumanoidRobotContextData masterContext,
                                                  TaskOverrunBehavior overrunBehavior, SensorReader sensorReader)
   {
      super(name, tasks, masterContext, overrunBehavior);
      this.sensorReader = sensorReader;
      this.masterContext = masterContext;
   }

   @Override
   public void doControl()
   {
      // Reads the sensor data from the robot into the master context.
      long timestamp = sensorReader.read(masterContext.getSensorDataContext());
      masterContext.setTimestamp(timestamp);

      super.doControl();
   }
}
