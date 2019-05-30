package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;
   private final SensorReader sensorReader;

   private final ThreadTimer timer;

   private final List<Runnable> taskThreadRunnables = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   private boolean sensorsRead = false;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;
      this.sensorReader = estimatorThread.getSensorReader();

      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer("Estimator", estimatorThread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      if (!sensorsRead)
         return;

      timer.start();
      estimatorThread.read();
      estimatorThread.run();
      estimatorThread.write();
      runAll(taskThreadRunnables);
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      runAll(schedulerThreadRunnables);
      masterResolver.resolveHumanoidRobotContextDataEstimator(estimatorThread.getHumanoidRobotContextData(), masterContext);

      // Abuse the fact that this is running on the robot synchronized thread to update the sensor data and the time.
      long newTimestamp = sensorReader.read(masterContext.getSensorDataContext());
      masterContext.setTimestamp(newTimestamp);
      sensorsRead = true;
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      estimatorResolver.resolveHumanoidRobotContextDataScheduler(masterContext, estimatorThread.getHumanoidRobotContextData());
      estimatorResolver.resolveHumanoidRobotContextDataController(masterContext, estimatorThread.getHumanoidRobotContextData());
   }

   @Override
   public void addRunnableOnTaskThread(Runnable runnable)
   {
      taskThreadRunnables.add(runnable);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }

}
