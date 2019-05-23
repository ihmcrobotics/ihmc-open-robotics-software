package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;
   private final SensorReader sensorReader;

   private final RobotVisualizer robotVisualizer;

   private final ThreadTimer timer;

   private boolean sensorsRead = false;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel, RobotVisualizer robotVisualizer)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;
      this.sensorReader = estimatorThread.getSensorReader();
      this.robotVisualizer = robotVisualizer;

      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer("Estimator", estimatorThread.getYoVariableRegistry());

      robotVisualizer.setMainRegistry(estimatorThread.getYoVariableRegistry(), estimatorThread.getFullRobotModel().getElevator(),
                                      estimatorThread.getYoGraphicsListRegistry());
   }

   @Override
   protected void execute()
   {
      if (!sensorsRead)
         return;

      timer.start();
      estimatorThread.run();
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      estimatorThread.write();
      masterResolver.resolveHumanoidRobotContextDataEstimator(estimatorThread.getHumanoidRobotContextData(), masterContext);

      // Abuse the fact that this is running on the scheduler thread to safely update the robot visualizer.
      robotVisualizer.update(masterContext.getTimestamp(), estimatorThread.getYoVariableRegistry());

      // Abuse the fact that this is running on the robot synchronized thread to update the sensor data and the time.
      long newTimestamp = sensorReader.read(masterContext.getSensorDataContext());
      masterContext.setTimestamp(newTimestamp);
      sensorsRead = true;

      estimatorThread.read();
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      estimatorResolver.resolveHumanoidRobotContextDataScheduler(masterContext, estimatorThread.getHumanoidRobotContextData());
      estimatorResolver.resolveHumanoidRobotContextDataController(masterContext, estimatorThread.getHumanoidRobotContextData());
   }

}
