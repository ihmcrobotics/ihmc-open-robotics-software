package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;

   private final RobotVisualizer robotVisualizer;

   private final ThreadTimer timer;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel, RobotVisualizer robotVisualizer)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;
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
      timer.start();
      estimatorThread.run();
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      estimatorThread.write();
      masterResolver.resolveHumanoidRobotContextDataEstimatorToController(estimatorThread.getHumanoidRobotContextData(), masterContext);
      robotVisualizer.update(masterContext.getTimestamp(), estimatorThread.getYoVariableRegistry());
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      estimatorResolver.resolveHumanoidRobotContextDataControllerToEstimator(masterContext, estimatorThread.getHumanoidRobotContextData());
      estimatorThread.read();
   }

}
