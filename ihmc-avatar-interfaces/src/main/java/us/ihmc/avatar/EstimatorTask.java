package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;
   private final YoLong estimatorTick;
   private final YoDouble estimatorDT;
   private final YoDouble estimatorTimer;

   private long lastStartTime;

   private final RobotVisualizer robotVisualizer;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel, RobotVisualizer robotVisualizer)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;
      this.robotVisualizer = robotVisualizer;

      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);
      estimatorTick = new YoLong("EstimatorTick", estimatorThread.getYoVariableRegistry());
      estimatorDT = new YoDouble("EstimatorDT", estimatorThread.getYoVariableRegistry());
      estimatorTimer = new YoDouble("EstimatorTimer", estimatorThread.getYoVariableRegistry());

      robotVisualizer.setMainRegistry(estimatorThread.getYoVariableRegistry(), estimatorThread.getFullRobotModel().getElevator(),
                                      estimatorThread.getYoGraphicsListRegistry());
   }

   @Override
   protected boolean initialize()
   {
      estimatorThread.initialize();
      return true;
   }

   @Override
   protected void execute()
   {
      long startTime = System.nanoTime();
      if (lastStartTime != 0)
         estimatorDT.set(Conversions.nanosecondsToMilliseconds((double) (startTime - lastStartTime)));
      lastStartTime = startTime;

      estimatorTick.increment();
      estimatorThread.run();

      estimatorTimer.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - lastStartTime)));
   }

   @Override
   protected void cleanup()
   {
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
