package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class EstimatorTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver estimatorResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarEstimatorThread estimatorThread;
   private final YoLong estimatorTick;

   public EstimatorTask(AvatarEstimatorThread estimatorThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.estimatorThread = estimatorThread;
      estimatorResolver = new CrossRobotCommandResolver(estimatorThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);
      estimatorTick = new YoLong("EstimatorTick", estimatorThread.getYoVariableRegistry());
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
      estimatorTick.increment();
      estimatorThread.read(System.nanoTime());
      estimatorThread.run();
      estimatorThread.write();
   }

   @Override
   protected void cleanup()
   {
      estimatorThread.dispose();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      masterResolver.resolveHumanoidRobotContextDataEstimatorToController(estimatorThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      estimatorResolver.resolveHumanoidRobotContextDataControllerToEstimator(masterContext, estimatorThread.getHumanoidRobotContextData());
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return estimatorThread.getYoVariableRegistry();
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return estimatorThread.getYoGraphicsListRegistry();
   }

}
