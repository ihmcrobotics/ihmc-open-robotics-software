package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class ControllerTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThread controllerThread;
   private final YoLong controllerTick;

   public ControllerTask(AvatarControllerThread controllerThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.controllerThread = controllerThread;
      controllerResolver = new CrossRobotCommandResolver(controllerThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);
      controllerTick = new YoLong("ControllerTick", controllerThread.getYoVariableRegistry());
   }

   @Override
   protected boolean initialize()
   {
      controllerThread.initialize();
      return true;
   }

   @Override
   protected void execute()
   {
      controllerTick.increment();
      controllerThread.read(System.nanoTime());
      controllerThread.run();
      controllerThread.write();
   }

   @Override
   protected void cleanup()
   {
      controllerThread.dispose();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      masterResolver.resolveHumanoidRobotContextDataControllerToEstimator(controllerThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataEstimatorToController(masterContext, controllerThread.getHumanoidRobotContextData());
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return controllerThread.getYoVariableRegistry();
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return controllerThread.getYoGraphicsListRegistry();
   }

}
