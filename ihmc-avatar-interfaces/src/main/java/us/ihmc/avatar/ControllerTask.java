package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ThreadTimer;

public class ControllerTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThread controllerThread;

   private final RobotVisualizer robotVisualizer;

   private final ThreadTimer timer;

   public ControllerTask(AvatarControllerThread controllerThread, long divisor, FullHumanoidRobotModel masterFullRobotModel, RobotVisualizer robotVisualizer)
   {
      super(divisor);
      this.controllerThread = controllerThread;
      this.robotVisualizer = robotVisualizer;

      controllerResolver = new CrossRobotCommandResolver(controllerThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);

      timer = new ThreadTimer("Controller", controllerThread.getYoVariableRegistry());

      robotVisualizer.addRegistry(controllerThread.getYoVariableRegistry(), controllerThread.getYoGraphicsListRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      controllerThread.run();
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      controllerThread.write();
      masterResolver.resolveHumanoidRobotContextDataController(controllerThread.getHumanoidRobotContextData(), masterContext);
      robotVisualizer.update(masterContext.getTimestamp(), controllerThread.getYoVariableRegistry());
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataEstimator(masterContext, controllerThread.getHumanoidRobotContextData());
      controllerThread.read();
   }

}
