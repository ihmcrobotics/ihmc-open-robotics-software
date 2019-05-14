package us.ihmc.avatar;

import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class ControllerTask extends HumanoidRobotControlTask
{
   private final CrossRobotCommandResolver controllerResolver;
   private final CrossRobotCommandResolver masterResolver;

   private final AvatarControllerThread controllerThread;
   private final YoLong controllerTick;
   private final YoDouble controllerDT;
   private final YoDouble controllerTimer;

   private long lastStartTime;

   private final MirroredYoVariableRegistry registry;

   public ControllerTask(AvatarControllerThread controllerThread, long divisor, FullHumanoidRobotModel masterFullRobotModel)
   {
      super(divisor);
      this.controllerThread = controllerThread;
      controllerResolver = new CrossRobotCommandResolver(controllerThread.getFullRobotModel());
      masterResolver = new CrossRobotCommandResolver(masterFullRobotModel);
      controllerTick = new YoLong("ControllerTick", controllerThread.getYoVariableRegistry());
      controllerDT = new YoDouble("ControllerDT", controllerThread.getYoVariableRegistry());
      controllerTimer = new YoDouble("ControllerTimer", controllerThread.getYoVariableRegistry());

      registry = new MirroredYoVariableRegistry(controllerThread.getYoVariableRegistry());
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
      long startTime = System.nanoTime();
      if (lastStartTime != 0)
         controllerDT.set(Conversions.nanosecondsToMilliseconds((double) (startTime - lastStartTime)));
      lastStartTime = startTime;

      controllerTick.increment();
      controllerThread.run();

      controllerTimer.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - lastStartTime)));
   }

   @Override
   protected void cleanup()
   {
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      controllerThread.write();
      // TODO: do the robot visualizer update here.
      registry.updateMirror();
      masterResolver.resolveHumanoidRobotContextDataControllerToEstimator(controllerThread.getHumanoidRobotContextData(), masterContext);
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      controllerResolver.resolveHumanoidRobotContextDataEstimatorToController(masterContext, controllerThread.getHumanoidRobotContextData());
      controllerThread.read();
   }

   @Override
   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return controllerThread.getYoGraphicsListRegistry();
   }

}
