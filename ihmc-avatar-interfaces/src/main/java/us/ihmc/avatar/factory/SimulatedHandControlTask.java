package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.yoVariables.variable.YoLong;

public class SimulatedHandControlTask extends HumanoidRobotControlTask
{
   private final SimulatedHandSensorReader handSensorReader;
   private final AvatarSimulatedHandControlThread handControlThread;
   private final SimulatedHandOutputWriter handOutputWriter;

   private final long divisor;
   private final ThreadTimer timer;
   private final YoLong ticksBehindScheduled;

   private final List<Runnable> postControllerCallback = new ArrayList<>();
   private final List<Runnable> schedulerThreadRunnables = new ArrayList<>();

   private boolean controllerRan = false;

   public SimulatedHandControlTask(SimulatedHandSensorReader handSensorReader,
                                   AvatarSimulatedHandControlThread handControllerThread,
                                   SimulatedHandOutputWriter handOutputWriter,
                                   long divisor,
                                   double schedulerDt)
   {
      super(divisor);
      this.handSensorReader = handSensorReader;
      this.handOutputWriter = handOutputWriter;
      this.divisor = divisor;
      this.handControlThread = handControllerThread;

      String prefix = "HandController";
      timer = new ThreadTimer(prefix, schedulerDt * divisor, handControllerThread.getYoVariableRegistry());
      ticksBehindScheduled = new YoLong(prefix + "TicksBehindScheduled", handControllerThread.getYoVariableRegistry());
   }

   @Override
   protected void execute()
   {
      timer.start();
      long schedulerTick = handControlThread.getHumanoidRobotContextData().getSchedulerTick();
      ticksBehindScheduled.set(schedulerTick - timer.getTickCount() * divisor);
      handControlThread.run();
      runAll(postControllerCallback);
      controllerRan = handControlThread.hasControllerRan();
      timer.stop();
   }

   @Override
   protected void updateMasterContext(HumanoidRobotContextData masterContext)
   {
      if (!controllerRan)
         return;
      runAll(schedulerThreadRunnables);
      handOutputWriter.write(handControlThread.getHumanoidRobotContextData().getJointDesiredOutputList());
   }

   @Override
   protected void updateLocalContext(HumanoidRobotContextData masterContext)
   {
      handControlThread.getHumanoidRobotContextData().setTimestamp(masterContext.getTimestamp());
      handControlThread.getHumanoidRobotContextData().setSchedulerTick(masterContext.getSchedulerTick());
      handSensorReader.read(handControlThread.getHumanoidRobotContextData().getSensorDataContext());
      handControlThread.getHumanoidRobotContextData().setEstimatorRan(masterContext.getEstimatorRan());
      handControlThread.getHumanoidRobotContextData().setControllerRan(masterContext.getControllerRan());
   }

   @Override
   public void addCallbackPostTask(Runnable runnable)
   {
      postControllerCallback.add(runnable);
   }

   @Override
   public void addRunnableOnSchedulerThread(Runnable runnable)
   {
      schedulerThreadRunnables.add(runnable);
   }

   @Override
   protected void cleanup()
   {
      super.cleanup();
      handControlThread.cleanup();
   }
}
