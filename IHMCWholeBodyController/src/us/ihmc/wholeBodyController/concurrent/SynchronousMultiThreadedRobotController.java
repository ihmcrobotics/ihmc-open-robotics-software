package us.ihmc.wholeBodyController.concurrent;

import java.util.ArrayList;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.thread.ThreadTools;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class SynchronousMultiThreadedRobotController implements MultiThreadedRobotControlElementCoordinator
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final TimestampProvider timestampProvider;
   private final ArrayList<ReentrantLockedControlElementRunner> robotControllerRunners = new ArrayList<>();
   private final ThreadFactory namedThreadFactory;

   public SynchronousMultiThreadedRobotController(MultiThreadedRobotControlElement sensorReader, TimestampProvider timestampProvider)
   {
      this.sensorReader = sensorReader;
      this.timestampProvider = timestampProvider;
      namedThreadFactory = ThreadTools.getNamedThreadFactory(this.getClass().getSimpleName());
   }

   public void addController(MultiThreadedRobotControlElement robotController, int sensorReaderTicksPerControlTick)
   {
      robotControllerRunners.add(new ReentrantLockedControlElementRunner(robotController, this.timestampProvider, sensorReaderTicksPerControlTick));
   }

   public void read()
   {
      sensorReader.read(timestampProvider.getTimestamp());
      sensorReader.run();
      sensorReader.write(timestampProvider.getTimestamp());

      for(int i = 0; i < robotControllerRunners.size(); i++)
      {
         robotControllerRunners.get(i).tick();
         robotControllerRunners.get(i).runIfReady();
      }
   }

   public void start()
   {
      for (int i = 0; i < robotControllerRunners.size(); i++)
      {
         ReentrantLockedControlElementRunner runner = robotControllerRunners.get(i);

         runner.getController().initialize();

         namedThreadFactory.newThread(runner).start();
      }
   }

   private class ReentrantLockedControlElementRunner implements Runnable
   {
      private final MultiThreadedRobotControlElement controller;
      private final ReentrantLock reentrantLock = new ReentrantLock();
      private final Condition shouldRunCondition;
      private final Condition isRunningCondition;
      private final TimestampProvider timestampProvider;
      private final int ticksPerExecution;
      private int ticks;

      private ReentrantLockedControlElementRunner(MultiThreadedRobotControlElement controller, TimestampProvider timestampProvider, int ticksPerExecution)
      {
         this.controller = controller;
         this.timestampProvider = timestampProvider;
         this.ticksPerExecution = ticksPerExecution;
         this.ticks = 0;

         shouldRunCondition = reentrantLock.newCondition();
         isRunningCondition = reentrantLock.newCondition();
      }

      @Override
      public void run()
      {
         while (true)
         {
            reentrantLock.lock();
            try
            {
               controller.read(timestampProvider.getTimestamp());
               controller.run();
               controller.write(timestampProvider.getTimestamp());
               isRunningCondition.signalAll();
               shouldRunCondition.await();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
            finally
            {
               reentrantLock.unlock();
            }
         }
      }

      private void tick()
      {
         reentrantLock.lock();
         try
         {
            this.ticks++;
         }
         finally
         {
            reentrantLock.unlock();
         }
      }

      private void runIfReady()
      {
         reentrantLock.lock();
         try
         {
            if (this.ticks >= this.ticksPerExecution)
            {
               this.ticks = 0;
               shouldRunCondition.signalAll();
               isRunningCondition.await();
            }
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
         finally
         {
            reentrantLock.unlock();
         }
      }

      private MultiThreadedRobotControlElement getController()
      {
         return controller;
      }
   }
}
