package us.ihmc.avatar;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

public class BlockingSimulationRunnerSCS2
{
   private static final long CLOSING_SLEEP_TIME = 1000;
   private final SimulationConstructionSet2 scs2;

   private final double maximumClockRunTimeInSeconds;
   private final boolean destroySimulationIfOverrunMaxTime;

   private final AtomicBoolean hasControllerFailed = new AtomicBoolean(false);
   private final AtomicBoolean hasICPBeenInvalid = new AtomicBoolean(false);

   private boolean checkICPPosition = false;

   public BlockingSimulationRunnerSCS2(SimulationConstructionSet2 scs2, double maximumClockRunTimeInSeconds)
   {
      this(scs2, maximumClockRunTimeInSeconds, true);
   }

   public BlockingSimulationRunnerSCS2(SimulationConstructionSet2 scs2, double maximumClockRunTimeInSeconds, boolean destroySimulationaIfOverrunMaxTime)
   {
      this.scs2 = scs2;

      this.maximumClockRunTimeInSeconds = maximumClockRunTimeInSeconds;
      destroySimulationIfOverrunMaxTime = destroySimulationaIfOverrunMaxTime;
   }

   public void setCheckDesiredICPPosition(boolean checkICPPosition)
   {
      this.checkICPPosition = checkICPPosition;
   }

   public void simulateNTicksAndBlock(int numberOfTicks) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      // TODO: Sometimes you need to sleep before simulating and blocking. Need to fix up the threading stuff in SCS to make more reliable.
      //    if (!hasSleptOnce)
      //    {
      //       hasSleptOnce = true;
      //       sleep(3000);
      //    }

      scs2.simulate(numberOfTicks);

      //    waitForSimulationToStart();
      waitForSimulationToFinish(scs2, maximumClockRunTimeInSeconds, destroySimulationIfOverrunMaxTime);
      checkIfControllerHasFailed();
      if (checkICPPosition)
         checkIfICPHasBeenInvalid();
   }

   public boolean simulateNTicksAndBlockAndCatchExceptions(int numberOfTicks) throws SimulationExceededMaximumTimeException
   {
      try
      {
         simulateNTicksAndBlock(numberOfTicks);
         return true;
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         return false;
      }
   }

   public void simulateAndBlock(double simulateTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      //    System.out.println("Starting Simulation for " + simulateTime);

      // TODO: Sometimes you need to sleep before simulating and blocking. Need to fix up the threading stuff in SCS to make more reliable.

      //    if (!hasSleptOnce)
      //    {
      //       hasSleptOnce = true;
      //       sleep(3000);
      //    }

      double startTime = scs2.getTime().getDoubleValue();
      scs2.simulate(simulateTime);

      //    waitForSimulationToStart();
      waitForSimulationToFinish(scs2, maximumClockRunTimeInSeconds, destroySimulationIfOverrunMaxTime);
      checkIfControllerHasFailed();
      if (checkICPPosition)
         checkIfICPHasBeenInvalid();

      double endTime = scs2.getTime().getDoubleValue();
      double elapsedTime = endTime - startTime;

      if (Math.abs(elapsedTime - simulateTime) > 0.01)
      {
         throw new SimulationExceededMaximumTimeException("Elapsed time didn't equal requested. Sim probably crashed");
      }

      //    System.out.println("Done Simulation for " + simulateTime);

   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      try
      {
         simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         return false;
      }
   }

   public boolean doOneShotRewindTest(double t0, double t1, double t2) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      boolean passed = true;

      // Run to t0 and put a setpoint there.
      sleep(1000);
      simulateAndBlock(t0);
      sleep(1000);

      scs2.setInPoint();
      sleep(1000);

      // Run to t1 and save the state.
      simulateAndBlock(t1 - t0);
      sleep(1000);

      String filenameOne = "Tests/test_" + getTimeString(scs2.getTime()) + ".state";
      scs2.writeState(filenameOne);
      sleep(1000);

      // Run to t2
      simulateAndBlock(t2 - t1);
      sleep(1000);

      // Rewind to t0 and simulate to t1 again.
      scs2.gotoInPointNow();
      sleep(1000);

      simulateAndBlock(t1 - t0);
      sleep(1000);

      // Save the state again.
      String filenameTwo = "Tests/test_" + getTimeString(scs2.getTime()) + "_Rewind.state";
      scs2.writeState(filenameTwo);
      sleep(1000);

      // Now compare the two:

      double maxPercentDiff = 0.001;
      ArrayList<VariableDifference> changedVariables = StateFileComparer.percentualCompareStateFiles(filenameOne, filenameTwo, maxPercentDiff, null);

      if (changedVariables.size() > 0)
      {
         System.err.println("Difference between " + filenameOne + " and " + filenameTwo);
         passed = false;
      }

      return passed;
   }

   private static void sleep(long sleepTimeMillis)
   {
      try
      {
         Thread.sleep(sleepTimeMillis);
      }
      catch (InterruptedException ex)
      {
      }
   }

   private static String getTimeString(double time)
   {
      String ret = Double.toString(time);

      ret = ret.substring(0, Math.min(8, ret.length() - 1));

      return ret;
   }

   public void destroySimulation()
   {
      destroySimulation(scs2);
   }

   private static void destroySimulation(SimulationConstructionSet2 scs)
   {
      ThreadTools.sleep(CLOSING_SLEEP_TIME);
      scs.setShutdownSessionOnVisualizerClose(true);
      scs = null;
   }

   public static void waitForSimulationToFinish(SimulationConstructionSet2 scs, double maximumClockRunTimeInSeconds, boolean destroySimulationaIfOverrunMaxTime)
         throws SimulationExceededMaximumTimeException
   {
      long startTime = System.currentTimeMillis();

      while (scs.isSimulating())
      {
         sleep(100);

         long currentTime = System.currentTimeMillis();
         double elapsedTime = (currentTime - startTime) * 0.001;

         if (elapsedTime > maximumClockRunTimeInSeconds)
         {
            scs.stopSimulationThread();
            if (destroySimulationaIfOverrunMaxTime)
               destroySimulation(scs);

            throw new SimulationExceededMaximumTimeException("Simulation Exceeded maximumClockRunTimeInSeconds");
         }
      }
   }

   private void checkIfControllerHasFailed() throws ControllerFailureException
   {
      if (hasControllerFailed.get())
         throw new ControllerFailureException("Controller failure has been detected.");
   }

   private void checkIfICPHasBeenInvalid() throws ControllerFailureException
   {
      if (hasICPBeenInvalid.get())
      {
         throw new ControllerFailureException("The desired ICP position has been invalid.");
      }
   }

   public void notifyControllerHasFailed()
   {
      hasControllerFailed.set(true);
      scs2.stop();
   }

   public void createValidDesiredICPListener()
   {
      YoDouble desiredICPX = (YoDouble) scs2.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) scs2.findVariable("desiredICPY");

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (!Double.isFinite(v.getValueAsDouble()))
               hasICPBeenInvalid.set(true);
         }
      });
      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (!Double.isFinite(v.getValueAsDouble()))
               hasICPBeenInvalid.set(true);
         }
      });
   }

   public static class SimulationExceededMaximumTimeException extends Exception
   {
      private static final long serialVersionUID = 9041559998947724357L;

      public SimulationExceededMaximumTimeException(String description)
      {
         super(description);
      }
   }
}
