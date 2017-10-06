package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

import java.rmi.Naming;

import us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;

public class DispatchHost
{
   private final String hostName;
   private final String hostProcess;

   private final String remoteSimulationRunnerName;

   private RemoteSimulationRunnerInterface remoteSim;
   private boolean isBroken = false;
   private SimulationToDispatch dispatchSim;

   private int NumberOfSimulationsRun = 0;
   private double approxTimePerSimulation = 60.0;
   private double lastSimulationTime = 60.0;

   private long inTime = System.currentTimeMillis();

   private boolean isRunningSim = false;
   private String password;

   public DispatchHost(String hostName, String password)
   {
      this.password = password;

      int separatorIndex = hostName.indexOf(":");

      if (separatorIndex < 0)
      {
         this.hostName = hostName;
         this.hostProcess = "0";
      }

      else
      {
         this.hostName = hostName.substring(0, separatorIndex);
         this.hostProcess = hostName.substring(separatorIndex + 1, hostName.length());
      }

      System.out.println("hostName = " + hostName + ", this.hostName = " + this.hostName + ", hostProcess = " + hostProcess);
      this.remoteSimulationRunnerName = "RemoteSimulationRunner" + hostProcess;
      lookUpRemoteSimulationRunner();

   }

   public String getHostName()
   {
      return hostName;
   }

   public String getHostProcess()
   {
      return hostProcess;
   }

   public void reportBroken()
   {
      remoteSim = null;
      isRunningSim = false;
      isBroken = true;
   }


   public int getNumberOfSimulationsRun()
   {
      return this.NumberOfSimulationsRun;
   }

   public int getApporxTimePerSimulation()
   {
      return (int) this.approxTimePerSimulation;
   }

   public int getlastSimulationTime()
   {
      return (int) this.lastSimulationTime;
   }

   public double getTimeSinceLastSimulationStarted()
   {
      long diffTimeMillis = System.currentTimeMillis() - inTime;

      return ((double) diffTimeMillis) / 1000.0;
   }


   public void reportSimulationStarted(SimulationToDispatch dispatchSim)
   {
      this.dispatchSim = dispatchSim;
      inTime = System.currentTimeMillis();
      isRunningSim = true;
   }

   private static final double alpha = 0.9;

   public void reportSimulationFinished()
   {
      NumberOfSimulationsRun++;
      isRunningSim = false;

      lastSimulationTime = getTimeSinceLastSimulationStarted();
      approxTimePerSimulation = alpha * approxTimePerSimulation + (1.0 - alpha) * lastSimulationTime;

      // System.out.println(lastSimulationTime + " seconds. " + hostName + " has run " + NumberOfSimulationsRun + " simulations at an approximate time of " + approxTimePerSimulation + " seconds per simulation");
      inTime = System.currentTimeMillis();
   }


   private Thread lookupThread = new Thread("Dispatch Host Lookup")
   {
      @Override
      public void run()
      {
         while (true)
         {
            if (!isBroken)
            {
               try
               {
                  // System.out.println("Looking up RemoteSimulationRunner for host: " + hostName); System.out.flush();
//                remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//" + hostName + "/RemoteSimulationRunnerGoob");
                  remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//" + hostName + "/" + remoteSimulationRunnerName);
                  System.out.println("Found RemoteSimulationRunner on Host: " + hostName);
                  System.out.flush();
                  pingRemoteSimulationRunner();

                  if (!remoteSim.isSimulationDone(password))
                  {
                     System.err.println(hostName + " already seems to be running a simulation");
                  }
               }
               catch (Exception e)
               {
                  remoteSim = null;
               }
            }

            try
            {
               Thread.sleep(600000);
            }    // Sleep 10 minutes until interrupted
            catch (InterruptedException e)
            {
            }
         }
      }
   };

   private synchronized void lookUpRemoteSimulationRunner()
   {
      if (!lookupThread.isAlive())
         lookupThread.start();
      else
         lookupThread.interrupt();

      /*
       * try
       * {
       * System.out.println("Looking up RemoteSimulationRunner for host: " + hostName); System.out.flush();
       * remoteSim = (RemoteSimulationRunnerInterface) Naming.lookup("//" + hostName + "/RemoteSimulationRunner");
       * System.out.println("Found RemoteSimulationRunner on Host: " + hostName); System.out.flush();
       * pingRemoteSimulationRunner();
       * }
       * catch(Exception e) {remoteSim = null;}
       */
   }

   private synchronized void pingRemoteSimulationRunner()
   {
      try
      {
         remoteSim.ping(password);

         // System.out.println(hostName + " pinged successfully!");
      }
      catch (Exception e)
      {
         System.out.println("Remote Simulation Runner on host " + hostName + "couldn't be pinged!!");
         remoteSim = null;
         isRunningSim = false;
      }
   }

   public RemoteSimulationRunnerInterface getRemoteSim()
   {
      return remoteSim;
   }

   public SimulationToDispatch getDispatchedSim()
   {
      return dispatchSim;
   }

   public boolean isAlive()
   {
      return (remoteSim != null);
   }

   public boolean isRunningSim()
   {
      return this.isRunningSim;
   }

   protected void setIsRunningSim(boolean isRunningSim)
   {
      this.isRunningSim = isRunningSim;
   }

   public void pingAndOrFind()
   {
      if (remoteSim != null)
      {
         pingRemoteSimulationRunner();
      }
      else
      {
         lookUpRemoteSimulationRunner();
      }
   }
}
