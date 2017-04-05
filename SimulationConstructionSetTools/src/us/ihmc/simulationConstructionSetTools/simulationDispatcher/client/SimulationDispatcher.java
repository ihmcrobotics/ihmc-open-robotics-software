package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

import java.net.MalformedURLException;
import java.rmi.Naming;
import java.rmi.NotBoundException;
import java.rmi.RMISecurityManager;
import java.rmi.RemoteException;
import java.rmi.registry.LocateRegistry;
import java.security.Permission;
import java.util.ArrayList;

import us.ihmc.simulationConstructionSetTools.simulationDispatcher.client.gui.SimulationDispatcherGUI;
import us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;

public class SimulationDispatcher implements Runnable
{
   private final String password = "***REMOVED***";
   private final DispatchHostList dispatchHostList;

   private final ArrayList<SimulationToDispatch> simulationsToDispatch = new ArrayList<SimulationToDispatch>();
   private final ArrayList<RemoteSimulationPair> remoteSimulationPairs = new ArrayList<RemoteSimulationPair>();
   private final ArrayList<SimulationToDispatch> doneSimulations = new ArrayList<SimulationToDispatch>();

   private final ArrayList<SimulationsChangedListener> simulationsChangedListeners = new ArrayList<SimulationsChangedListener>();

   public synchronized int getNumberSimulationsToDispatch()
   {
      return simulationsToDispatch.size();
   }

   public synchronized SimulationToDispatch[] getSimulationsToDispatch()
   {
      SimulationToDispatch[] ret = new SimulationToDispatch[simulationsToDispatch.size()];

      simulationsToDispatch.toArray(ret);

      return ret;
   }

   public synchronized SimulationToDispatch[] getDoneSimulations()
   {
      SimulationToDispatch[] ret = new SimulationToDispatch[doneSimulations.size()];

      doneSimulations.toArray(ret);

      return ret;
   }

   public synchronized int getNumberSimulationsCurrentlyDispatched()
   {
      return remoteSimulationPairs.size();
   }


   public SimulationDispatcher(String[] hostNames, String myCodeBase)
   {
      RMISecurityManager laxManager = new RMISecurityManager()
      {
         @Override
         public void checkPermission(Permission p)
         {
            // System.out.println(p);
            // try
            // {super.checkPermission(p);}
            // catch(Exception e)
            {
               // System.out.println(p);
            }
         }
      };


      System.setSecurityManager(laxManager);    // new RMISecurityManager());
      dispatchHostList = new DispatchHostList(this.password);

      for (int i = 0; i < hostNames.length; i++)
      {
         String hostName = hostNames[i];
         int indexOfColon = hostName.indexOf(':');

         String justTheName = hostName.substring(0, indexOfColon);

         int numberOfThreads = Integer.valueOf(hostName.substring(indexOfColon + 1));

         for (int j = 0; j < numberOfThreads; j++)
         {
            dispatchHostList.addHost(justTheName + ":" + j);

         }
      }

      System.out.println("Creating an RMI Registry");

      try
      {
         LocateRegistry.createRegistry(1099);
      }
      catch (RemoteException e)
      {
         System.out.println("Registry already running");
      }


      String codebase = System.getProperty("java.rmi.server.codebase");
      if (codebase == "null")
         codebase = "";

      if (myCodeBase != null)
         codebase = codebase + myCodeBase;

      System.out.println("Setting the RMI codebase to " + codebase);
      System.setProperty("java.rmi.server.codebase", codebase);

      System.out.println("Registering the RemoteIPRegistryServer to my local RMI Registry...");

      try
      {
         RemoteIPRegistryServer server = new RemoteIPRegistryServer(dispatchHostList, this.password);
         Naming.rebind("RemoteIPRegistry", server);
      }
      catch (Exception e)
      {
         System.out.println("Couldn't register the DispatchHostList on my local RMI Registry. Make sure that it is started..." + e.getMessage());
         e.printStackTrace();
      }

      new SimulationDispatcherGUI(this, dispatchHostList, this.password);

      Thread anim = new Thread(this);
      anim.start();
   }

   public synchronized void addSimulationsChangedListener(SimulationsChangedListener listener)
   {
      this.simulationsChangedListeners.add(listener);
   }

   private synchronized void notifyListeners()
   {
      for (int i = 0; i < simulationsChangedListeners.size(); i++)
      {
         ((SimulationsChangedListener) simulationsChangedListeners.get(i)).simulationsChanged();
      }
   }

   private Throwable getRootCause(Throwable throwable)
   {
      while (throwable.getCause() != null)
      {
         throwable = throwable.getCause();
      }

      return throwable;
   }

   @Override
   public void run()
   {
      int doneIndex = 0;

      while (true)
      {
         synchronized (this)
         {
            // Check on the simulations that need to be dispatched and dispatch them:
            if (this.simulationsToDispatch.size() > 0)
            {
               if (dispatchHostList.isHostReady())
               {
                  DispatchHost dispatchHost = dispatchHostList.getNextAvailableDispatchHost();    // This returns null if there is none
                  SimulationToDispatch dispatchSim = simulationsToDispatch.get(0);

                  if (dispatchHost != null)
                  {
                     try
                     {
                        dispatchOneSimulation(dispatchSim, dispatchHost.getRemoteSim());

                        synchronized (this)
                        {
                           remoteSimulationPairs.add(new RemoteSimulationPair(dispatchSim, dispatchHost));
                        }

                        simulationsToDispatch.remove(dispatchSim);
                        dispatchHost.reportSimulationStarted(dispatchSim);
                        this.notifyListeners();
                     }
                     catch (Exception exception)
                     {
                        Throwable rootCause = getRootCause(exception);

                        System.out.println("Host " + dispatchHost.getHostName() + ": " + dispatchHost.getHostProcess() + " is broken!!! \n Root cause = "
                                           + rootCause);

                        dispatchHost.reportBroken();

//                      rootCause.printStackTrace();

                        dispatchHostList.checkInDispatchHost(dispatchHost);
                        simulationsToDispatch.add(dispatchSim);
                        this.notifyListeners();
                     }
                  }
               }
            }


            // Check on the simulations that are running remotely and see if they are done or dead.
            if (remoteSimulationPairs.size() > 0)
            {
               if (doneIndex >= remoteSimulationPairs.size())
                  doneIndex = 0;
               RemoteSimulationPair remotePair = (RemoteSimulationPair) remoteSimulationPairs.get(doneIndex);
               SimulationToDispatch dispatchSim = remotePair.simulationToDispatch;
               DispatchHost dispatchHost = remotePair.dispatchHost;

               try
               {
                  if (dispatchHost.getRemoteSim().isSimulationDone(this.password))
                  {
                     DispatchDoneListener listener = dispatchSim.getDispatchDoneListener();
                     double[] finalState = (double[]) dispatchHost.getRemoteSim().getSimulationState(this.password);
                     dispatchSim.setFinalState(finalState);

                     if (listener != null)
                     {
//                      System.out.println("Dispatch Done. Notifying Listener!");
                        listener.dispatchDone(dispatchSim, finalState);
                     }

                     dispatchHost.reportSimulationFinished();

                     remoteSimulationPairs.remove(remotePair);

                     // System.out.println("Remote Simulation Completed Successfully and is being removed from the list of running simulations!!!");

                     dispatchSim.setSimulationFinished();
                     doneSimulations.add(dispatchSim);

                     dispatchHostList.checkInDispatchHost(dispatchHost);
                     this.notifyListeners();
                  }

                  // else{System.out.println("Sim not done yet...");}
               }
               catch (Exception e)
               {
                  System.out.println(
                      "Oops!! Simulation Died after being put on a server successfully.  Need to put it back in the list of sims to run and return the host");
                  remoteSimulationPairs.remove(remotePair);
                  simulationsToDispatch.add(remotePair.simulationToDispatch);
                  dispatchHostList.checkInDispatchHost(dispatchHost);
                  this.notifyListeners();
               }
            }
         }

         doneIndex++;

         try
         {
            Thread.sleep(500);
         }
         catch (InterruptedException e)
         {
         }

      }

   }

   public synchronized void addSimulation(SimulationToDispatch sim)
   {
      System.out.println("Dispatching Simulation: " + sim.getDescription());
      simulationsToDispatch.add(sim);
      this.notifyListeners();
   }


   private void dispatchOneSimulation(SimulationToDispatch dispatchSim, RemoteSimulationRunnerInterface remoteSim)
           throws RemoteException, MalformedURLException, NotBoundException
   {
      DispatchedSimulationDescription description = new DispatchedSimulationDescription(dispatchSim.getConstructor(), dispatchSim.getInputStateVariableNames(),
                                                       dispatchSim.getOutputStateVariableNames());

      // System.out.println("Creating the Simulation."); System.out.flush();
      remoteSim.createSimulation(description, dispatchSim.getStructuralParameterNames(), dispatchSim.getStructuralParameterValues(), this.password);

      // System.out.println("Setting the Simulation State."); System.out.flush();
      remoteSim.setSimulationState(dispatchSim.getInputState(), this.password);

      // System.out.println("Running the Simulation."); System.out.flush();
      remoteSim.startSimulation(this.password);
   }


   public class RemoteSimulationPair
   {
      private SimulationToDispatch simulationToDispatch;
      private DispatchHost dispatchHost;

      public RemoteSimulationPair(SimulationToDispatch simulationToDispatch, DispatchHost dispatchHost)
      {
         this.simulationToDispatch = simulationToDispatch;
         this.dispatchHost = dispatchHost;
      }
   }



}
