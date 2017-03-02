package us.ihmc.simulationconstructionset.simulationDispatcher.server;

import java.net.InetAddress;
import java.rmi.Naming;
import java.rmi.RMISecurityManager;
import java.rmi.RemoteException;
import java.rmi.registry.LocateRegistry;
import java.rmi.registry.Registry;
import java.rmi.server.UnicastRemoteObject;

import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationDescription;
import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;


public class RemoteSimulationRunner extends UnicastRemoteObject implements RemoteSimulationRunnerInterface
{
   private static final long serialVersionUID = 9053419694765471403L;

   private static boolean DEBUG = false;

   private static final boolean SET_CODEBASE = true;    // true;

   // String codebase = System.getProperty("java.rmi.server.codebase");
   // if (codebase == "null"); codebase = "";

   // String codebase = "http://www.coginst.uwf.edu/~jpratt/SimulationDispatcher/codebase/simulationdispatcherinterfaces.jar http://www.coginst.uwf.edu/~jpratt/SimulationDispatcher/codebase/simulationdispatcherserver.jar";
// String codebase = "http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationdispatcherinterfaces.jar http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationdispatcherserver.jar";
// String codebase = ""; //http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationdispatcherinterfaces.jar http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationdispatcherserver.jar";

//   private final static String codebase =
//      "file://halo/Public/Projects/SimulationDispatcher/SimulationDispatcherInterfaces.jar file://halo/Public/Projects/SimulationDispatcher/SimulationDispatcherClient.jar file://halo/Public/Projects/SimulationDispatcher/SimulationDispatcherServer.jar";


   private final static String codebase = "https://bengal.ihmc.us/svn/SimulationDispatcher/SimulationDispatcherInterfaces/SimulationDispatcherInterfaces.jar "
                                          + "https://bengal.ihmc.us/svn/SimulationDispatcher/SimulationDispatcherClient/SimulationDispatcherClient.jar "
                                          + "https://bengal.ihmc.us/svn/SimulationDispatcher/SimulationDispatcherServer/SimulationDispatcherServer.jar";

   private RemoteSimulationDescription description;
   private static final String password = "***REMOVED***";

   public RemoteSimulationRunner() throws RemoteException
   {
      super();
   }

   @Override
   public int add(int a, int b)
   {
      return a + b;
   }

   @Override
   public void createSimulation(RemoteSimulationDescription description, String[] structuralParameterNames, double[] structuralParameterValues, String password)
           throws RemoteException
   {
      if (DEBUG)
         System.out.println("Creating Simulation!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");
      this.description = description;
      description.createSimulation(structuralParameterNames, structuralParameterValues);
   }

   @Override
   public void destroySimulation(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Destroy Simulation!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");
      if (description != null)
         description.destroySimulation();
      description = null;
   }

   @Override
   public boolean ping(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Ping!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");

      return true;
   }

   @Override
   public void setSimulationState(Object state, String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Set Simulation State!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");
      description.setSimulationState(state);
   }

   @Override
   public void startSimulation(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Starting Simulation!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");
      description.startSimulation();
   }

   @Override
   public boolean isSimulationDone(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Checking isSimulationDone!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");
      if (description == null)
         return true;

      return description.isSimulationDone();
   }

   @Override
   public Object getSimulationState(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Getting Simulation State!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");

      return description.getSimulationState();
   }

   @Override
   public Object getSimulationData(String password) throws RemoteException
   {
      if (DEBUG)
         System.out.println("Getting Simulation Data!");

      if (!password.equals(RemoteSimulationRunner.password))
         throw new RemoteException("Invalid Password");

      return description.getSimulationData();
   }

   public static void main(String[] args)
   {
      int numberOfServers = 8;    // 16; //8;

      if ((args != null) && (args.length > 0) && (args[0] != null))
      {
         numberOfServers = Integer.parseInt(args[0]);
      }

      RMISecurityManager laxManager = new RMISecurityManager()
      {
         @Override
         public void checkPermission(java.security.Permission p)
         {
            // System.out.println("In checkPermission. permission = " + p);System.out.flush();

            // if (p.toString().equals("(java.lang.reflect.ReflectPermission suppressAccessChecks)")) return;
            // if (p.toString().equals("(java.util.PropertyPermission java.rmi.server.codebase write)")) return;

            // System.out.println(p);
            // try
            // {super.checkPermission(p);}
            // catch(Exception e)
            {
               // System.out.println(p);
            }

            // super.checkPermission(p);
         }

         /*
          * public void checkPropertiesAccess(){System.out.println("In checkPropertiesAccess");return;}
          * public void checkPropertyAccess(String key)
          * {
          * System.out.println("In checkPropertyAccess.  Key = " + key);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkLink(String lib)
          * {
          * System.out.println("In checkPropertyAccess.  Lib = " + lib);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkRead(String file)
          * {
          * System.out.println("In checkRead(Sting file).  file = " + file);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkRead(String file, Object context)
          * {
          * System.out.println("In checkRead(Sting file, Object context).  file = " + file);
          * System.out.flush();
          * return;
          * }
          * public void checkRead(java.io.FileDescriptor fd)
          * {
          * System.out.println("In checkRead(FileDescriptor fd)." );
          * System.out.flush();
          * return;
          * }
          *
          * public void checkAccess(Thread t)
          * {
          * System.out.println("In checkAccess(Thread t)." );
          * System.out.flush();
          * return;
          * }
          *
          * public void checkAccess(ThreadGroup g)
          * {
          * System.out.println("In checkAccess(ThreadGroup g)." );
          * System.out.flush();
          * return;
          * }
          *
          * public void checkConnect(String host, int port)
          * {
          * System.out.println("In checkConnect(String host, int port). host = " + host + " port = " + port);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkConnect(String host, int port, Object context)
          * {
          * System.out.println("In checkConnect(String host, int port, Object context). host = " + host + " port = " + port);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkWrite(String file)
          * {
          * System.out.println("In checkWrite(String file). file = " + file);
          * System.out.flush();
          * return;
          * }
          *
          * public void checkWrite(java.io.FileDescriptor fd)
          * {
          * System.out.println("In checkWrite(FileDescriptor fd). ");
          * System.out.flush();
          * return;
          * }
          */
      };

      // laxManager.checkWrite(

      System.setSecurityManager(laxManager);    // new RMISecurityManager());


      try
      {
         System.out.println("Creating RMI Registry.");
         Registry registry = LocateRegistry.createRegistry(1099);    // Start an RMI Registry
         System.out.println("RMI Registry " + registry + " Created.");
      }
      catch (RemoteException e)
      {
         System.out.println("RMI Registry allready exists!");

         // e.printStackTrace();
         // return;
      }


      if (SET_CODEBASE)
      {
         System.out.println("Setting the RMI codebase to " + codebase);
         System.setProperty("java.rmi.server.codebase", codebase);
      }


      for (int i = 0; i < numberOfServers; i++)
      {
         try
         {
            String runnerName = "RemoteSimulationRunner" + i;
            System.out.println("Creating and Binding " + runnerName);

            RemoteSimulationRunner rs = new RemoteSimulationRunner();
            Naming.rebind(runnerName, rs);
         }
         catch (Exception e)
         {
            System.out.println("RemoteSimulation Exception: " + e.getMessage());
            e.printStackTrace();

            return;
         }
      }

      System.out.println(numberOfServers + " RemoteSimulationRunners are now bound.");    // Time to tell lynx.coginst.uwf.edu about me...");
      InetAddress address;

      try
      {
         address = InetAddress.getLocalHost();
         System.out.println(address);
      }
      catch (java.net.UnknownHostException e)
      {
         System.out.println("Could not find this computer's address");

         return;
      }

//    while(true)
//    {
//      try
//      {
//        //RemoteIPRegistryInterface registry = (RemoteIPRegistryInterface) Naming.lookup("//lynx.coginst.uwf.edu/RemoteIPRegistry");
//        RemoteIPRegistryInterface registry = (RemoteIPRegistryInterface) Naming.lookup("//jaguar.ihmc.us/RemoteIPRegistry");
//        //RemoteIPRegistryInterface registry = (RemoteIPRegistryInterface) Naming.lookup("//lynx/RemoteIPRegistry");
//        registry.registerMe(address.toString(), RemoteSimulationRunner.password);
//
//        //java.io.net.
//        System.out.println("Waiting to Serve up Remote Simulations");
//
//        try{Thread.sleep(120000);} //0);} // sleep for two minutes...
//        catch(InterruptedException e){}
//      }
//      catch(Exception e)
//      {
//        //System.out.println("Could not find Lynx to tell about me...");
//        //e.printStackTrace();
//        try{Thread.sleep(10000);} //0);} // sleep for ten seconds...
//        catch(InterruptedException e2){}
//      }
//
//    }
   }


}
