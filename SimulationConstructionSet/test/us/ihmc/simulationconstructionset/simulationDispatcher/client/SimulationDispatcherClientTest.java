package us.ihmc.simulationconstructionset.simulationDispatcher.client;

import java.rmi.registry.LocateRegistry;
import java.rmi.registry.Registry;

import org.junit.Test;

import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationRunnerInterface;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.MANUAL)
public class SimulationDispatcherClientTest
{
   private static final String pwd = "***REMOVED***";

   // private static final String serverURL =
   // "//10.100.0.48/RemoteSimulationRunner";
//   private static final String serverURL = "RemoteSimulationRunner0";
   
   private static final String serverURL = "cheetah:0";

   
// private static final String serverURL = "//10.100.0.74/RemoteSimulationRunner0";

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout=300000)
   public void test()
   {
//    YoVariable foo = new YoVariable("foo", null);
//    foo.val = 7.3;

      String codebase = System.getProperty("java.rmi.server.codebase");
      if (codebase == "null")
      {
         codebase = "";
      }

//      String myCodeBase = "file://halo/Public/Projects/SimulationDispatcher/SimulationDispatcherTestClient.jar";
      String myCodeBase = "https://bengal.ihmc.us/svn/SimulationDispatcher/SimulationDispatcherTestClient/SimulationDispatcherTestClient.jar";

      if (myCodeBase != null)
         codebase = codebase + myCodeBase;

      // http://www.coginst.uwf.edu/~jpratt/SimulationDispatcher/codebase/flamingodispatcher78.jar
//    codebase = codebase + " http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationconstructionset.jar http://www.ihmc.us/~jpratt/SimulationDispatcher/codebase/simulationdispatcherclient.jar";

      System.out.println("Setting the RMI codebase to " + codebase);
      System.setProperty("java.rmi.server.codebase", codebase);


      TestDescription description = new TestDescription();    // foo);

      System.out.println("Dispatching Simulation. ");

      RemoteSimulationRunnerInterface remoteSim = null;

//    System.setSecurityManager(new RMISecurityManager());  

      try
      {
         Registry registry = LocateRegistry.getRegistry("localhost");
         remoteSim = (RemoteSimulationRunnerInterface) registry.lookup(serverURL);

         int onePlusTwo = remoteSim.add(1, 2);
         System.out.println("onePlusTwo = " + onePlusTwo);

      }
      catch (Exception e)
      {
         System.out.println("Exception in TestClient1:  " + e.getMessage());
         e.printStackTrace();
         System.exit(-1);
      }

      try
      {
         remoteSim.createSimulation(description, null, null, pwd);
         System.out.println("Created Simulation");

      }
      catch (Exception e)
      {
         System.out.println("Exception in TestClient2");

//       e.printStackTrace();

         System.out.println("Root Cause = " + getRootCause(e));
         System.exit(-1);
      }


      try
      {
         remoteSim.startSimulation(pwd);
         System.out.println("Started Simulation");

         // remoteSim.setSimulation(new PinJoint("test", new Vector3d(), rob,
         // Axis.X));
         // remoteSim.setSimulation(new Link("test"));
         // remoteSim.setSimulation(YoAppearance.Red());
         // remoteSim.setSimulation(controller);

      }
      catch (Exception e)
      {
         System.out.println("Exception in TestClient3:  " + e.getMessage());
         e.printStackTrace();
         System.exit(-1);
      }

      System.out.println("Test Completed!!");

   }

   private Throwable getRootCause(Throwable throwable)
   {
      Throwable cause = throwable;

      while (cause.getCause() != null)
      {
         cause = cause.getCause();
      }

      return cause;
   }
}
