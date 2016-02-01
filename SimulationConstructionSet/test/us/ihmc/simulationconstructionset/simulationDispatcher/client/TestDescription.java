package us.ihmc.simulationconstructionset.simulationDispatcher.client;

import us.ihmc.simulationconstructionset.simulationDispatcher.interfaces.RemoteSimulationDescription;

@SuppressWarnings("serial")
public class TestDescription implements RemoteSimulationDescription
{
// YoVariable var;

   public TestDescription()    // YoVariable var)
   {
      System.out.println("Creating Test Description");

//    this.var = var;
//    System.out.println("Added YoVariable: " + var.getName());
   }

   public void executeSimulation()
   {
      System.out.println("TestDescription: executeSimulation");

//    System.out.println("YoVariable " + var.getName() + "has value: " + var.val);

   }

   public void createSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      System.out.println("TestDescription: createSimulation");
   }

   public void destroySimulation()
   {
      System.out.println("TestDescription: destroySimulation");
   }

   public void setSimulationState(Object object)
   {
   }

   public void startSimulation()
   {
      System.out.println("TestDescription: startSimulation");
   }

   public boolean isSimulationDone()
   {
      return false;
   }

   public Object getSimulationState()
   {
      return null;
   }

   public Object getSimulationData()
   {
      return null;
   }

}
