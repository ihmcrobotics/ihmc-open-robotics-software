package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

import us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces.RemoteSimulationDescription;

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

   @Override
   public void createSimulation(String[] structuralParameterNames, double[] structuralParameterValues)
   {
      System.out.println("TestDescription: createSimulation");
   }

   @Override
   public void destroySimulation()
   {
      System.out.println("TestDescription: destroySimulation");
   }

   @Override
   public void setSimulationState(Object object)
   {
   }

   @Override
   public void startSimulation()
   {
      System.out.println("TestDescription: startSimulation");
   }

   @Override
   public boolean isSimulationDone()
   {
      return false;
   }

   @Override
   public Object getSimulationState()
   {
      return null;
   }

   @Override
   public Object getSimulationData()
   {
      return null;
   }

}
