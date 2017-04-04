package us.ihmc.simulationConstructionSetTools.gui.Debug;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DebugSCSVarListException
{
   public DebugSCSVarListException()
   {
   }

   public static void main(String[] args)
   {
      Robot robot = new Robot("Test");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      Thread thread = new Thread(scs);
      thread.start();

      int count = 0;
      while (true)
      {
         YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
         
         DoubleYoVariable foo = new DoubleYoVariable("foo_" + count, "", registry);

         YoVariableList varList = new YoVariableList("foo_" + count);
         varList.addVariable(foo);

         scs.addVarList(varList);

         count++;

         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }

   }
}
