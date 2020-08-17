package us.ihmc.simulationConstructionSetTools.gui.Debug;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoDouble;

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
         YoRegistry registry = new YoRegistry("testRegistry");
         
         YoDouble foo = new YoDouble("foo_" + count, "", registry);

         YoVariableList varList = new YoVariableList("foo_" + count);
         varList.add(foo);

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
