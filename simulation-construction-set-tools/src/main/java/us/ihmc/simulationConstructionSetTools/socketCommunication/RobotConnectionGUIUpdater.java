package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.util.List;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.CreatedNewVariablesListener;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoVariable;

public class RobotConnectionGUIUpdater implements CreatedNewVariablesListener, ReceivedDataListener
{
   private final SimulationConstructionSet scs;
   
   public RobotConnectionGUIUpdater(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   @Override
   public void receivedData(List<YoVariable> sendVariables)
   {
      scs.tickAndUpdateLeisurely(4);    // +++JEP 050722: Make GUI more responsive...
   }

   @Override
   public void createdNewVariables(YoVariableList newVariables)
   {
      scs.addVarList(newVariables);
   }
   
   @Override
   public void createdNewVariable(YoVariable variable)
   {
      System.err.println("RobotConnectionGUIUpdater: created new variable: " + variable.getFullNameString());
   }
}
