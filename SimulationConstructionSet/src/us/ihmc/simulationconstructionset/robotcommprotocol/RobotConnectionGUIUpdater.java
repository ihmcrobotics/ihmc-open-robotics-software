package us.ihmc.simulationconstructionset.robotcommprotocol;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariableList;

public class RobotConnectionGUIUpdater implements CreatedNewVariablesListener, ReceivedDataListener
{
   private final SimulationConstructionSet scs;
   
   public RobotConnectionGUIUpdater(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   public void receivedData(ArrayList<YoVariable> sendVariables)
   {
      scs.tickAndUpdateLeisurely(4);    // +++JEP 050722: Make GUI more responsive...
   }

   public void createdNewVariables(YoVariableList newVariables)
   {
      scs.addVarList(newVariables);
   }
   
   public void createdNewVariable(YoVariable variable)
   {
      System.err.println("RobotConnectionGUIUpdater: created new variable: " + variable.getFullNameWithNameSpace());
   }
}
