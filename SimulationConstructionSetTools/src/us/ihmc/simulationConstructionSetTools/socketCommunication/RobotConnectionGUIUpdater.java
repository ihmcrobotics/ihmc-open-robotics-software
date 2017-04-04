package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.CreatedNewVariablesListener;

public class RobotConnectionGUIUpdater implements CreatedNewVariablesListener, ReceivedDataListener
{
   private final SimulationConstructionSet scs;
   
   public RobotConnectionGUIUpdater(SimulationConstructionSet scs)
   {
      this.scs = scs;
   }

   @Override
   public void receivedData(ArrayList<YoVariable<?>> sendVariables)
   {
      scs.tickAndUpdateLeisurely(4);    // +++JEP 050722: Make GUI more responsive...
   }

   @Override
   public void createdNewVariables(YoVariableList newVariables)
   {
      scs.addVarList(newVariables);
   }
   
   @Override
   public void createdNewVariable(YoVariable<?> variable)
   {
      System.err.println("RobotConnectionGUIUpdater: created new variable: " + variable.getFullNameWithNameSpace());
   }
}
