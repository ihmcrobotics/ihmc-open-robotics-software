package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class MasterVariableChangedListener implements VariableChangedListener
{
   private final Collection<DoubleYoVariable> slaves;

   public MasterVariableChangedListener(Collection<DoubleYoVariable> slaves)
   {
      this.slaves = slaves;
   }
   
   public MasterVariableChangedListener(DoubleYoVariable slave)
   {
      this.slaves = new ArrayList<DoubleYoVariable>(1);
      slaves.add(slave);
   }

   @Override
   public void variableChanged(YoVariable<?> master)
   {
      for (DoubleYoVariable slave : slaves)
      {
         slave.set(master.getValueAsDouble());
      }
   }
}