package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class MasterVariableChangedListener implements VariableChangedListener
{
   private final Collection<YoDouble> slaves;

   public MasterVariableChangedListener(Collection<YoDouble> slaves)
   {
      this.slaves = slaves;
   }
   
   public MasterVariableChangedListener(YoDouble slave)
   {
      this.slaves = new ArrayList<YoDouble>(1);
      slaves.add(slave);
   }

   @Override
   public void notifyOfVariableChange(YoVariable<?> master)
   {
      for (YoDouble slave : slaves)
      {
         slave.set(master.getValueAsDouble());
      }
   }
}