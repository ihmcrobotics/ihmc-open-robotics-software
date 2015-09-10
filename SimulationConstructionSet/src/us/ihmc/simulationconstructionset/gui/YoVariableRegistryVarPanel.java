package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;
import java.util.List;

import javax.swing.event.ChangeListener;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoVariableRegistryVarPanel extends VarPanel
{
   private static final long serialVersionUID = -9079475583549031191L;
   private final YoVariableRegistry registry;
   
   public YoVariableRegistryVarPanel(YoVariableRegistry registry, SelectedVariableHolder holder, VarPanelJPopupMenu varPanelJPopupMenu)
   {
      super(registry.getName(), holder, varPanelJPopupMenu);
      this.registry = registry;
      clearAndSetUpTextFields();
   }
   
   protected YoVariable getYoVariable(int index)
   {
      return registry.getYoVariable(index);
   }

   protected List<YoVariable> getAllYoVariablesCopy()
   {
      return new ArrayList<YoVariable>(registry.getAllVariablesInThisListOnly());
   }

   protected int getNumberOfYoVariables()
   {
      return registry.getNumberOfYoVariables();
   }
   
   public YoVariable getYoVariable(String name)
   {
      return registry.getVariable(name);
   }

   public void addChangeListener(ChangeListener changeListener)
   {
      throw new RuntimeException("YoVariableRegistryVarList.addChangeListener() not yet implemented.");
   }
}
