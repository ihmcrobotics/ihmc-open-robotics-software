package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;
import java.util.List;

import javax.swing.event.ChangeListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;

public class YoVariableRegistryVarPanel extends YoVariablePanel
{
   private static final long serialVersionUID = -9079475583549031191L;
   private final YoVariableRegistry registry;
   
   public YoVariableRegistryVarPanel(YoVariableRegistry registry, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      super(registry.getName(), holder, varPanelJPopupMenu);
      this.registry = registry;
      clearAndSetUpTextFields();
   }
   
   @Override
   protected YoVariable<?> getYoVariable(int index)
   {
      return registry.getYoVariable(index);
   }

   @Override
   protected List<YoVariable<?>> getAllYoVariablesCopy()
   {
      return new ArrayList<YoVariable<?>> (registry.getAllVariablesInThisListOnly());
   }

   @Override
   protected int getNumberOfYoVariables()
   {
      return registry.getNumberOfYoVariables();
   }
   
   @Override
   public YoVariable<?> getYoVariable(String name)
   {
      return registry.getVariable(name);
   }

   @Override
   public void addChangeListener(ChangeListener changeListener)
   {
      throw new RuntimeException("YoVariableRegistryVarList.addChangeListener() not yet implemented.");
   }
}
