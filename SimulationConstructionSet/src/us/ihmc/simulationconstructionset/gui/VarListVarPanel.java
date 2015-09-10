package us.ihmc.simulationconstructionset.gui;

import java.awt.Component;
import java.util.ArrayList;
import java.util.List;

import javax.swing.event.ChangeListener;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;

public class VarListVarPanel extends VarPanel
{
   private static final long serialVersionUID = -995471860623897960L;
   private YoVariableList varList;

   public VarListVarPanel(String name, SelectedVariableHolder holder, VarPanelJPopupMenu varPanelJPopupMenu, VariableSearchPanel searchPanel)
   {
      this(new YoVariableList(name), holder, varPanelJPopupMenu, searchPanel);
   }

   public VarListVarPanel(YoVariableList list, SelectedVariableHolder holder, VarPanelJPopupMenu varPanelJPopupMenu, VariableSearchPanel searchPanel)
   {
      super(list.getName(), holder, varPanelJPopupMenu, searchPanel);
      this.varList = list;
      clearAndSetUpTextFields();
   }

   public VarListVarPanel(String name, SelectedVariableHolder holder, VarPanelJPopupMenu varPanelJPopupMenu)
   {
      this(new YoVariableList(name), holder, varPanelJPopupMenu, null);
   }

   public VarListVarPanel(YoVariableList list, SelectedVariableHolder holder, VarPanelJPopupMenu varPanelJPopupMenu)
   {
      this(list, holder, varPanelJPopupMenu, null);
   }

   protected int getNumberOfYoVariables()
   {
      if (varList == null)
         return 0;

      return varList.size();
   }

   protected YoVariable getYoVariable(int index)
   {
      return varList.getVariable(index);
   }

   protected List<YoVariable> getAllYoVariablesCopy()
   {
      return new ArrayList<YoVariable>(varList.getVariables());
   }

   public void addChangeListener(ChangeListener changeListener)
   {
      varList.addChangeListener(changeListener);
   }

   public YoVariable getYoVariable(String name)
   {
      return varList.getVariable(name);
   }

   public void addVariable(final YoVariable v)
   {
      if (v != null)
      {
         varList.addVariable(v);

         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            public void run()
            {
               addTextFieldForVariable(v);
            }
         });
      }
   }

   public void removeVariable(YoVariable v)
   {
      int indexOfVariableValueToRemove = varList.getIndexOfVariable(v);
      if ((indexOfVariableValueToRemove >= 0) && (indexOfVariableValueToRemove < textFields.size()))
      {
         System.out.println("removing var");
         Component field = textFields.remove(indexOfVariableValueToRemove);
         this.remove(field);
      }

      varList.removeVariable(v);
      this.invalidate();
      this.revalidate();
      this.updateUI();
      this.repaint();
   }

   public void removeAllVariables()
   {
      varList.removeAllVariables();
      textFields.clear();
      this.removeAll();

      //    this.updateUI();
   }

   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      retBuffer.append(varList.toString());

      return retBuffer.toString();
   }
}