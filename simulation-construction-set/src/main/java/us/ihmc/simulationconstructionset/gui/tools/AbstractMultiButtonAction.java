package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.Collection;

import javax.swing.AbstractAction;
import javax.swing.AbstractButton;
import javax.swing.Action;

public abstract class AbstractMultiButtonAction extends AbstractAction implements Action
{
   private static final long serialVersionUID = 5549057675462205945L;
   protected ArrayList<AbstractButton> buttons = new ArrayList<AbstractButton>();

   public AbstractMultiButtonAction(String name)
   {
      super(name);
   }

   @Override
   public abstract void actionPerformed(ActionEvent e);


   public void addButton(AbstractButton button)
   {
      buttons.add(button);
   }

   public void removeButton(AbstractButton button)
   {
      buttons.remove(button);
   }

   public boolean addAllButtons(Collection<? extends AbstractButton> arg0)
   {
      return buttons.addAll(arg0);
   }

   public void clearButtons()
   {
      buttons.clear();
   }

   public boolean containsButton(AbstractButton button)
   {
      return buttons.contains(button);
   }

   public boolean isButtonsEmpty()
   {
      return buttons.isEmpty();
   }

   public boolean removeAllButtons(Collection<? extends AbstractButton> buttons)
   {
      return buttons.removeAll(buttons);
   }

   public int sizeOfButtons()
   {
      return buttons.size();
   }

   public AbstractButton[] toArrayOfButtons()
   {
      AbstractButton[] ret = new AbstractButton[buttons.size()];
      buttons.toArray(ret);
      return ret;
   }
}
