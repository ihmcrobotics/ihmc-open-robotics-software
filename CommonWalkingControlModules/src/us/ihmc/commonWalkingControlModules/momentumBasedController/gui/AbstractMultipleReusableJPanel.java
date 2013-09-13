package us.ihmc.commonWalkingControlModules.momentumBasedController.gui;

import java.awt.Component;
import java.awt.GridLayout;
import java.util.ArrayList;

import javax.swing.JPanel;

public abstract class AbstractMultipleReusableJPanel<E extends JPanel> extends JPanel
{
   private static final long serialVersionUID = -4892971095902174475L;
   private static final boolean DEBUG = true;

   // private final ArrayList<E> jPanelPool = new ArrayList<DesiredJointAccelerationJPanel>();
   private final ArrayList<E> jPanelsInUse = new ArrayList<E>();

   public abstract E constructNewJPanel();


   protected void rearrangePanelsIfNecessary(int numberOfDesiredJPanels)
   {
      int numberJPanelsInUse = jPanelsInUse.size();

      boolean changed = numberOfDesiredJPanels != numberJPanelsInUse;
     
      if (changed)
      {
         this.setLayout(new GridLayout(numberOfDesiredJPanels, 1));
      }
      
      if (numberOfDesiredJPanels > numberJPanelsInUse)
      {
         int numberExtraWeNeedToAdd = numberOfDesiredJPanels - numberJPanelsInUse;

         printIfDebug("Creating " + numberExtraWeNeedToAdd + " extra panels.");

         for (int i = 0; i < numberExtraWeNeedToAdd; i++)
         {
            E desiredJPanel = constructNewJPanel();
            jPanelsInUse.add(desiredJPanel);
            this.add(desiredJPanel);
         }
      }
      else if (numberOfDesiredJPanels < numberJPanelsInUse)
      {
         int numberExtraWeNeedToTakeAway = numberJPanelsInUse - numberOfDesiredJPanels;

         printIfDebug("Removing " + numberExtraWeNeedToTakeAway + " extra panels.");

         for (int i = 0; i < numberExtraWeNeedToTakeAway; i++)
         {
            int removalIndex = jPanelsInUse.size() - 1;
            E panelToRemove = jPanelsInUse.remove(removalIndex);
            this.remove(panelToRemove);
         }
      }
      
      if (changed)
      {
         verifyCorrectLayout(numberOfDesiredJPanels);
         this.updateUI();
      }

   }

   protected E getJPanel(int i)
   {
      return this.jPanelsInUse.get(i);
   }
   
   private void verifyCorrectLayout(int numberOfDesiredJPanels)
   {
      printIfDebug("Verifying correct layout...");

      if (numberOfDesiredJPanels != jPanelsInUse.size())
      {
         throw new RuntimeException("numberOfDesiredJPanels != jPanelsInUse.size()");
      }

      Component[] components = this.getComponents();
      if (components.length != jPanelsInUse.size())
      {
         throw new RuntimeException("components.length != jPanelsInUse.size()");
      }

      for (int i = 0; i < components.length; i++)
      {
         if (components[i] != jPanelsInUse.get(i))
         {
            throw new RuntimeException("components[i] != jPanelsInUse.get(i)");
         }
      }
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

}
