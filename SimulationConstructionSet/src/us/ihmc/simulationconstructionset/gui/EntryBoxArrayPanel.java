package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JButton;
import javax.swing.JPanel;

import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;


public class EntryBoxArrayPanel extends JPanel
{
   private static final long serialVersionUID = 269886151605236788L;
   private static final int ENTRY_BOX_HEIGHT = 26;
   public static final int DESIRED_PIXELS_PER_ENTRY_BOX = 200; // 180;
   public static final int MAXIMUM_PIXELS_PER_ENTRY_BOX = 400;

   public static final int MAX_ENTRY_BOXES = 100; // 40;

   private JButton plusButton;
   private JButton minusButton;

   private ArrayList<YoEntryBox> entryBoxesOnThisPanel;
   private SelectedVariableHolder selectedVariableHolder;
   private final FlowLayout layout;

   private Timer alertChangeListenersTimer;
   private TimerTask alertChangeListenersTask;
   private final long OBSERVER_NOTIFICATION_PERIOD = 250;

   public EntryBoxArrayPanel(Container frame, SelectedVariableHolder holder, DoubleYoVariable[] varsToEnter, JButton plusButton, JButton minusButton)
   {
      this.setName("EntryBoxArrayPanel");

      this.plusButton = plusButton;
      this.minusButton = minusButton;

      this.add(plusButton);
      this.add(minusButton);

      layout = new FlowLayout(FlowLayout.LEFT, 0, 0);
      this.setLayout(layout);

      this.selectedVariableHolder = holder;
      this.setBackground(Color.lightGray);

      this.setOpaque(true);
      this.entryBoxesOnThisPanel = new ArrayList<YoEntryBox>();

      this.addEntryBox(new YoEntryBox(this, selectedVariableHolder));

      for (int i = 0; i < varsToEnter.length; i++)
      {
         if (varsToEnter[i] != null)
         {
            YoEntryBox e = new YoEntryBox(this, selectedVariableHolder);
            e.addVariable(varsToEnter[i]);
            this.addEntryBox(e);
         }
      }

      createAndStartPeriodicUIUpdateThread();

      this.validate();
   }

   private void createAndStartPeriodicUIUpdateThread()
   {
      alertChangeListenersTimer = new Timer("EntryBoxArrayPanelTimer");
      alertChangeListenersTask = new TimerTask()
      {
         public void run()
         {
            final ArrayList<YoEntryBox> entryBoxes = new ArrayList<YoEntryBox>(entryBoxesOnThisPanel);
            if (entryBoxes.size() > 0)
            {
               EventDispatchThreadHelper.justRun(new Runnable()
               {

                  public void run()
                  {
                     for (YoEntryBox entryBox : entryBoxes)
                     {
                        entryBox.updateActiveContainer();
                     }
                  }
               });
            }
         }
      };
      alertChangeListenersTimer.schedule(alertChangeListenersTask, 1000, OBSERVER_NOTIFICATION_PERIOD);
   }

   public void closeAndDispose()
   {
      this.removeAll();

      plusButton = null;
      minusButton = null;

      if (entryBoxesOnThisPanel != null)
      {
         entryBoxesOnThisPanel.clear();
         entryBoxesOnThisPanel = null;
      }

      selectedVariableHolder = null;

      if (alertChangeListenersTimer != null)
      {
         alertChangeListenersTimer.cancel();
         alertChangeListenersTimer = null;
      }

      alertChangeListenersTask = null;

   }

   public boolean isHoldingVariable(YoVariable v)
   {
      boolean ret = false;

      for (int i = 0; i < entryBoxesOnThisPanel.size(); i++)
      {
         YoEntryBox e = entryBoxesOnThisPanel.get(i);
         if (e.isHoldingVariable(v))
            ret = true;
      }

      return ret;

   }

   public ArrayList<YoEntryBox> getEntryBoxesOnThisPanel()
   {
      return entryBoxesOnThisPanel;
   }

   public synchronized void addEntryBox(YoEntryBox entryBox)
   {
      selectedVariableHolder.addChangeListener(entryBox);
      int numBoxes = this.entryBoxesOnThisPanel.size();
      if (numBoxes > EntryBoxArrayPanel.MAX_ENTRY_BOXES)
         return;
      else if (numBoxes < EntryBoxArrayPanel.MAX_ENTRY_BOXES)
      {
         this.entryBoxesOnThisPanel.add(entryBox);
         this.add(entryBox);
      }
      else
      {
         YoEntryBox lastEntryBox = this.entryBoxesOnThisPanel.get(numBoxes - 1);
         if (lastEntryBox.getNumVars() == 0)
         {
            this.entryBoxesOnThisPanel.remove(lastEntryBox);
            this.remove(lastEntryBox);

            this.entryBoxesOnThisPanel.add(entryBox);
            this.add(entryBox);
         }
      }

      this.checkStatus();
   }

   public void addEntryBox(final YoVariable v)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         public void run()
         {
            YoEntryBox entryBox = new YoEntryBox(getEntryBoxArrayPanel(), selectedVariableHolder);
            entryBox.addVariable(v);

            addEntryBox(entryBox);
         }
      });
   }

   private EntryBoxArrayPanel getEntryBoxArrayPanel()
   {
      return this;
   }

   public void removeEntryBox(YoEntryBox entryBox)
   {
      this.entryBoxesOnThisPanel.remove(entryBox);
      this.remove(entryBox);
      this.checkStatus();
   }

   public void removeAllEntryBoxes()
   {
      this.entryBoxesOnThisPanel.clear();
      this.removeAll();

      this.add(plusButton);
      this.add(minusButton);

      updateRowsColumns();

   }

   public synchronized void checkStatus()
   {
      EventDispatchThreadHelper.invokeLater(new Runnable()
      {
         public void run()
         {
            checkStatusThreadUnsafe();
         }
      });

   }

   private synchronized void checkStatusThreadUnsafe()
   {
      YoEntryBox dirtyEntryBox = null;

      int numEntryBoxes = entryBoxesOnThisPanel.size();
      for (int i = 0; i < numEntryBoxes; i++)
      {
         YoEntryBox entryBox = entryBoxesOnThisPanel.get(i);
         if ((entryBox.getNumVars() == 0) && (i < numEntryBoxes - 1))
         {
            dirtyEntryBox = entryBox;
         }
      }

      if (dirtyEntryBox != null)
      {
         this.remove(dirtyEntryBox);
         this.entryBoxesOnThisPanel.remove(dirtyEntryBox);
         checkStatus();
      }

      else
      {
         if (numEntryBoxes == 0)
         {
            addEntryBox(new YoEntryBox(this, selectedVariableHolder));
         }
         else if (numEntryBoxes < EntryBoxArrayPanel.MAX_ENTRY_BOXES)
         {
            YoEntryBox lastEntryBox = this.entryBoxesOnThisPanel.get(numEntryBoxes - 1);
            if (lastEntryBox.getNumVars() > 0)
               addEntryBox(new YoEntryBox(this, selectedVariableHolder));
         }

         updateRowsColumns();
      }
   }

   public void updateRowsColumns()
   {
      int numRows = 1;
      int accumulatedWidth = 0;
      accumulatedWidth += this.plusButton.getPreferredSize().width;
      accumulatedWidth += this.minusButton.getPreferredSize().width;
      int panelWidth = this.getWidth();
      if (panelWidth == 0)
      {
         panelWidth = 1428;
      }
      for (YoEntryBox box : entryBoxesOnThisPanel)
      {
         accumulatedWidth += box.getPreferredSize().width;
         if (accumulatedWidth > panelWidth)
         {
            numRows++;
            accumulatedWidth = box.getPreferredSize().width;
         }
      }
      this.setPreferredSize(new Dimension(panelWidth, numRows * ENTRY_BOX_HEIGHT));
   }

   public String getXMLRepresentationOfClass()
   {
      String returnString = "<Entry Boxes>";
      int numOfFullBoxes = 0;
      ArrayList<YoEntryBox> entryBoxesOnThisPanel = getEntryBoxesOnThisPanel();

      for (int i = 0; i < entryBoxesOnThisPanel.size(); i++)
      {
         YoEntryBox yo = entryBoxesOnThisPanel.get(i);
         if (yo.getVariableInThisBox() != null)
         {
            numOfFullBoxes++;
         }
      }

      int currentBox = 0;

      for (int i = 0; i < entryBoxesOnThisPanel.size(); i++)
      {
         YoEntryBox yo = entryBoxesOnThisPanel.get(i);
         if (yo.getVariableInThisBox() != null)
         {
            returnString += yo.getVariableInThisBox().getName();
            currentBox++;
            if (currentBox < numOfFullBoxes)
               returnString += ",";
         }
      }

      returnString += "</Entry Boxes>";

      return returnString;
   }

}
