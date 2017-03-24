package us.ihmc.simulationconstructionset.gui;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.text.NumberFormat;
import java.util.ArrayList;

import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.TransferHandler;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class YoEntryBoxNew extends JPanel implements MouseListener, ActionListener, FocusListener
{
   public interface YoEntryContainer
   {
      void bindToVariable(YoVariable<?> variable);

      void focusGained(FocusEvent evt);

      void focusLost(FocusEvent evt);

      void update();

      String getToolTipText();

      void removeVariable();

      void actionPerformed(ActionEvent evt);
   }


   public class YoTextEntryBox implements YoEntryContainer, ActionListener
   {
      private YoEntryBoxNew entryBox;
      private JTextField jTextField;
      private JLabel label;
      private NumberFormat numFormat;
      private YoVariable<?> variableInThisBox;

      public YoTextEntryBox(YoEntryBoxNew entryBox, String label)
      {
         this.entryBox = entryBox;
         this.label = new JLabel(label);
         this.label.setHorizontalAlignment(SwingConstants.RIGHT);

         this.numFormat = NumberFormat.getInstance();
         this.numFormat.setMaximumFractionDigits(4);
         this.numFormat.setMinimumFractionDigits(1);

         this.numFormat.setGroupingUsed(false);

         jTextField = new JTextField("0.0");
         jTextField.setHorizontalAlignment(SwingConstants.LEFT);
         jTextField.addActionListener(this);
         jTextField.addFocusListener(entryBox);
         entryBox.add(jTextField);
      }

      @Override
      public void actionPerformed(ActionEvent evt)
      {
         String text = jTextField.getText();

         try
         {
            double val = Double.valueOf(text).doubleValue();
            if (variableInThisBox != null)
            {
               variableInThisBox.setValueFromDouble(val);

               setTextField();

               // this.updateUI();
               entryBox.repaint();

               for (int i = 0; i < variableChangedListeners.size(); i++)
               {
                  VariableChangedListener listener = variableChangedListeners.get(i);
                  listener.variableChanged(variableInThisBox);
               }
            }

            if (entryBoxArrayPanel != null)
               entryBoxArrayPanel.requestFocus();
            else
               entryBox.getParent().requestFocus();

         }
         catch (NumberFormatException e)
         {
            setTextField();
         }

      }

      @Override
      public void bindToVariable(YoVariable<?> variable)
      {
         String textName = variable.getName();

         label.setText(textName);
         setTextField();

      }

      @Override
      public void focusGained(FocusEvent evt)
      {
         if (variableInThisBox != null)
         {
            setTextField();
            jTextField.setCaretPosition(1);
         }

      }

      @Override
      public void focusLost(FocusEvent evt)
      {
         if (evt.getSource().equals(jTextField))
         {
            if (variableInThisBox != null)
            {
               setTextField();
               jTextField.setCaretPosition(1);
            }
         }

      }

      public JTextField getjTextField()
      {
         return jTextField;
      }

      public JLabel getLabel()
      {
         return label;
      }

      @Override
      public String getToolTipText()
      {
         return label.getText();
      }

      @Override
      public void removeVariable()
      {
         variableInThisBox = null;
         label.setText(DEFAULT_EMPTY_ENTRY_BOX_LABEL);

      }

      public void setjTextField(JTextField jTextField)
      {
         this.jTextField = jTextField;
      }

      public void setLabel(JLabel label)
      {
         this.label = label;
      }

      @Override
      public void update()
      {
         setTextField();
      }

      public synchronized void setTextField()
      {
         if (jTextField.hasFocus())
         {
         }
         else
         {
            if (variableInThisBox != null)
            {
               double varInBoxValue = variableInThisBox.getValueAsDouble();

               String textForTextField = numFormat.format(varInBoxValue) + " ";
               if (textForTextField.length() < 15)
               {
                  for (int i = 0; i < 15 - textForTextField.length(); i++)
                  {
                     textForTextField = textForTextField + " ";
                  }
               }

               jTextField.setText(textForTextField);


               // jTextField.setText("  " + numFormat.format(variableInThisBox.val));
            }
            else
               jTextField.setText(numFormat.format(0.0));

            jTextField.setCaretPosition(0);

            entryBox.updateUI();
         }
      }

   }


   public static final String DEFAULT_EMPTY_ENTRY_BOX_LABEL = "UNUSED";

   private static final long serialVersionUID = -3598041913472018651L;

   private static ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   public static void attachVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }

   public static void removeVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.remove(listener);
   }

   private YoEntryContainer activeEntryContainer;

   private EntryBoxArrayPanel entryBoxArrayPanel;


   private JPopupMenu popupMenu;


   // private JLabel label;
// private JTextField jTextField;
   private SelectedVariableHolder selectedVariableHolder;

   private YoVariable<?> variableInThisBox;

   public YoEntryBoxNew(EntryBoxArrayPanel entryBoxArrayPanel, SelectedVariableHolder holder)
   {
      super(new GridLayout(1, 2));

      this.selectedVariableHolder = holder;

      this.entryBoxArrayPanel = entryBoxArrayPanel;
      this.setOpaque(true);


      activeEntryContainer = new YoTextEntryBox(this, DEFAULT_EMPTY_ENTRY_BOX_LABEL);

//    label = new JLabel("UNUSED");
//    label.setHorizontalAlignment(SwingConstants.RIGHT);
//    this.add(label);

//    this.numFormat = NumberFormat.getInstance();
//    this.numFormat.setMaximumFractionDigits(4);
//    this.numFormat.setMinimumFractionDigits(1);

//    this.numFormat.setGroupingUsed(false);

//    jTextField = new JTextField("0.0");
//    jTextField.setHorizontalAlignment(SwingConstants.LEFT);
//    jTextField.addActionListener(this);
//    jTextField.addFocusListener(this);
      this.addFocusListener(this);

//    this.add(jTextField);
      this.addMouseListener(this);
      //this.setDropTarget(new DropTarget(this, new YoEntryBoxTargetListener(this)));
      this.setTransferHandler(new YoEntryBoxTransferHandler());

      popupMenu = new ForcedRepaintPopupMenu();
      JMenuItem delete = new JMenuItem("Delete Entry Box");
      delete.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            if (variableInThisBox != null)
               removeVariable(variableInThisBox);
         }
      });
      popupMenu.add(delete);
      this.setToolTipText(activeEntryContainer.getToolTipText());
   }

   @Override
   public void actionPerformed(ActionEvent evt)
   {
      // JTextField textField = (JTextField) evt.getSource();
      // String com = evt.getActionCommand();
      activeEntryContainer.actionPerformed(evt);


   }

   public void addVariable(YoVariable<?> variable)
   {
      if ((entryBoxArrayPanel != null) && entryBoxArrayPanel.isHoldingVariable(variable))
         return;

      activeEntryContainer.bindToVariable(variable);
      variableInThisBox = variable;

//    label.setText(textName);
      String toolTip = variableInThisBox.getDescription();
      if ((toolTip == null) || toolTip.equals(""))
         toolTip = variableInThisBox.getFullNameWithNameSpace();
      this.setToolTipText(toolTip);

//    setTextField();

      if (entryBoxArrayPanel != null)
         this.entryBoxArrayPanel.checkStatus();

   }

   @Override
   public void focusGained(FocusEvent evt)
   {
      activeEntryContainer.focusGained(evt);

      if (entryBoxArrayPanel != null)
         entryBoxArrayPanel.requestFocus();
      else
         this.getParent().requestFocus();
   }

   @Override
   public void focusLost(FocusEvent evt)
   {
      activeEntryContainer.focusLost(evt);

      if (evt.getSource().equals(this))
      {
         if (popupMenu.isVisible())
         {
            popupMenu.setVisible(false);
         }
      }
   }

   public int getNumVars()
   {
      if (this.variableInThisBox != null)
         return 1;
      else
         return 0;
   }

   public SelectedVariableHolder getSelectedVariableHolder()
   {
      return selectedVariableHolder;
   }

   public YoVariable<?> getVariableInThisBox()
   {
      return variableInThisBox;
   }

   public boolean isHoldingVariable(YoVariable<?> v)
   {
      return (this.variableInThisBox == v);
   }

   @Override
   public void mouseClicked(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }

   public void update()
   {
	   activeEntryContainer.update();
   }
   @Override
   public void mousePressed(MouseEvent evt)
   {
      this.requestFocus();

      if (evt.isMetaDown() &&!(evt.isAltDown()))
      {
         popupMenu.setLocation(evt.getXOnScreen(), evt.getYOnScreen());
         popupMenu.setVisible(true);
      }

      // Left click places index:

      else
      {
         if (variableInThisBox == null)
         {
            // YoVariable v = VarPanel.selectedVariable;

            YoVariable<?> v = selectedVariableHolder.getSelectedVariable();
            if (v != null)
               this.addVariable(v);
         }
         else
         {
            selectedVariableHolder.setSelectedVariable(variableInThisBox);

            if (!evt.isControlDown())
            {
               this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.MOVE);
               YoGraph.setActionPerformedByDragAndDrop(TransferHandler.MOVE);
            }
            else if (evt.isControlDown())
            {
               this.getTransferHandler().exportAsDrag(this, evt, TransferHandler.COPY);
               YoGraph.setActionPerformedByDragAndDrop(TransferHandler.COPY);
            }

            YoGraph.setSourceOfDrag(this);
         }
      }


   }

// public synchronized void setTextField()
// {
//    if (jTextField.hasFocus())
//    {
//    }
//    else
//    {
//       if (variableInThisBox != null)
//       {
//          double varInBoxValue = variableInThisBox.getValueAsDouble();
//
//          String textForTextField = numFormat.format(varInBoxValue) + " ";
//          if (textForTextField.length() < 15)
//          {
//             for (int i = 0; i < 15 - textForTextField.length(); i++)
//             {
//                textForTextField = textForTextField + " ";
//             }
//          }
//
//          jTextField.setText(textForTextField);
//
//
//          // jTextField.setText("  " + numFormat.format(variableInThisBox.val));
//       }
//       else
//          jTextField.setText(numFormat.format(0.0));
//
//       jTextField.setCaretPosition(0);
//
//       this.updateUI();
//    }
// }


   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }


   public void removeVariable(YoVariable<?> variable)
   {
      if (variableInThisBox == variable)
      {
         variableInThisBox = null;
         activeEntryContainer.removeVariable();
      }




      this.setToolTipText(DEFAULT_EMPTY_ENTRY_BOX_LABEL);

      if (entryBoxArrayPanel != null)
         this.entryBoxArrayPanel.checkStatus();

      if (popupMenu.isVisible())
         popupMenu.setVisible(false);

      this.updateUI();
   }


   public void setVariableInThisBox(YoVariable<?> variableInThisBox)
   {
      activeEntryContainer.bindToVariable(variableInThisBox);

      this.variableInThisBox = variableInThisBox;

      String toolTip = variableInThisBox.getDescription();
      if ((toolTip == null) || toolTip.equals(""))
         toolTip = variableInThisBox.getFullNameWithNameSpace();
      this.setToolTipText(toolTip);
   }


}
