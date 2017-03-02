package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.FocusEvent;
import java.text.NumberFormat;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoTextEntryContainer implements YoVariableEntryContainer
{
   private static final int TEXT_BOX_WIDTH = 100;
   private static final int BOX_HEIGHT = YoEntryBox.COMPONENT_HEIGHT;
   private NumberFormat numFormat;
   private YoVariable<?> variableInThisBox;
   private JLabel label;
   private JTextField jTextField;

   protected YoTextEntryContainer()
   {
   }

   @Override
   public YoVariable<?> getVariable()
   {
      return variableInThisBox;
   }


   @Override
   public synchronized void update(YoEntryBox yoEntryBox)
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

         yoEntryBox.updateUI();
      }
   }

   @Override
   public void actionPerformed(YoEntryBox yoEntryBox, ActionEvent evt)
   {
      String text = jTextField.getText();

//    System.out.println("YoTextEntryContainer: Hey, I haz an actionPerformed!");

      try
      {
         double val = Double.valueOf(text).doubleValue();
         if (variableInThisBox != null)
         {
            variableInThisBox.setValueFromDouble(val);
            update(yoEntryBox);

            yoEntryBox.repaint();

            YoEntryBox.informVariableChangedListeners(getVariable());
         }

         yoEntryBox.passOnFocusRequest();

      }
      catch (NumberFormatException e)
      {
         update(yoEntryBox);
      }

   }


   @Override
   public void removeVariable(YoVariable<?> variable)
   {
      if (getVariable() == variable)
         variableInThisBox = null;

      label.setText(YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
   }


   @Override
   public void setup(YoEntryBox yoEntryBox)
   {
      label = new JLabel(YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
      label.setHorizontalAlignment(SwingConstants.RIGHT);
      numFormat = NumberFormat.getInstance();
      numFormat.setMaximumFractionDigits(4);
      numFormat.setMinimumFractionDigits(1);
      numFormat.setGroupingUsed(false);
      jTextField = new JTextField("0.0");
      jTextField.setHorizontalAlignment(SwingConstants.LEFT);
      jTextField.addActionListener(yoEntryBox);
      jTextField.addFocusListener(yoEntryBox);


      yoEntryBox.add(label);
      yoEntryBox.add(jTextField);
   }

   @Override
   public void shutdown(YoEntryBox yoEntryBox)
   {
      numFormat = null;
      variableInThisBox = null;
      yoEntryBox.remove(label);
      yoEntryBox.remove(jTextField);
      label = null;
      jTextField = null;
   }


   @Override
   public void bindToVariable(YoEntryBox yoEntryBox, YoVariable<?> variable)
   {
      variableInThisBox = variable;
      label.setText("  "+variable.getName()+"  ");

      String labelText = label.getText();

      Font labelFont = label.getFont();

      int stringWidth = label.getFontMetrics(labelFont).stringWidth(labelText);

      
//      updateJLabelTextSize(label,yoEntryBox);

      
      yoEntryBox.setPreferredSize(new Dimension(stringWidth + TEXT_BOX_WIDTH, BOX_HEIGHT));


   }

   private void updateJLabelTextSize(JLabel label, JComponent componentTOAddTo)
   {
      Font labelFont = label.getFont();
      String labelText = label.getText();

      int stringWidth = label.getFontMetrics(labelFont).stringWidth(labelText);
      int componentWidth = componentTOAddTo.getWidth() / 2;

      // Find out how much the font can grow in width.
      double widthRatio = (double) componentWidth / (double) stringWidth;

      System.out.println("OLD " + labelFont.getSize());

      int newFontSize = (int) (labelFont.getSize() * widthRatio);
      int componentHeight = componentTOAddTo.getHeight();

      System.out.println("componentHeight " + componentHeight);

      // Pick a new font size so it will not be larger than the height of label.
      int fontSizeToUse = Math.max(12, componentHeight);

      // Set the label's font size to the newly determined size.
      label.setFont(new Font(labelFont.getName(), Font.PLAIN, fontSizeToUse));
      System.out.println("New " + fontSizeToUse);

   }



   @Override
   public boolean isEventSource(YoEntryBox yoEntryBox, FocusEvent evt)
   {
      return evt.getSource().equals(jTextField);
   }

   @Override
   public void focusLost(YoEntryBox yoEntryBox)
   {
      update(yoEntryBox);
      jTextField.setCaretPosition(1);
   }

   @Override
   public void focusGained(YoEntryBox yoEntryBox)
   {
      update(yoEntryBox);
      jTextField.setCaretPosition(1);
   }

}
