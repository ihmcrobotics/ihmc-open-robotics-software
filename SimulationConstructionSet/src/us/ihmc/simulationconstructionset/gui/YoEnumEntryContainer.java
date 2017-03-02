package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import javax.swing.BoxLayout;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.SwingConstants;

import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class YoEnumEntryContainer implements YoVariableEntryContainer, ActionListener
{
   private static final int ENTRY_BOX_HEIGHT = YoEntryBox.COMPONENT_HEIGHT;
   private static final int HORIZONTAL_LABEL_BORDER = 6;
   private static final int MAX_TOTAL_LENGTH = YoEntryBox.MAX_COMPONENT_LENGTH;
   private static final int HORIZONTAL_ENTRYBOX_SLOP = 6;
   private static final int COMBO_BOX_MIN_LENGTH = 28;
   
   private EnumYoVariable<?> variableInThisBox;
//   private LinkedHashMap<E, String> enumToNameMap;
//   private LinkedHashMap<Integer, E> indexToEnumMap;
//   private LinkedHashMap<E, Integer> enumToIndexMap;
   private YoEntryBox yoEntryBox;
   private JLabel label;
   private JComboBox<String> comboBox;


   protected YoEnumEntryContainer(String[] enumValues)
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
      if (!comboBox.hasFocus())
      {
         if (variableInThisBox != null)
         {
            if (comboBox.getSelectedIndex() != getIndexOf(variableInThisBox.getOrdinal()))
               comboBox.setSelectedIndex(getIndexOf(variableInThisBox.getOrdinal()));
         }
      }
   }

   @Override
   public void actionPerformed(YoEntryBox yoEntryBox, ActionEvent evt)
   {
//      System.out.println("YoEnumEntryContainer: someone gave me an actionPerformed!");
   }


   @Override
   public void removeVariable(YoVariable<?> variable)
   {
      if (getVariable() == variable)
         variableInThisBox = null;

      label.setText(YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
      label.setName(""); //YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
   }


   @Override
   public void setup(YoEntryBox yoEntryBox)
   {
      this.yoEntryBox = yoEntryBox;
      label = new JLabel(YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
//      label.setName(YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
      label.setHorizontalAlignment(SwingConstants.RIGHT);
      comboBox = new JComboBox<String>(new String[]{YoEntryBox.DEFAULT_UNBOUND_ENTRY_BOX_LABEL});
      this.yoEntryBox = yoEntryBox;

      BoxLayout mgr = new BoxLayout(yoEntryBox, BoxLayout.X_AXIS);
      yoEntryBox.setLayout(mgr);
      yoEntryBox.add(label);
      yoEntryBox.add(comboBox);
   }

   @Override
   public void shutdown(YoEntryBox yoEntryBox)
   {
      variableInThisBox = null;
      yoEntryBox.remove(label);
      yoEntryBox.remove(comboBox);
      label = null;
      comboBox = null;
   }

   @Override
   public void bindToVariable(YoEntryBox yoEntryBox, YoVariable<?> variable)
   {
      if (variable instanceof EnumYoVariable<?>)
      {
         EnumYoVariable<?> enumYoVariable = (EnumYoVariable<?>) variable;
//         if (enumYoVariable.getEnumType() == enumType)
//         {
            variableInThisBox = (EnumYoVariable<?>)enumYoVariable;
            label.setText(variableInThisBox.getName());
            labelTextLength = label.getFontMetrics(label.getFont()).stringWidth(label.getText());
            setupComboBox();
//         }
//         else
//         {
//            throw new RuntimeException("Variable " + variable.getName() + " does not have correct enum type");
//         }
      }
      else
      {
         throw new RuntimeException("Cannot bind a YoEnumEntryContainer to a varible which is not an EnumYoVariable!");
      }
   }

   private void setupComboBox()
   {
      comboBox.setVisible(false);

      ArrayList<String> stringCollection = new ArrayList<>();
      stringCollection.addAll(Arrays.asList(this.variableInThisBox.getEnumValuesAsString()));
      if(this.variableInThisBox.getAllowNullValue())
      {
         stringCollection.add("null");
      }
      
      calculateStringFoldingParameters(stringCollection);
      String[] stringsArray = new String[stringCollection.size()];
      stringCollection.toArray(stringsArray);
      comboBox = new JComboBox<String>(stringsArray);

      comboBox.setSelectedIndex(getIndexOf(variableInThisBox.getOrdinal()));
      yoEntryBox.add(comboBox);
      comboBox.addActionListener(this);
      //comboBox.setFocusable(false);
      doLayout();
   }

   private void doLayout()
   {
      int desiredLabelLength = labelTextLength + HORIZONTAL_LABEL_BORDER;
     
      Dimension labelDimension = new Dimension(desiredLabelLength, ENTRY_BOX_HEIGHT);
      label.setPreferredSize(labelDimension);
      int remainingSpace = MAX_TOTAL_LENGTH - desiredLabelLength;
      int desiredComboBoxLength = pixelLengthOfLongestComboBoxEntry + COMBO_BOX_MIN_LENGTH;
      if (remainingSpace < desiredComboBoxLength)
         desiredComboBoxLength = remainingSpace;
      Dimension comboBoxDimension = new Dimension(desiredComboBoxLength, ENTRY_BOX_HEIGHT);
      comboBox.setPreferredSize(comboBoxDimension);
      this.yoEntryBox.setPreferredSize(new Dimension(desiredLabelLength + desiredComboBoxLength + HORIZONTAL_ENTRYBOX_SLOP, ENTRY_BOX_HEIGHT));
   }

   private int calculateStringLengthInComboBox(String string)
   {
      return comboBox.getFontMetrics(comboBox.getFont()).stringWidth(string);
   }

   @Override
   public boolean isEventSource(YoEntryBox yoEntryBox, FocusEvent evt)
   {
      return evt.getSource().equals(comboBox);
   }

   @Override
   public void focusLost(YoEntryBox yoEntryBox)
   {
   }

   @Override
   public void focusGained(YoEntryBox yoEntryBox)
   {
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      int selectedIndex = comboBox.getSelectedIndex();
      int ordinal = getOrdinalOf(selectedIndex);
      variableInThisBox.set(ordinal, true);
      YoEntryBox.informVariableChangedListeners(getVariable());

      // System.out.println("YoEnumEntryContainer: actionPerformed ActionEvent e has source: "+e.getSource());
//      System.out.println("YoEnumEntryContainer: Hey, I haz an actionPerformed!");

      // modifier = 16 means mouse click generated action. Other than that is key based. enter key does not generate an action, when it really should.
   }

//   private void setUpMaps(Class<E> enumType)
//   {
//      E[] enumConstants = enumType.getEnumConstants();
//      enumToNameMap = new LinkedHashMap<E, String>();
//      indexToEnumMap = new LinkedHashMap<Integer, E>();
//      enumToIndexMap = new LinkedHashMap<E, Integer>();
//      int i = 0;
//      for (E enumConstant : enumConstants)
//      {
//         enumToNameMap.put(enumConstant, enumConstant.toString());
//         indexToEnumMap.put(i, enumConstant);
//         enumToIndexMap.put(enumConstant, i);
//         i++;
//      }
//      if (this.variableInThisBox.getAllowNullValue())
//      {
//         enumToNameMap.put(null, "null");
//         indexToEnumMap.put(i, null);
//         enumToIndexMap.put(null, i);
//      }
//   }

   private int pixelLengthOfLongestComboBoxEntry = 10;
   public static final int STRING_LENGTH_CAP = 10;
   public static final int STRING_LENGTH_LOWER_BOUND = 5;
   public static final int STRING_TRUNCATION_FUNCTION_START_LENGTH = 10;
   private int labelTextLength;

   public void calculateStringFoldingParameters(Collection<String> strings)
   {
      int maxSize = 0;
      for (String string : strings)
      {
         int stringLength = calculateStringLengthInComboBox(string);
         if (stringLength > maxSize)
            maxSize = stringLength;
      }

      pixelLengthOfLongestComboBoxEntry = maxSize;
   }
   
   private int getOrdinalOf(int index)
   {
      if (this.variableInThisBox.getAllowNullValue())
      {
         if(index == this.variableInThisBox.getEnumSize())
         {
            return -1;
         }
      }
      return index;
   }

   private int getIndexOf(int ordinal)
   {
      if (this.variableInThisBox.getAllowNullValue())
      {
         if(ordinal == EnumYoVariable.NULL_VALUE)
         {
            return this.variableInThisBox.getEnumSize();
         }
      }
      return ordinal;
   }
}
