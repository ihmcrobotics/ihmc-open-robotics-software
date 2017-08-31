package us.ihmc.simulationconstructionset.gui;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.robotics.MathTools;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * Created by Peter on 8/29/2017.
 */
public class JSliderParameterControl extends JPanel implements CloseableAndDisposable, MouseListener
{
   private static final int SLIDER_MIN_VALUE = 0;
   private static final int SLIDER_MAX_VALUE = 100;
   private YoVariable<?> yoVariable;

   private static final long serialVersionUID = 8570638563928914747L;

   private boolean changeLock = false;
   private JTextField jTextFieldValue;
   private JSlider jSlider;

   private YoVariableChangeListener yoVariableChangeListener;

   private String name;

   private SelectedVariableHolder selectedVariableHolder;

   private TextFieldListener valueTextFieldListener;

   private boolean sliderToBeSetWithoutNotifiers;

   private MinMaxTextEntryBox minTextEntryBox;
   private MinMaxTextEntryBox maxTextEntryBox;

   public JSliderParameterControl(SelectedVariableHolder selectedVariableHolder, CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      super(new BorderLayout());
      this.setMinimumSize(new Dimension(200, 400));
      //TODO: how to handle the different types of Parameters and the possible min/max values?

      JPanel mainPanel = new JPanel(new BorderLayout());
      JPanel minMaxTextFields = new JPanel(new BorderLayout());

      jSlider = new JSlider(SwingConstants.VERTICAL, SLIDER_MIN_VALUE, SLIDER_MAX_VALUE, 50);

      mainPanel.add(minMaxTextFields, BorderLayout.CENTER);
      mainPanel.add(jSlider, BorderLayout.EAST);

      JTextField jTextFieldMin = new JTextField(10);

      minTextEntryBox = new MinMaxTextEntryBox(this, jTextFieldMin);
      minTextEntryBox.setValue(-1000.0);
      jTextFieldMin.addActionListener(minTextEntryBox);

      JTextField jTextFieldMax = new JTextField(10);

      maxTextEntryBox = new MinMaxTextEntryBox(this, jTextFieldMax);
      maxTextEntryBox.setValue(1000.0);
      jTextFieldMax.addActionListener(maxTextEntryBox);

      minMaxTextFields.add(jTextFieldMax, BorderLayout.NORTH);
      minMaxTextFields.add(jTextFieldMin, BorderLayout.SOUTH);

      yoVariable = null;
      setTitle();

      jSlider.setPaintLabels(false);
      jSlider.addChangeListener(new SliderChangeListener(jSlider, this));

      jSlider.setEnabled(false);

      this.selectedVariableHolder = selectedVariableHolder;

      jTextFieldValue = new JTextField(String.valueOf(1));

      valueTextFieldListener = new TextFieldListener(this);
      jTextFieldValue.addActionListener(valueTextFieldListener);

      this.add(mainPanel, BorderLayout.CENTER);

      this.add(jTextFieldValue, BorderLayout.SOUTH);

      closeableAndDisposableRegistry.registerCloseableAndDisposable(this);

      mainPanel.addMouseListener(this);
   }

   private boolean getSliderToBeSetWithoutNotifiers()
   {
      return sliderToBeSetWithoutNotifiers;
   }

   private void setSliderToBeSetWithoutNotifiers(boolean value)
   {
      sliderToBeSetWithoutNotifiers = value;
   }

   public String getName()
   {
      if (yoVariable != null)
      {
         return yoVariable.getName();
      }
      else
      {
         return "Not Set";
      }
   }

   @Override
   public void closeAndDispose()
   {

   }

   private void setTitle()
   {
      this.setBorder(new TitledBorder(this.getName()));
      this.repaint();
   }

   synchronized public void lock()
   {
      if (!changeLock)
      {
         Thread timer = new Thread(new Runnable()
         {
            @Override
            public void run()
            {
               changeLock = true;

               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
               }

               changeLock = false;
            }
         });
         timer.start();
      }
   }

   @Override
   public void mouseClicked(MouseEvent e)
   {
   }

   @Override
   public void mousePressed(MouseEvent e)
   {
      if (e.getButton() == MouseEvent.BUTTON1)
      {
         //System.out.println(", BUTTON1");
      }
      else if (e.getButton() == MouseEvent.BUTTON2)
      {
         YoVariable yoVariable = selectedVariableHolder.getSelectedVariable();

         //         yoVariable.

         if (yoVariable != null && yoVariable.isParameter())
         {
            //check if there is a current yoVariable
            if (this.yoVariable != null)
            {
               //clear this as a change listener
               this.yoVariable.removeVariableChangedListener(yoVariableChangeListener);
            }

            jSlider.setEnabled(true);

            this.yoVariable = yoVariable;
            setTitle();

            //add change listener
            addListenersForYoVariable();

            setMinMaxValuesFromYoVariable();

            setTextFieldDouble(this.yoVariable.getValueAsDouble());

         }

      }
      else if (e.getButton() == MouseEvent.BUTTON3)
      {
      }
   }

   private void setMinMaxValuesFromYoVariable()
   {
      minTextEntryBox.setValue(this.yoVariable.getManualScalingMin());
      maxTextEntryBox.setValue(this.yoVariable.getManualScalingMax());
      setSliderPosition(convertParameterValueToSlider(this.yoVariable.getValueAsDouble()));
   }

   private boolean areMinMaxValid()
   {
      return minTextEntryBox.getValue() < maxTextEntryBox.getValue();
   }

   private void addListenersForYoVariable()
   {
      yoVariableChangeListener = new YoVariableChangeListener(this);
      this.yoVariable.addVariableChangedListener(yoVariableChangeListener);
   }

   @Override
   public void mouseReleased(MouseEvent e)
   {
      //System.out.print("mouseReleased in slider " + name);
   }

   @Override
   public void mouseEntered(MouseEvent e)
   {
   }

   @Override
   public void mouseExited(MouseEvent e)
   {
   }

   private void setYoVariable(double value)
   {
      this.yoVariable.setValueFromDouble(value, false);

      //We do not want to notify ourself that the variable changed
      ArrayList<VariableChangedListener> listOfListeners = this.yoVariable.getVariableChangedListeners();

      for (VariableChangedListener variableChangedListener : listOfListeners)
      {
         if (variableChangedListener != yoVariableChangeListener)
         {
            variableChangedListener.variableChanged(this.yoVariable);
         }
      }
   }

   private double convertSliderToParameterValue(int sliderValue)
   {
      double percentOfRange = ((double) (sliderValue - SLIDER_MIN_VALUE)) / ((double) (SLIDER_MAX_VALUE - SLIDER_MIN_VALUE));
      double actualValue = minTextEntryBox.getValue() + percentOfRange * (maxTextEntryBox.getValue() - minTextEntryBox.getValue());

      return actualValue;
   }

   private int convertParameterValueToSlider(double value)
   {
      double percentOfRange = (value - minTextEntryBox.getValue()) / (maxTextEntryBox.getValue() - minTextEntryBox.getValue());
      percentOfRange = MathTools.clamp(percentOfRange, 0.0, 1.0);
      int actualValue = SLIDER_MIN_VALUE + ((int) (percentOfRange * (SLIDER_MAX_VALUE - SLIDER_MIN_VALUE)));

      return actualValue;
   }

   private double getTextFieldValue()
   {
      return Double.parseDouble(jTextFieldValue.getText());
   }

   private void setTextFieldDouble(double value)
   {
      String formattedValue = new DecimalFormat("#.#####").format(value);
      jTextFieldValue.setText(formattedValue);
   }

   private void setSliderPosition(int sliderValue)
   {
      this.setSliderToBeSetWithoutNotifiers(true);

      sliderValue = MathTools.clamp(sliderValue, SLIDER_MIN_VALUE, SLIDER_MAX_VALUE);
      jSlider.setValue(sliderValue);

      this.setSliderToBeSetWithoutNotifiers(false);
   }

   private class SliderChangeListener implements ChangeListener
   {
      JSlider slider;
      JSliderParameterControl jSliderParameterControl;

      public SliderChangeListener(JSlider slider, JSliderParameterControl jSliderParameterControl)
      {
         this.slider = slider;
         this.jSliderParameterControl = jSliderParameterControl;
      }

      @Override
      public void stateChanged(ChangeEvent e)
      {
         if (!jSliderParameterControl.getSliderToBeSetWithoutNotifiers())
         {

            int sliderValue = slider.getValue();

            if (yoVariable != null)
            {
               double value = convertSliderToParameterValue(sliderValue);
               setYoVariable(value);
            }

            setTextFieldDouble(convertSliderToParameterValue(sliderValue));
         }
      }
   }

   private void setSliderEnable(boolean value)
   {
      jSlider.setEnabled(value);
   }

   private class YoVariableChangeListener implements VariableChangedListener
   {
      JSliderParameterControl jSliderParameterControl;

      public YoVariableChangeListener(JSliderParameterControl jSliderParameterControl)
      {
         this.jSliderParameterControl = jSliderParameterControl;
      }

      @Override
      public void variableChanged(YoVariable<?> v)
      {
         //change slider
         jSliderParameterControl.setSliderPosition(convertParameterValueToSlider(v.getValueAsDouble()));

         //change text box
         setTextFieldDouble(v.getValueAsDouble());
      }
   }

   private class MinMaxTextEntryBox implements ActionListener
   {
      JSliderParameterControl jSliderParameterControl;
      MutableDouble valueToUpdate = new MutableDouble();
      JTextField jTextField;

      public MinMaxTextEntryBox(JSliderParameterControl jSliderParameterControl, JTextField jTextField)
      {
         this.jSliderParameterControl = jSliderParameterControl;
         this.jTextField = jTextField;
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         try
         {
            double value = Double.parseDouble(jTextField.getText());
            valueToUpdate.setValue(value);

            //Set slider
            jSliderParameterControl.setSliderPosition(jSliderParameterControl.convertParameterValueToSlider(yoVariable.getValueAsDouble()));

            setSliderEnable(areMinMaxValid());
         }
         catch (Exception ex)
         {
            jTextField.setText(String.valueOf(getValue()));
         }
      }

      public void setValue(double value)
      {
         valueToUpdate.setValue(value);
         jTextField.setText(String.valueOf(valueToUpdate.getValue()));
      }

      public double getValue()
      {
         return valueToUpdate.getValue();
      }
   }

   private class TextFieldListener implements ActionListener
   {
      JSliderParameterControl jSliderParameterControl;

      public TextFieldListener(JSliderParameterControl jSliderParameterControl)
      {
         this.jSliderParameterControl = jSliderParameterControl;
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         try
         {
            double value = jSliderParameterControl.getTextFieldValue();

            //Set slider
            jSliderParameterControl.setSliderPosition(jSliderParameterControl.convertParameterValueToSlider(value));

            //set yoVaiable
            setYoVariable(value);
         }
         catch (Exception ex)
         {
            setTextFieldDouble(yoVariable.getValueAsDouble());
         }
      }
   }
}
