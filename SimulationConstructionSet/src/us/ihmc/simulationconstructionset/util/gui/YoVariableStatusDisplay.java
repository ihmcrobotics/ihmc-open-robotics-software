package us.ihmc.simulationconstructionset.util.gui;

import java.awt.Color;
import java.net.URL;

import javax.swing.ImageIcon;
import javax.swing.JLabel;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.simulationconstructionset.NewDataListener;

@SuppressWarnings("serial")
public class YoVariableStatusDisplay extends JLabel implements YoVariableToggleContainer
{
   private YoVariableToggler buttonToggleState;


   private boolean useImageTrue = false;
   private boolean useImageFalse = false;

   private Color trueColor = Color.GREEN;
   private Color falseColor = Color.RED;
   private ImageIcon trueIcon;
   private ImageIcon falseIcon;

   
   public YoVariableStatusDisplay(String name, YoVariableRegistry parent, String currentStateVariableName)
   {
      this(name, parent, (BooleanYoVariable) parent.getVariable(currentStateVariableName));
   }

   public YoVariableStatusDisplay(String name, YoVariableRegistry parent, BooleanYoVariable currentStateVariable)
   {
      YoVariableTogglerOutlet nullOutlet = new YoVariableTogglerOutlet(name, parent);

      buttonToggleState = new YoVariableToggler(name, parent, this, currentStateVariable);
      this.setText(buttonToggleState.getCurrentStateString());
      setTrueString("ON", Color.GREEN);
      setFalseString("OFF", Color.RED);

//    this.setEnabled(false);
   }

   @Override
   public void handleStateChange()
   {
      this.setText(buttonToggleState.getNextStateString());

      if (buttonToggleState.currentState.getBooleanValue())
      {
         if (useImageTrue)
            this.setIcon(trueIcon);
         else
            this.setForeground(falseColor);

      }
      else
      {
         if (useImageFalse)
            this.setIcon(falseIcon);
         else
            this.setForeground(trueColor);
      }

//    setEnabled(true);
   }

   @Override
   public NewDataListener getDataListener()
   {
      return buttonToggleState;
   }

   @Override
   public void processingStateChange(boolean endStateValue)
   {
      try
      {
         throw new Exception("this should not be called");
      }
      catch (Exception e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public void setTrueString(String trueString, Color color)
   {
      falseColor = color;
      buttonToggleState.setFalseString(trueString);
      this.setText(buttonToggleState.getCurrentStateString());
      if (buttonToggleState.getCurrentStateString().equals(trueString))
         this.setForeground(falseColor);

   }

   public void setFalseString(String falseString, Color color)
   {
      trueColor = color;
      buttonToggleState.setTrueString(falseString);
      this.setText(buttonToggleState.getCurrentStateString());
      if (buttonToggleState.getCurrentStateString().equals(falseString))
         this.setForeground(trueColor);

   }

   public void setTrueIcon(URL fileLocation)
   {
      trueIcon = new ImageIcon(fileLocation);
      useImageTrue = true;
      this.setIcon(trueIcon);

   }

   public void setFalseIcon(URL fileLocation)
   {
      falseIcon = new ImageIcon(fileLocation);
      useImageFalse = true;
      this.setIcon(falseIcon);

   }

   @Override
   public void registerWithVariableChangedListener(VariableChangedListener changedListener)
   {
      buttonToggleState.registerWithVariableChangedListener(changedListener);
   }
}
