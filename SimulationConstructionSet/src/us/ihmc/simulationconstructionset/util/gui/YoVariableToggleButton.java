package us.ihmc.simulationconstructionset.util.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.simulationconstructionset.NewDataListener;

@SuppressWarnings("serial")
public class YoVariableToggleButton extends JButton implements YoVariableToggleContainer
{
   private final int MILLIS_TO_WAIT_FOR_RESPONSE = 2000;
   private YoVariableToggler buttonToggleState;

   private boolean stateChanged = false;

   public YoVariableToggleButton(String outletName, BooleanYoVariable currentStateVariable, YoVariableRegistry parent)
   {
      buttonToggleState = new YoVariableToggler(outletName, parent, this, currentStateVariable);
      this.setText(buttonToggleState.getCurrentStateString());

      this.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent arg0)
         {
            buttonToggleState.toggle();
         }
      });
   }

   @Override
   public void handleStateChange()
   {
      this.setText(buttonToggleState.getNextStateString());
      stateChanged = true;

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
      stateChanged = false;
      final String currentStateString = buttonToggleState.getCurrentStateString();

//    System.out.println("endStateValue " + endStateValue + " currentStateString " + currentStateString);

      if (endStateValue)
      {
         // this.setText("Attempting to " + buttonToggleState.getTrueString());
      }
      else
      {
         // this.setText("Attempting to " + buttonToggleState.getFalseString());
      }

      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            setEnabled(false);

            try
            {
               Thread.sleep(MILLIS_TO_WAIT_FOR_RESPONSE);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

            if (!stateChanged)
            {
               setText(currentStateString);
            }

            setEnabled(true);
         }
      }).start();
   }

   public void setTrueString(String trueString)
   {
      buttonToggleState.setTrueString(trueString);
      this.setText(buttonToggleState.getCurrentStateString());

   }

   public void setFalseString(String falseString)
   {
      buttonToggleState.setFalseString(falseString);
      this.setText(buttonToggleState.getCurrentStateString());

   }

   @Override
   public void registerWithVariableChangedListener(VariableChangedListener changedListener)
   {
      buttonToggleState.registerWithVariableChangedListener(changedListener);
   }
}
