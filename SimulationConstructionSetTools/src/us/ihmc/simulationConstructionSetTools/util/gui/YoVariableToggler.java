package us.ihmc.simulationConstructionSetTools.util.gui;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.simulationconstructionset.NewDataListener;

public class YoVariableToggler implements NewDataListener
{
   private YoVariableToggleContainer parentContainer;

   private final EnumYoVariable<ToggleMode> toggleMode;
   public BooleanYoVariable currentState = null;

   private boolean currentStateValue;

   private String trueString = "Turn On";
   private String falseString = "Turn Off";
   private boolean firstRun = true;

   public YoVariableToggler(String name, YoVariableRegistry parent, YoVariableToggleContainer parentContainer, BooleanYoVariable currentStateVariable)
   {
      if (currentStateVariable != null)
         this.currentState = currentStateVariable;
      this.parentContainer = parentContainer;
      toggleMode = (EnumYoVariable<ToggleMode>) parent.getVariable(name);

      toggleMode.set(ToggleMode.NO_CHANGE);

      if (currentState != null)
         currentStateValue = currentState.getBooleanValue();
   }


   private synchronized void checkForStateChange()
   {
      if (currentState != null)
      {
         synchronized (currentState)
         {
            if (firstRun)
            {
               toggleMode.set(ToggleMode.NO_CHANGE);
               firstRun = false;
            }

            if (currentState.getBooleanValue() != currentStateValue)
            {
               // a change has happened
               toggleMode.set(ToggleMode.NO_CHANGE);
               parentContainer.handleStateChange();

            }

            currentStateValue = currentState.getBooleanValue();
         }
      }

   }

   public void toggle()
   {
      if (currentStateValue)
      {
         setFalse();
         parentContainer.processingStateChange(false);
      }
      else
      {
         setTrue();
         parentContainer.processingStateChange(true);
      }
   }

   private void setTrue()
   {
      toggleMode.set(ToggleMode.SET_TRUE);
   }

   private void setFalse()
   {
      toggleMode.set(ToggleMode.SET_FALSE);

   }


   public String getTrueString()
   {
      return trueString;
   }



   public void setTrueString(String trueString)
   {
      this.trueString = trueString;
   }



   public String getFalseString()
   {
      return falseString;
   }



   public void setFalseString(String falseString)
   {
      this.falseString = falseString;
   }

   public String getNextStateString()
   {
      if (currentState == null)
      {
         return getTrueString();
      }

      if (currentStateValue)
         return getTrueString();
      else
         return getFalseString();
   }

   @Override
   public void newDataHasBeenSent()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void newDataHasBeenReceived()
   {
      checkForStateChange();
   }


   public String getCurrentStateString()
   {
      if (currentStateValue)
      {
         return getFalseString();
      }

      return getTrueString();
   }


   public void registerWithVariableChangedListener(VariableChangedListener changedListener)
   {
      toggleMode.addVariableChangedListener(changedListener);
   }

}
