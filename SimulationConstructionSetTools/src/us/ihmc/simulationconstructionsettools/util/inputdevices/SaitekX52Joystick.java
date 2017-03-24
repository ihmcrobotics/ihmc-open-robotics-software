package us.ihmc.simulationconstructionsettools.util.inputdevices;

import java.io.IOException;
import java.util.ArrayList;

import net.java.games.input.Component;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionsettools.joystick.BooleanYoVariableJoystickEventListener;
import us.ihmc.simulationconstructionsettools.joystick.DoubleYoVariableJoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.SaitekX52Mapping;

/**
 * @deprecated Use us.ihmc.tools.inputDevices.joystick.Joystick
 */
public class SaitekX52Joystick
{
   private static final int DEFAULT_POLL_INTERVAL_MS = 100;
   
   private Joystick joystick;
   
   private final ArrayList<JoystickEventListener> eventListeners = new ArrayList<>();
   
   // Axes
   // X, Y is the big joystick.
   // R is yaw* probably
   // Z is the throttle.
   // U is the linear thumb slider.
   // V is the thumb knob around the i button.
   // POV is the Large joystick thumb joystick, but doesn't seem to work right.

   // Holder
   protected YoVariableHolder holder;

   public SaitekX52Joystick()
   {
      try
      {
         joystick = new Joystick();
         
         if (!joystick.getModel().equals(JoystickModel.SAITEK_X52))
         {
            throw new IOException("Not SaitekX52");
         }
         
         joystick.setPollInterval(DEFAULT_POLL_INTERVAL_MS);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void printJoystickInfo()
   {

   }

   public void setPollInterval(int milliseconds)
   {
      joystick.setPollInterval(milliseconds);
   }
   
   public void mapDoubleVariableToComponent(YoVariableHolder holder, SaitekX52Mapping mapping, String variableName, double minValue, double maxValue, double deadZone, boolean invert)
   {
      DoubleYoVariable yoVariable = (DoubleYoVariable) holder.getVariable(variableName);
      if (yoVariable != null)
      {
         mapDoubleVariableToComponent(mapping, yoVariable, minValue, maxValue, deadZone, invert);
      }
      else
      {
         PrintTools.warn(this, "Variable " + variableName + " could not be found!");
      }
   }

   public void mapDoubleVariableToComponent(SaitekX52Mapping mapping, DoubleYoVariable yoVariable, double minValue, double maxValue, double deadZone, boolean invert)
   {
      Component component = joystick.findComponent(mapping.getIdentifier());
      DoubleYoVariableJoystickEventListener doubleYoVariableJoystickEventListener = new DoubleYoVariableJoystickEventListener(yoVariable, component, minValue, maxValue, deadZone, invert);
      eventListeners.add(doubleYoVariableJoystickEventListener);
      joystick.addJoystickEventListener(doubleYoVariableJoystickEventListener);
   }
   
   public void mapBooleanVariableToComponent(YoVariableHolder holder, SaitekX52Mapping mapping, String variableName, boolean toggle, boolean invert)
   {
      BooleanYoVariable yoVariable = (BooleanYoVariable) holder.getVariable(variableName);
      if (yoVariable != null)
      {
         mapBooleanVariableToComponent(mapping, yoVariable, toggle, invert);
      }
      else
      {
         PrintTools.warn(this, "Variable " + variableName + " could not be found!");
      }
   }

   public void mapBooleanVariableToComponent(SaitekX52Mapping mapping, BooleanYoVariable yoVariable, boolean toggle, boolean invert)
   {
      Component component = joystick.findComponent(mapping.getIdentifier());
      BooleanYoVariableJoystickEventListener booleanYoVariableJoystickEventListener = new BooleanYoVariableJoystickEventListener(yoVariable, component, toggle, invert);
      eventListeners.add(booleanYoVariableJoystickEventListener);
      joystick.addJoystickEventListener(booleanYoVariableJoystickEventListener);
   }

   public void setButton(int buttonNumber, String name, YoVariableHolder holder)
   {
      if (buttonNumber == 0)
      {
         mapBooleanVariableToComponent(holder, SaitekX52Mapping.TRIGGER_PARITIAL, name, false, false);
      }
      else if (buttonNumber == 1)
      {
         mapBooleanVariableToComponent(holder, SaitekX52Mapping.A, name, false, false);
      }
      else if (buttonNumber == 2)
      {
         mapBooleanVariableToComponent(holder, SaitekX52Mapping.B, name, false, false);
      }
      else if (buttonNumber == 3)
      {
         mapBooleanVariableToComponent(holder, SaitekX52Mapping.C, name, false, false);
      }
      else if (buttonNumber == 4)
      {
         mapBooleanVariableToComponent(holder, SaitekX52Mapping.D, name, false, false);
      }
      else
      {
         throw new RuntimeException("Use mapBooleanVariableToComponent instead!");
      }
   }

   /** @deprecated */
   public void attachVariableChangedListener(VariableChangedListener listener)
   {      
      
   }

   public void mapDoubleToStickRoll(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.STICK_ROLL, name, min, max, 0.0, false);
   }

   public void setXAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      mapDoubleVariableToComponent(SaitekX52Mapping.STICK_ROLL, variable, min, max, 0.0, false);
   }

   public void setYAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.STICK_PITCH, name, min, max, 0.0, false);
   }

   public void setYAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      mapDoubleVariableToComponent(SaitekX52Mapping.STICK_PITCH, variable, min, max, 0.0, false);
   }

   public void setZAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.THROTTLE, name, min, max, 0.0, false);
   }

   public void setZAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      mapDoubleVariableToComponent(SaitekX52Mapping.THROTTLE, variable, min, max, 0.0, false);
   }

   public void setRAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.STICK_YAW, name, min, max, 0.0, false);
   }

   public void setRAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      mapDoubleVariableToComponent(SaitekX52Mapping.STICK_YAW, variable, min, max, 0.0, false);
   }

   public void setUAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.LEFT_THUMB_SLIDER, name, min, max, 0.0, false);
   }

   public void setVAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      mapDoubleVariableToComponent(holder, SaitekX52Mapping.BIG_ROTARY_KNOB, name, min, max, 0.0, false);
   }
}
