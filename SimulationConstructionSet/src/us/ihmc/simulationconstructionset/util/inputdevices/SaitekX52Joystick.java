package us.ihmc.simulationconstructionset.util.inputdevices;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

import com.centralnexus.input.Joystick;
import com.centralnexus.input.JoystickListener;

/**
 * <p>
 * Title: SimulationConstructionSet
 * </p>
 *
 * <p>
 * Description:
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2000
 * </p>
 *
 * <p>
 * Company: Yobotics, Inc.
 * </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SaitekX52Joystick implements JoystickListener
{
   protected Joystick joystick;

   // Axes
   // X, Y is the big joystick.
   // Z is the throttle.
   // U is the linear thumb slider.
   // V is the thumb knob around the i button.
   // POV is the Large joystick thumb joystick, but doesn't seem to work right.
   // The large knob around the E button doesn't seem to work...

   public static final int
      X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2, R_AXIS = 3, U_AXIS = 4, V_AXIS = 5, POV_AXIS = 6;
   private static final int NUM_AXES = 7;

   private double[] axesMinVals = new double[NUM_AXES];
   private double[] axesMaxVals = new double[NUM_AXES];
   private double[] axesExponents = new double[NUM_AXES];

   private double[] rawAxesValues = new double[NUM_AXES];
   private double[] oldRawAxesValues = new double[NUM_AXES];

   private DoubleYoVariable[] axesVariables = new DoubleYoVariable[NUM_AXES];
   private String[] axesNames = new String[NUM_AXES];

   // Buttons
   protected static final int NUM_BUTTONS = 32;

   private boolean[] oldButtonDowns = new boolean[NUM_BUTTONS];
   private BooleanYoVariable[] buttonVariables = new BooleanYoVariable[NUM_BUTTONS];
   private String[] buttonNames = new String[NUM_BUTTONS];

   private int oldModeDial;
   private DoubleYoVariable modeDialVariable;
   private String modeDialName;

   // Holder
   protected YoVariableHolder holder;

   public SaitekX52Joystick()
   {
      try
      {
         joystick = Joystick.createInstance();

         joystick.setPollInterval(100);
         joystick.setDeadZone(0.05);
         joystick.addJoystickListener(this);
      }
      catch (Exception ex)
      {
         System.err.println("Could not connect to joystick");
//         System.err.println("Exception Thrown when trying to connect to the SaitekX52 Joystick: " + ex);
//         ex.printStackTrace();
      }
      catch (UnsatisfiedLinkError ex)
      {
         System.err.println("UnsatisfiedLinkError thrown when trying to connect to the SaitekX52 Joystick: " + ex);
//         ex.printStackTrace();
         System.err.println("Joystick library may not be available on your platform.");
      }
   }

   public void printJoystickInfo()
   {
      if (joystick == null)
         return;

      int numButtons = joystick.getNumButtons();
      int numAxes = joystick.getNumAxes();

      System.out.println("This joystick has " + numButtons + " buttons and " + numAxes + " axes");

      int capabilities = joystick.getCapabilities();
      System.out.println("Capabilities: " + capabilities);

      if (joystick.getCapability(Joystick.HAS_R))
         System.out.println("Joystick has R");
         ;
      if (joystick.getCapability(Joystick.HAS_U))
         System.out.println("Joystick has U");
         ;
      if (joystick.getCapability(Joystick.HAS_V))
         System.out.println("Joystick has V");
         ;
      if (joystick.getCapability(Joystick.HAS_Z))
         System.out.println("Joystick has Z");
         ;
      if (joystick.getCapability(Joystick.HAS_POV))
         System.out.println("Joystick has POV");
         ;
      if (joystick.getCapability(Joystick.HAS_POV4DIR))
         System.out.println("Joystick has POV4DIR");
         ;
      if (joystick.getCapability(Joystick.HAS_POVCONT))
         System.out.println("Joystick has POVCONT");
         ;

   }

   public void setPollInterval(int milliseconds)
   {
      if (joystick == null)
         return;

      joystick.setPollInterval(milliseconds);
   }

   private ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   public void attachVariableChangedListener(VariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }

   public void setAxis(int axisID, String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      if (joystick == null)
         return;

      this.holder = holder;

      DoubleYoVariable variable = null;
      if (holder != null)
         variable = (DoubleYoVariable) holder.getVariable(name);

      setAxis(axisID, name, variable, min, max, exponent);
   }

   private void setAxis(int axisID, String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      if (joystick == null)
         return;

      // System.out.println("Setting axis " + axisID + " to name " + name +
      // " with variable " + variable);

      axesMinVals[axisID] = min;
      axesMaxVals[axisID] = max;
      axesExponents[axisID] = exponent;
      axesNames[axisID] = name;
      axesVariables[axisID] = variable;
   }

   public void setButton(int buttonNumber, String name, YoVariableHolder holder)
   {
      if (joystick == null)
         return;

      this.holder = holder;

      BooleanYoVariable variable = null;
      if (holder != null)
         variable = (BooleanYoVariable) holder.getVariable(name);

      setButton(buttonNumber, name, variable);
   }

   public void setButton(int buttonNumber, String name, BooleanYoVariable variable)
   {
      if (joystick == null)
         return;

      // System.out.println("Setting axis " + axisID + " to name " + name +
      // " with variable " + variable);

      buttonNames[buttonNumber] = name;
      buttonVariables[buttonNumber] = variable;
   }

   public void setModeDial(String name, DoubleYoVariable variable)
   {
      if (joystick == null)
         return;

      this.modeDialName = name;
      this.modeDialVariable = variable;
   }

   public void setModeDial(String name, YoVariableHolder holder)
   {
      if (joystick == null)
         return;

      this.holder = holder;

      DoubleYoVariable variable = null;
      if (holder != null)
         variable = (DoubleYoVariable) holder.getVariable(name);

      setModeDial(name, variable);
   }

   public void joystickAxisChanged(Joystick joystick)
   {
      if (joystick == null)
         return;

      // System.out.println("Joystick Axis Changed");

      rawAxesValues[X_AXIS] = joystick.getX();
      rawAxesValues[Y_AXIS] = joystick.getY();
      rawAxesValues[Z_AXIS] = joystick.getZ();
      rawAxesValues[R_AXIS] = joystick.getR();
      rawAxesValues[U_AXIS] = joystick.getU();
      rawAxesValues[V_AXIS] = joystick.getV();
      rawAxesValues[POV_AXIS] = joystick.getPOV();

      for (int axis = 0; axis < NUM_AXES; axis++)
      {
         // System.out.println("axis " + axis + " is at " + rawAxesValues[axis]);
         DoubleYoVariable variable = axesVariables[axis];
         if (variable == null)
         {
            if ((holder != null) && (axesNames[axis] != null))
               axesVariables[axis] = variable = (DoubleYoVariable) holder.getVariable(axesNames[axis]);
         }

         if ((variable != null) && (rawAxesValues[axis] != oldRawAxesValues[axis]))
         {
            // System.out.println("New raw value: " + rawAxesValues[i] + " for "
            // + names[i]);
            axesVariables[axis].set(interpretAxisValue(axesMinVals[axis], axesMaxVals[axis], axesExponents[axis], rawAxesValues[axis]));

            oldRawAxesValues[axis] = rawAxesValues[axis];

            notifyVariableChangedListeners(variable);
         }
      }
   }

   private double interpretAxisValue(double min, double max, double exponent, double axisValue)
   {
      double alpha = axisValue;
      double beta = Math.pow(Math.abs(alpha), exponent);
      if (alpha < 0.0)
         beta = -beta;

      double newValue = (min + max) / 2.0 + (max - min) / 2.0 * beta;

      // double newValue = axisValue;

      return newValue;
   }

   public void joystickButtonChanged(Joystick joystick)
   {
      if (joystick == null)
         return;

      // System.out.println("Joystick Button Changed");

      for (int button = 0; button < NUM_BUTTONS; button++)
      {
         BooleanYoVariable variable = buttonVariables[button];
         if (variable == null)
         {
            if ((holder != null) && (buttonNames[button] != null))
               buttonVariables[button] = variable = (BooleanYoVariable) holder.getVariable(buttonNames[button]);
         }

         boolean isButtonDown = joystick.isButtonDown(0x01 << button);

//       if (isButtonDown)
//          System.out.println("Button " + button + " is down");

         if ((variable != null) && (isButtonDown != oldButtonDowns[button]))
         {
            buttonVariables[button].set(isButtonDown);
            oldButtonDowns[button] = isButtonDown;

            notifyVariableChangedListeners(variable);
         }
      }

      if ((modeDialName != null) && (modeDialVariable == null))
      {
         modeDialVariable = (DoubleYoVariable) holder.getVariable(modeDialName);

         if (joystick.isButtonDown(0x01 << 23))
            oldModeDial = 0;
         else if (joystick.isButtonDown(0x01 << 24))
            oldModeDial = 1;
         else if (joystick.isButtonDown(0x01 << 25))
            oldModeDial = 2;
      }

      if (modeDialVariable != null)
      {
         int modeDial = -1;

         if (joystick.isButtonDown(0x01 << 23))
            modeDial = 0;
         else if (joystick.isButtonDown(0x01 << 24))
            modeDial = 1;
         else if (joystick.isButtonDown(0x01 << 25))
            modeDial = 2;

         if (oldModeDial != modeDial)
         {
            modeDialVariable.set(modeDial);
            oldModeDial = modeDial;

            notifyVariableChangedListeners(modeDialVariable);
         }
      }

   }

   public void setXAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(X_AXIS, name, holder, min, max, exponent);
   }

   public void setXAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(X_AXIS, name, holder, min, max, 1.0);
   }

   public void setXAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(X_AXIS, name, variable, min, max, exponent);
   }

   public void setXAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(X_AXIS, name, variable, min, max, 1.0);
   }

   public void setYAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(Y_AXIS, name, holder, min, max, exponent);
   }

   public void setYAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(Y_AXIS, name, holder, min, max, 1.0);
   }

   public void setYAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(Y_AXIS, name, variable, min, max, exponent);
   }

   public void setYAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(Y_AXIS, name, variable, min, max, 1.0);
   }

   public void setZAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(Z_AXIS, name, holder, min, max, exponent);
   }

   public void setZAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(Z_AXIS, name, holder, min, max, 1.0);
   }

   public void setZAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(Z_AXIS, name, variable, min, max, exponent);
   }

   public void setZAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(Z_AXIS, name, variable, min, max, 1.0);
   }

   public void setRAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(R_AXIS, name, holder, min, max, exponent);
   }

   public void setRAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(R_AXIS, name, holder, min, max, 1.0);
   }

   public void setRAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(R_AXIS, name, variable, min, max, exponent);
   }

   public void setRAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(R_AXIS, name, variable, min, max, 1.0);
   }

   public void setUAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(U_AXIS, name, holder, min, max, exponent);
   }

   public void setUAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(U_AXIS, name, holder, min, max, 1.0);
   }

   public void setUAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(U_AXIS, name, variable, min, max, exponent);
   }

   public void setUAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(U_AXIS, name, variable, min, max, 1.0);
   }

   public void setVAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(V_AXIS, name, holder, min, max, exponent);
   }

   public void setVAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(V_AXIS, name, holder, min, max, 1.0);
   }

   public void setVAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(V_AXIS, name, variable, min, max, exponent);
   }

   public void setVAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(V_AXIS, name, variable, min, max, 1.0);
   }

   public void setPOVAxis(String name, YoVariableHolder holder, double min, double max, double exponent)
   {
      setAxis(POV_AXIS, name, holder, min, max, exponent);
   }

   public void setPOVAxis(String name, YoVariableHolder holder, double min, double max)
   {
      setAxis(POV_AXIS, name, holder, min, max, 1.0);
   }

   public void setPOVAxis(String name, DoubleYoVariable variable, double min, double max, double exponent)
   {
      setAxis(POV_AXIS, name, variable, min, max, exponent);
   }

   public void setPOVAxis(String name, DoubleYoVariable variable, double min, double max)
   {
      setAxis(POV_AXIS, name, variable, min, max, 1.0);
   }

   protected void notifyVariableChangedListeners(YoVariable variable)
   {
//    System.out.println("SaitekX52Joystick: variableChanged");
      for (VariableChangedListener variableChangedListener : variableChangedListeners)
      {
         variableChangedListener.variableChanged(variable);
      }
   }
}
