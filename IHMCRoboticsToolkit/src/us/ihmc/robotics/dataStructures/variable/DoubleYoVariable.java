package us.ihmc.robotics.dataStructures.variable;

import java.text.FieldPosition;
import java.text.NumberFormat;

import org.apache.commons.math3.util.Precision;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 *
 * <p>YoVariables provide a simple, convenient mechanism for storing and manipulating robot data.  While each
 * essentially contains a double value YoVariables are designed for integration into the SCS GUI.  Once registered,
 * a variable will automatically become available to the GUI for graphing, modification and other data manipulation.
 * Historical values of all registered YoVariables are stored in the DataBuffer which may be exported for later use.</p>
 * <p>
 */
public class DoubleYoVariable extends YoVariable<DoubleYoVariable>
{
   private static final java.text.NumberFormat DOUBLE_FORMAT = new java.text.DecimalFormat(" 0.00000;-0.00000");
   private static final FieldPosition FIELD_POSITION = new FieldPosition(NumberFormat.INTEGER_FIELD);

   private double val;

   /**
    * Creates a new YoVariable with the given name and adds it to the specified registry.
    *
    * @param name name to be used for all references of this variable by SCS
    * @param registry YoVariableRegistry with which this variable is to be registerd
    * @see YoVariableRegistry YoVariableRegistry
    */
   public DoubleYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, "", registry);
   }

   /**
    * Creates a new YoVariable with the given name and adds it to the specified registry.  This
    * constructor allows the user to provide a simple description of the variable, which can
    * come in handy when faced with thousands.  The description is displayed in the VarPanel
    * component of the GUI.  This variant also allows the user to specify min and max values for
    * manual scaling of graphs.
    *
    * @param name name to be used for all references of this variable by SCS
    * @param description A short description of this variable
    * @param registry YoVariableRegistry with which this variable is to be registered
    * @param minScaling minimum value for scaling purposes
    * @param maxScaling maximum value for scaling purpouses
    * @see YoVariableRegistry YoVariableRegistry
    */
   public DoubleYoVariable(String name, String description, YoVariableRegistry registry, double minScaling, double maxScaling)
   {
      this(name, description, registry);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   /**
    * Creates a new YoVariable with the given name and adds it to the specified registry.  This
    * constructor allows the user to provide a simple description of the variable, which can
    * come in handy when faced with thousands.  The description is displayed in the VarPanel
    * component of the GUI.
    *
    * @param name name to be used for all references of this variable by SCS
    * @param description A short description of this variable
    * @param registry YoVariableRegistry with which this variable is to be registered
    * @see YoVariableRegistry YoVariableRegistry
    */
   public DoubleYoVariable(String name, String description, YoVariableRegistry registry)
   {
      super(YoVariableType.DOUBLE, name, description, registry);

      this.set(0.0);
   }

   /**
    * Retrieves a string representation of this variable.
    *
    * @return String representation
    */
   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();
      retBuffer.append(getName());
      retBuffer.append(": ");
      retBuffer.append(getDoubleValue());
      return retBuffer.toString();
   }

   public boolean isNaN()
   {
      return Double.isNaN(val);
   }

   public void add(DoubleYoVariable variable)
   {
      this.set(this.getDoubleValue() + variable.getDoubleValue());
   }

   public void sub(DoubleYoVariable variable)
   {
      this.set(this.getDoubleValue() - variable.getDoubleValue());
   }

   public void sub(double value)
   {
      this.set(this.getDoubleValue() - value);
   }

   public void add(double value)
   {
      this.set(this.getDoubleValue() + value);
   }

   public void mul(double value)
   {
      this.set(this.getDoubleValue() * value);
   }

   public void mul(DoubleYoVariable value)
   {
      this.set(this.getDoubleValue() * value.getDoubleValue());
   }

   /**
    * Check if the value contained by this variable is equal to the given double.  If not of double type
    * a warning will be printed to the console.
    *
    * @param value double to be compared
    * @return boolean are they equal?
    */
   public boolean valueEquals(double value)
   {
      return (val == value);
   }

   /**
    * Retrieve the double value of this variable, if not of double type a warning will be printed.
    *
    * @return double value of this
    */
   public double getDoubleValue()
   {
      return val;
   }

   /**
    * Set the value of this YoVariable.  All four types are represented via the same interal double.  If
    * of integer type this value is cast as an int whenever accessed.  It represents the ordinal if of enum type.
    * Boolean values are triggered around the 0.5 threshold with < 0.5 being false and greater or equal being true.
    *
    * @param value double value to store
    */
   public boolean set(double value, boolean notifyListeners)
   {
      if (val != value)
      {
         val = value;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }
      return false;
   }

   public void set(double value)
   {
      set(value, true);
   }

   //   NOTE: JEP October 30, 2010:
   //   The following is very useful for debugging things so please do not delete!
   //   I should probably use the change listener stuff instead, but this is nice for eavesdropping
   //   to catch when a variable changes or in order to compare two runs that should be identical
   //   to discover the first time their YoVariables differ...
   //
   //   private static boolean startDisplaying = false;
   //   private static boolean stopDisplaying = false;
   //   private static DoubleYoVariable time;
   //   private static PrintWriter writer;
   //
   //   private void setAndLogToAFile(double value)
   //   {
   //      if ((time == null) && (this.getName().equals("t")))
   //      {
   //         System.out.println("found time");
   //         time = this;
   //
   //
   //         try
   //         {
   //            writer = new PrintWriter("run.txt");
   //         } catch (FileNotFoundException e)
   //         {
   //
   //         }
   //      }
   //
   //      if ((time != null) && (time.getDoubleValue() >= 1.656-1e-7)) startDisplaying = true;
   //      if ((time != null) && (time.getDoubleValue() >= 1.6632+1e-7))
   //      {
   //         stopDisplaying = true; //1.6705
   //         writer.close();
   //      }
   //
   //      if (startDisplaying & !stopDisplaying)
   //      {
   //         if ((Math.abs(time.getDoubleValue() - 1.6632) < 0.00005) && (this.getName().equals("o_tau_rh_roll")))
   //         {
   //            System.out.println("time = " + time.getDoubleValue());
   //            System.out.println(this.getName() + " is getting set to " + value);
   //         }
   //
   //         if (!this.name.contains("DurationMilli"))
   //         {
   //            writer.println(time.getDoubleValue() + ": " + this.name + " = " + value);
   //            //       System.out.println(time.getDoubleValue() + ": " + this.name + " = " + value);
   //         }
   //      }
   //
   //   }

   /**
    * Appends the value of this variable to the end of the given StringBuffer.  This representation is
    * based on variable type.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override
   public void getValueString(StringBuffer stringBuffer)
   {
      getValueStringFromDouble(stringBuffer, val);
   }

   @Override
   public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      DOUBLE_FORMAT.format(doubleValue, stringBuffer, FIELD_POSITION); // Add the variable value to it
   }

   @Override
   public double getValueAsDouble()
   {
      return getDoubleValue();
   }

   @Override
   public void setValueFromDouble(double value, boolean notifyListeners)
   {
      set(value, notifyListeners);
   }

   @Override
   public long getValueAsLongBits()
   {
      return Double.doubleToLongBits(val);
   }

   @Override
   public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set(Double.longBitsToDouble(value), notifyListeners);
   }

   @Override
   public DoubleYoVariable duplicate(YoVariableRegistry newRegistry)
   {
      DoubleYoVariable retVar = new DoubleYoVariable(getName(), getDescription(), newRegistry, getManualScalingMin(), getManualScalingMax());
      retVar.set(val);
      return retVar;
   }

   @Override
   public boolean setValue(DoubleYoVariable value, boolean notifyListeners)
   {
      return set(value.getDoubleValue(), notifyListeners);
   }

   public void setToNaN()
   {
      this.set(Double.NaN);

   }

   @Override
   public boolean isZero()
   {
      return Precision.equals(0.0, getDoubleValue(), 1);
   }
}
