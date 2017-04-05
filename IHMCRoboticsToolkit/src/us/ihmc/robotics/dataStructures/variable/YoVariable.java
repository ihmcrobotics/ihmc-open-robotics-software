package us.ihmc.robotics.dataStructures.variable;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.regex.Pattern;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

/**
 * Title:        Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 *
 * <p>YoVariables provide a simple, convienent mechanism for storing and manipulating robot data.  While each
 * essentially contains a double value YoVariables are designed for integration into the SCS GUI.  Once registered,
 * a variable will automatically become available to the GUI for graphing, modification and other data manipulation.
 * Historical values of all registered YoVariables are stored in the DataBuffer which may be exported for later use.</p>
 *
 */
public abstract class YoVariable<T extends YoVariable<T>>
{
   public static final Pattern ILLEGAL_CHARACTERS = Pattern.compile("[ .*?@#$%/^&()<>,:{}'\"\\\\]");
   private static final String SPACE_STRING = "  ";

   public static final int MAX_LENGTH_SHORT_NAME = 20;
   public static boolean warnAboutNullRegistries = true;

   protected transient String stackTraceAtInitialization;
   protected ArrayList<VariableChangedListener> variableChangedListeners;
   protected double manualMinScaling = 0.0, manualMaxScaling = 1.0, stepSize = 0.1;

   private String name;
   private String shortName;
   private String description;
   private YoVariableType type;
   private YoVariableRegistry registry;
   
   public YoVariable(YoVariableType type, String name, String description, YoVariableRegistry registry)
   {
      checkForIllegalCharacters(name);

      this.type = type;
      this.name = name;
      this.shortName = createShortName(name);
      this.description = description;
      this.registry = registry;
      this.variableChangedListeners = null;

      Throwable t = new Throwable();
      StringWriter sw = new StringWriter();
      PrintWriter pw = new PrintWriter(sw);
      t.printStackTrace(pw);
      stackTraceAtInitialization = sw.toString();
      stackTraceAtInitialization = stackTraceAtInitialization.substring(21);
      stackTraceAtInitialization = stackTraceAtInitialization.replaceAll("at(.*)\\(", "at ");
      stackTraceAtInitialization = stackTraceAtInitialization.replaceAll("\\)", "");

      registerVariable(registry, this);
   }
   
   private static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!

      if (ILLEGAL_CHARACTERS.matcher(name).find())
      {
         throw new RuntimeException(name +
               " is an invalid name for a YoVariable. A YoVariable cannot have crazy characters in them, otherwise namespaces will not work.");
      }
   }

   /**
    * Internal method used to generate the short name.  Does nothing if name is already less than or equal
    * to MAX_LENGTH_SHORT_NAME.
    *
    * @param name to be shortened
    * @return New short name
    */
   private static String createShortName(String name)
   {
      int textLength = name.length();

      if (textLength > MAX_LENGTH_SHORT_NAME)
      {
         return name.substring(0, MAX_LENGTH_SHORT_NAME / 2 - 2) + "..." + name.substring(textLength - MAX_LENGTH_SHORT_NAME / 2 + 1, textLength);
      }
      else
      {
         return name;
      }
   }

   private static void registerVariable(YoVariableRegistry registry, YoVariable<?> variable)
   {
      if (registry != null)
      {
         registry.registerVariable(variable);
      }
      else if (warnAboutNullRegistries)
      {
         System.err.println("[WARN] " + YoVariable.class.getSimpleName() + ": " + variable.getName() + " is getting created with a null registry");
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   /**
    * Retrieves the name of this YoVariable.
    *
    * @return the full name
    */
   public String getName()
   {
      return this.name;
   }

   /**
    * Retrieves a shortened version of this variables name.  The shortened name is created from the full name using the following method:
    * <OL>
    * <LI>Take the first 8 characters</LI>
    * <LI>Insert "..."</LI>
    * <LI>Add the last 9 characters</LI>
    * </OL>
    *
    * @return the name of this variable reduced to 20 characters
    */
   public String getShortName()
   {
      return this.shortName;
   }

   /**
    * Retrieve the description of this variable, "" if not specified.
    *
    * @return the description of this variable
    */
   public String getDescription()
   {
      return this.description;
   }
   
   public String getStackTraceAtInitialization()
   {
      return this.stackTraceAtInitialization;
   }

   /**
    * Adds the name of this variable to the provided string buffer.  This is done to avoid object
    * creation.
    *
    * @param buffer StringBuffer to which the name will be added at the beginning
    */
   public void getName(StringBuffer buffer)
   {
      buffer.insert(0, this.name);
   }

   /**
    * Set the min and max scaling values for graphing purposes.  By default graphs are created using manual scaling
    * based on these values where min = 0.0 and max = 1.0.
    *
    * @param minScaling double representing the min scale value
    * @param maxScaling double representing the max scale value
    */
   public void setManualScalingMinMax(double minScaling, double maxScaling)
   {
      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;
   }

   /**
    * Retrieve the current minimum value for manual scaling.
    *
    * @return double min value
    */
   public double getManualScalingMin()
   {
      return manualMinScaling;
   }

   /**
    * Retrieve the current maximum value for manual scaling.
    *
    * @return double max value
    */
   public double getManualScalingMax()
   {
      return manualMaxScaling;
   }

   /**
    * Retrieve the current maximum value for manual scaling.
    *
    * @return double step size
    */
   public double getStepSize()
   {
      return stepSize;
   }

   public void setStepSize(double stepSize)
   {
      this.stepSize = stepSize;
   }

   public final YoVariableType getYoVariableType()
   {
      return type;
   }

   /**
    * Adds the variables name & value to the beginning of the given string buffer
    *
    * @param stringBuffer StringBuffer to which the data will be added
    */
   public void getNameAndValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(name);    // buffer.insert(0,this.name); // Add the variable name to it.
      stringBuffer.append(SPACE_STRING);    // Add a space.

      getValueString(stringBuffer);
   }
   
   public void getNameAndValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      stringBuffer.append(name);    // buffer.insert(0,this.name); // Add the variable name to it.
      stringBuffer.append(SPACE_STRING);    // Add a space.

      getValueStringFromDouble(stringBuffer, doubleValue);
   }

   /**
    * fullNameEndsWith
    *
    * @param name String
    * @return boolean
    */
   public boolean fullNameEndsWithCaseInsensitive(String name)
   {
      int lastDotIndex = name.lastIndexOf(".");

      if (lastDotIndex == -1)
      {
         return this.name.toLowerCase().equals(name.toLowerCase());
      }
      
      String endOfName = name.substring(lastDotIndex + 1);
      String nameSpace = name.substring(0, lastDotIndex);

      if (!endOfName.toLowerCase().equals(this.name.toLowerCase()))
         return false;

      if (registry == null)
         return false;

      return registry.getNameSpace().endsWith(nameSpace);
   }

   /**
    * hasSameFullName
    *
    * @param variable YoVariable
    * @return boolean
    */
   public boolean hasSameFullName(YoVariable<?> variable)
   {
      return this.getFullNameWithNameSpace().equals(variable.getFullNameWithNameSpace());
   }

   public String getFullNameWithNameSpace()
   {
      if (registry == null)
         return this.name;
      if (registry.getNameSpace() == null)
         return this.name;

      return registry.getNameSpace() + "." + this.name;
   }
   
   public NameSpace getNameSpace()
   {
      return registry.getNameSpace();
   }


   public void addVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      if (variableChangedListeners == null)
      {
         variableChangedListeners = new ArrayList<>();
      }

      this.variableChangedListeners.add(variableChangedListener);
   }

   public void removeAllVariableChangedListeners()
   {
      if (this.variableChangedListeners != null)
      {
         this.variableChangedListeners.clear();
      }
   }

   public ArrayList<VariableChangedListener> getVariableChangedListeners()
   {
      return variableChangedListeners;
   }

   public void removeVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      boolean success;

      if (variableChangedListeners == null)
         success = false;
      else
         success = this.variableChangedListeners.remove(variableChangedListener);

      if (!success)
         throw new NoSuchElementException("Listener not found");
   }


   public void notifyVariableChangedListeners()
   {
      if (variableChangedListeners != null)
      {
         for(int i = 0; i <  variableChangedListeners.size(); i++)
         {
            variableChangedListeners.get(i).variableChanged(this);
         }
      }
   }

   public String getNumericValueAsAString()
   {
      return Double.toString(getValueAsDouble());
   }

   public abstract double getValueAsDouble();

   public final void setValueFromDouble(double value)
   {
      setValueFromDouble(value, true);
   }
   
   public abstract void setValueFromDouble(double value, boolean notifyListeners);

   public abstract void getValueString(StringBuffer stringBuffer);
   
   public abstract void getValueStringFromDouble(StringBuffer stringBuffer, double value);
   
   public abstract long getValueAsLongBits();
   
   public final void setValueFromLongBits(long value)
   {
      setValueFromLongBits(value, true);
   }
   
   public abstract void setValueFromLongBits(long value, boolean notifyListeners);

   public abstract T duplicate(YoVariableRegistry newRegistry);
   
   /**
    * @return true if value changed
    */
   public abstract boolean setValue(T value, boolean notifyListeners);
   
   public abstract boolean isZero();
}
