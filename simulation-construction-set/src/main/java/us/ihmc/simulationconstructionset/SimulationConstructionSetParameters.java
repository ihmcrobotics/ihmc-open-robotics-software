package us.ihmc.simulationconstructionset;

import us.ihmc.commons.PrintTools;

import java.util.HashMap;

public class SimulationConstructionSetParameters
{
   public static final String SHOW_SPLASH_SCREEN = "show.splash.screen";
   public static final String CREATE_SCS_GUI = "create.scs.gui";
   public static final String SHOW_SCS_WINDOWS = "show.scs.windows";
   public static final String SCS_DATA_BUFFER_SIZE = "scs.dataBuffer.size";
   public static final String SHOW_SCS_YOGRAPHICS = "show.scs.yographics";
   public static final String SCS_YOGRAPHICS_GLOBALSCALE = "scs.yographics.globalscale";

   protected HashMap<String, TypeHolder> parameters = new HashMap<>();

   public SimulationConstructionSetParameters()
   {
      parameters.put(SHOW_SPLASH_SCREEN, new BooleanHolder("true"));
      parameters.put(CREATE_SCS_GUI, new BooleanHolder("true"));
      parameters.put(SHOW_SCS_WINDOWS, new BooleanHolder("true"));
      parameters.put(SCS_DATA_BUFFER_SIZE, new IntHolder("8192"));
      parameters.put(SHOW_SCS_YOGRAPHICS, new BooleanHolder("true"));
      parameters.put(SCS_YOGRAPHICS_GLOBALSCALE, new DoubleHolder("1.0"));
   }

   public SimulationConstructionSetParameters(boolean createGUI, int bufferSize)
   {
      this();

      setCreateGUI(createGUI);
      setDataBufferSize(bufferSize);
   }

   public SimulationConstructionSetParameters(int dataBufferSize)
   {
      this();

      setDataBufferSize(dataBufferSize);
   }

   public SimulationConstructionSetParameters(boolean createGUI)
   {
      this();

      setCreateGUI(createGUI);
   }

   public static SimulationConstructionSetParameters createFromSystemProperties()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setFromSystemProperties();
      return parameters;
   }

   /**
    * @deprecated Use {@link #createFromSystemProperties()} instead.
    */
   public static SimulationConstructionSetParameters createFromEnvironmentVariables()
   {
      return createFromSystemProperties();
   }

   public final void setFromSystemProperties()
   {
      for (String systemPropertyName : parameters.keySet())
      {
         String propertyValue = System.getProperty(systemPropertyName);
         if (propertyValue != null)
         {
            parameters.get(systemPropertyName).setFromString(propertyValue);
            PrintTools.info(this, "Loading " + systemPropertyName + ": " + propertyValue);
         }
         else
         {
            PrintTools.warn(this, "System property not set: " + systemPropertyName + ". Current value: " + parameters.get(systemPropertyName).getStringValue());
         }
      }
   }

   public int getDataBufferSize()
   {
      return ((IntHolder) parameters.get(SCS_DATA_BUFFER_SIZE)).value;
   }

   public boolean getCreateGUI()
   {
      return ((BooleanHolder) parameters.get(CREATE_SCS_GUI)).value;
   }

   public void setCreateGUI(boolean value)
   {
      ((BooleanHolder) parameters.get(CREATE_SCS_GUI)).value = value;
   }

   public void setDataBufferSize(int value)
   {
      ((IntHolder) parameters.get(SCS_DATA_BUFFER_SIZE)).value = value;
   }

   public void setShowSplashScreen(boolean value)
   {
      ((BooleanHolder) parameters.get(SHOW_SPLASH_SCREEN)).value = value;
   }

   public boolean getShowWindows()
   {
      return ((BooleanHolder) parameters.get(SHOW_SCS_WINDOWS)).value;
   }

   public void setShowWindows(boolean value)
   {
      ((BooleanHolder) parameters.get(SHOW_SCS_WINDOWS)).value = value;
   }

   public boolean getShowSplashScreen()
   {
      return ((BooleanHolder) parameters.get(SHOW_SPLASH_SCREEN)).value;
   }

   public boolean getShowYoGraphicObjects()
   {
      return ((BooleanHolder) parameters.get(SHOW_SCS_YOGRAPHICS)).value;
   }

   public void setShowYoGraphicObjects(boolean value)
   {
      ((BooleanHolder) parameters.get(SHOW_SCS_YOGRAPHICS)).value = value;
   }

   public double getYoGraphicsGlobalScale()
   {
      return ((DoubleHolder) parameters.get(SCS_YOGRAPHICS_GLOBALSCALE)).value;
   }

   public void setYoGraphicsGlobalScale(double value)
   {
      ((DoubleHolder) parameters.get(SCS_YOGRAPHICS_GLOBALSCALE)).value = value;
   }

   protected class DoubleHolder extends TypeHolder
   {
      public double value;

      public DoubleHolder(String initialValue)
      {
         super(initialValue);
      }

      public void setFromString(String stringValue)
      {
         value = Double.parseDouble(stringValue);
      }

      public String getStringValue()
      {
         return String.valueOf(value);
      }
   }

   protected class IntHolder extends TypeHolder
   {
      public int value;

      public IntHolder(String initialValue)
      {
         super(initialValue);
      }

      public void setFromString(String stringValue)
      {
         value = Integer.parseInt(stringValue);
      }

      public String getStringValue()
      {
         return String.valueOf(value);
      }
   }

   protected class BooleanHolder extends TypeHolder
   {
      public boolean value;

      public BooleanHolder(String initialValue)
      {
         super(initialValue);
      }

      public void setFromString(String stringValue)
      {
         value = Boolean.parseBoolean(stringValue);
      }

      public String getStringValue()
      {
         return String.valueOf(value);
      }
   }

   protected abstract class TypeHolder
   {
      public TypeHolder(String initialValue)
      {
         setFromString(initialValue);
      }

      public abstract void setFromString(String stringValue);

      public abstract String getStringValue();
   }
}
