package us.ihmc.alexander.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Interface defining common helper methods for parameter initialization
 * in SimulationParameters and RealRobotParameters.
 */
public interface ParameterInitializationHelper
{
   // ------ Registry finders -----
   /**
    * Finds a registry with the given name starting from the root.
    *
    * @param name the registry name
    * @return YoRegistry instance
    * @throws IllegalArgumentException if not found
    */
   YoRegistry findReg(String name);

   /**
    * Finds a registry with the given name starting from the parent registry
    * @param parent the parent registry
    * @param name the sub-registry name
    * @throws IllegalArgumentException if not found
    */
   YoRegistry findReg(YoRegistry parent, String name);
   

   /**
    * Finds a registry with the given names with aliases to throw if none from the parent registry
    * @param parent the parent registry
    * @param names the sub-registry names (aliases)
    * @throws IllegalArgumentException if not found
    */
   YoRegistry findReg(YoRegistry parent, String... names);

   /**
    * Finds a sub-registry from the given parent registry.
    *
    * @param parent parent registry
    * @param names   sub-registry name
    * @return sub-registry or null if not found
    */
   YoRegistry findRegIfPresent(YoRegistry parent, String... names);
   
   
   // -------- Variable setters: single-name exact ---------
   /**
    * 
    * @param reg registry name
    * @param varName variable name in the parent registry
    * @param value boolean values to be set
    * @return
    */
   YoBoolean setBoolean(YoRegistry reg, String varName, boolean value);

   /**
    * 
    * @param reg registry name
    * @param varName variable name in the parent registry
    * @param value double values to be set in the parameter
    * @return
    */
   YoDouble setDouble(YoRegistry reg, String varName, double value);

   /**
    * Sets an integer variable in the given registry.
    * @param reg registry name
    * @param varName variable name in the registry
    * @param value integer value to be set
    * @return
    */
   YoInteger setInt(YoRegistry reg, String varName, int value);
   
   
   // --------- Variable setters: alias-name ---------
   /**
    * Finds and sets a YoDouble variable from the given registry by multiple alias names.
    *
    * @param reg      registry containing the variable
    * @param varNames possible variable names (aliases)
    * @param value    default value to set
    * @return found YoDouble instance
    * @throws IllegalArgumentException if none found
    */
   YoDouble setAnyDouble(YoRegistry reg, String[] varNames, double value);

   /**
    * Finds and sets a YoBoolean variable from the given registry by multiple alias names.
    *
    * @param reg      registry containing the variable
    * @param varNames possible variable names (aliases)
    * @param value    default value to set
    * @return found YoBoolean instance
    * @throws IllegalArgumentException if none found
    */
   YoBoolean setAnyBoolean(YoRegistry reg, String[] varNames, boolean value);

   /**
    * Finds and sets a YoInteger variable from the given registry by multiple alias names.
    *
    * @param reg      registry containing the variable
    * @param varNames possible variable names (aliases)
    * @param value    default value to set
    * @return found YoInteger instance
    * @throws IllegalArgumentException if none found
    */
   YoInteger setAnyInt(YoRegistry reg, String[] varNames, int value);
}
