package us.ihmc.wholeBodyController.parameters;

import java.io.IOException;
import java.io.InputStream;

import us.ihmc.commons.PrintTools;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterLoaderHelper
{
   private static final boolean debugLoading = false;

   public static void loadParameters(Object caller, WholeBodyControllerParameters controllerParameters, YoVariableRegistry registry)
   {
      InputStream parameterFile = controllerParameters.getWholeBodyControllerParametersFile();
      InputStream overwriteFile = controllerParameters.getParameterOverwrites();
      loadParameters(caller, parameterFile, overwriteFile, registry);
   }

   public static void loadParameters(Object caller, InputStream parameterFile, YoVariableRegistry registry)
   {
      loadParameters(caller, parameterFile, null, registry);
   }

   public static void loadParameters(Object caller, InputStream parameterFile, InputStream overwriteFile, YoVariableRegistry registry)
   {
      if (parameterFile == null)
      {
         PrintTools.error(caller, "No parameter file provided. Falling back to loading the default values for parameters.");
         DefaultParameterReader reader = new DefaultParameterReader();
         reader.readParametersInRegistry(registry);
      }
      else
      {
         XmlParameterReader reader;
         try
         {
            reader = new XmlParameterReader(debugLoading, parameterFile);
            if (overwriteFile != null)
            {
               reader.overwrite(overwriteFile);
            }
            loadAndCheckStatistics(registry, reader);
         }
         catch (IOException e)
         {
            throw new RuntimeException("Cannot read parameters.", e);
         }
      }
   }

   /**
    * This will check some statistics about the loading of the parameters and print
    * information if not everything went as expected. If that is the case consider
    * turning on the debug flag {@link ParameterLoaderHelper#debugLoading} to see
    * what parameters were not specified in the XML file.
    */
   private static void loadAndCheckStatistics(YoVariableRegistry registry, XmlParameterReader reader)
   {
      int defaults = reader.readParametersInRegistry(registry);
      int parametersInXML = reader.getNumberOfParameters();
      int parametersInRegistry = registry.getAllParameters().size();
      int loadedParameters = parametersInRegistry - defaults;
      int parametersInXMLWithoutMatch = parametersInXML - loadedParameters;
      if (defaults != 0 || parametersInXMLWithoutMatch != 0)
      {
         PrintTools.error("There might be something wrong with your parameter file:");
         System.err.println("   Number of parameters in registry: " + parametersInRegistry);
         System.err.println("   Number of parameters using their default value: " + defaults);
         System.err.println("   Number of parameters in XML with no match: " + parametersInXMLWithoutMatch);
      }
   }
}
