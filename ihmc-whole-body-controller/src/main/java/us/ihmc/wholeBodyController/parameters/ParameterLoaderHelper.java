package us.ihmc.wholeBodyController.parameters;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashSet;

import us.ihmc.log.LogTools;
import us.ihmc.robotics.Skully;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.parameters.AbstractParameterReader;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.XmlParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterLoaderHelper
{
   private static final boolean debugLoading = false;

   public static void loadParameters(Object caller, WholeBodyControllerParameters<?> controllerParameters, YoVariableRegistry registry)
   {
      InputStream parameterFile = controllerParameters.getWholeBodyControllerParametersFile();
      InputStream overwriteFile = controllerParameters.getParameterOverwrites();
      try 
      {
         loadParameters(caller, parameterFile, overwriteFile, registry, true);
      }
      catch(RuntimeException exception)
      {
         LogTools.error(exception.getMessage() + "\n file: " + controllerParameters.getParameterFileName());
         throw exception;
      }
   }

   public static void loadParameters(Object caller, InputStream parameterFile, YoVariableRegistry registry)
   {
      loadParameters(caller, parameterFile, null, registry, true);
   }

   public static void loadParameters(Object caller, InputStream parameterFile, YoVariableRegistry registry, boolean printWarnings)
   {
      loadParameters(caller, parameterFile, null, registry, printWarnings);
   }

   public static void loadParameters(Object caller, InputStream parameterFile, InputStream overwriteFile, YoVariableRegistry registry, boolean printWarnings)
   {
      if (parameterFile == null)
      {
         LogTools.error("No parameter file provided. Falling back to loading the default values for parameters.");
         DefaultParameterReader reader = new DefaultParameterReader();
         reader.readParametersInRegistry(registry);
      }
      else
      {
         XmlParameterReader reader;
         try
         {
            reader = new XmlParameterReader(debugLoading, registry.getName(), parameterFile);
            if (overwriteFile != null)
            {
               reader.overwrite(overwriteFile);
            }
            loadAndCheckStatistics(registry, reader, printWarnings);
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
   private static void loadAndCheckStatistics(YoVariableRegistry registry, AbstractParameterReader reader, boolean printWarnings)
   {
      HashSet<String> defaultParameters = new HashSet<>();
      HashSet<String> unmatchedParameters = new HashSet<>();
      reader.readParametersInRegistry(registry, defaultParameters, unmatchedParameters);

      if (printWarnings && !unmatchedParameters.isEmpty() && !debugLoading)
      {
         String message = "I think something is off in your parameter file.";
         String additionalInfo = "Parameters in registry: " + registry.getAllParameters().size() + "\n" +
               "Parameters using their default value: " + defaultParameters.size() + "\n" +
               "Parameters in XML with no match: " + unmatchedParameters.size();
         Skully.say(message, additionalInfo);
      }

      if (debugLoading)
      {
         LogTools.info("When loading " + registry.getName() + " with " + registry.getAllParameters().size() + " parameters:");
         LogTools.info("\n---> Unmatched in XML <---");
         unmatchedParameters.forEach(parameter -> LogTools.info(parameter));
         LogTools.info("\n---> Default Values: <---");
         defaultParameters.forEach(parameter -> LogTools.info(parameter));
      }
   }
}