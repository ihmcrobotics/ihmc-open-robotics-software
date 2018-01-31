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
            reader = new XmlParameterReader(parameterFile);
            if (overwriteFile != null)
            {
               reader.overwrite(overwriteFile);
            }
            reader.readParametersInRegistry(registry);
         }
         catch (IOException e)
         {
            throw new RuntimeException("Cannot read parameters.", e);
         }
      }
   }
}
