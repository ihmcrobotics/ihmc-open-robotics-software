package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.log.LogTools;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

public class DRCRobotSDFLoader
{
   public static JaxbSDFLoader loadDRCRobot(String[] resourceDirectories, InputStream sdfFile, SDFDescriptionMutator descriptionMutator)
   {
      ArrayList<String> resources = new ArrayList<String>();

      for (String resource : resourceDirectories)
      {
         resources.add(resource);
      }

      if (sdfFile == null)
      {
         LogTools.error("Failed to load sdf file: null");
      }

      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(sdfFile, resources, descriptionMutator);
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException("Cannot find SDF file: " + e.getMessage());
      }
      catch (JAXBException e)
      {
         e.printStackTrace();

         throw new RuntimeException("Invalid SDF file: " + e.getMessage());
      }

      return jaxbSDFLoader;
   }
}
