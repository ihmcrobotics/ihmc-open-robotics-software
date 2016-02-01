package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFDescriptionMutator;

public class DRCRobotSDFLoader
{
     
   public static JaxbSDFLoader loadDRCRobot(String[] resourceDirectories, InputStream sdfFile, boolean headless, SDFDescriptionMutator descriptionMutator)
   {
      ArrayList<String> resources = new ArrayList<String>();

      if (!headless)
      {
        for(String resource : resourceDirectories)
        {
           resources.add(resource);
        }
      }
       
      if(sdfFile==null)
      {
        System.err.println("failed to load sdf file");
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
