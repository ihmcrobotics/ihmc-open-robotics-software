package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;

public class DRCRobotSDFLoader
{
     
   public static JaxbSDFLoader loadDRCRobot(String[] resourceDirectories, InputStream sdfFile, boolean headless)
   {
      ArrayList<String> resources = new ArrayList<String>();
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;

      if (!headless)
      {
         resources.add("models");
         resources.add("models/GFE/");
         resources.add("models/gazebo/");
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
         jaxbSDFLoader = new JaxbSDFLoader(sdfFile, resources);
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
