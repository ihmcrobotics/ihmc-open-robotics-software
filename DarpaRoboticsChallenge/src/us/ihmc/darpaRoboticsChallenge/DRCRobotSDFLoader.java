package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class DRCRobotSDFLoader
{
   public static JaxbSDFLoader loadDRCRobot(DRCRobotModel robotModel)
   {
      return loadDRCRobot(robotModel, false);
   }
   
   public static JaxbSDFLoader loadDRCRobot(DRCRobotModel robotModel, boolean headless)
   {
      return loadDRCRobot(robotModel.getResourceDirectories(), robotModel.getSdfFileAsStream(), headless);
   }
   
   public static JaxbSDFLoader loadDRCRobot(String[] resourceDirectories, InputStream sdfFile, boolean headless)
   {
      ArrayList<String> resources = new ArrayList<String>();
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;

      if (!headless)
      {
         resources.add(myClass.getResource("models").getFile());
         resources.add(myClass.getResource("models/GFE/").getFile());
         resources.add(myClass.getResource("models/GFE/gazebo").getFile());
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
