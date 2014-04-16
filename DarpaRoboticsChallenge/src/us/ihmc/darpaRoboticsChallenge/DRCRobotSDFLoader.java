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
   public static JaxbSDFLoader loadDRCRobot(DRCRobotJointMap jointMap)
   {
      return loadDRCRobot(jointMap, false);
   }

   public static JaxbSDFLoader loadDRCRobot(DRCRobotJointMap jointMap, boolean headless)
   {
      InputStream fileInputStream;
      ArrayList<String> resourceDirectories = new ArrayList<String>();
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
      DRCRobotModel selectedModel = jointMap.getSelectedModel();

      if (!headless)
      {
         resourceDirectories.add(myClass.getResource("models").getFile());
         resourceDirectories.add(myClass.getResource("models/GFE/").getFile());
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo").getFile());
    	  for(String resource : selectedModel.getResourceDirectories())
    	  {
    		  resourceDirectories.add(resource);
    	  }
      }

      fileInputStream = selectedModel.getSdfFileAsStream();
      
      if(fileInputStream==null)
      {
    	  System.err.println("failed to load sdf file");
      }


      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(fileInputStream, resourceDirectories);
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
