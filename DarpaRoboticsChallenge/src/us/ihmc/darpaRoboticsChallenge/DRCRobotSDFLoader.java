package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotSDFLoader
{
   private static final String fileName = "Models/GFE/atlas.sdf";
   private static final String modelName = "atlas";
   private static final String resourceDirectory = "Models/GFE/models/";


   public static JaxbSDFLoader loadDRCRobot()
   {
      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(fileName, modelName, resourceDirectory, new DRCRobotJointMap());
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
