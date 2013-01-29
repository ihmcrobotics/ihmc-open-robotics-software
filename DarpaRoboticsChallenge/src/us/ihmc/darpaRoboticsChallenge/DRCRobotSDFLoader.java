package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotSDFLoader
{
   public static RobotModel selectedModel = RobotModel.ATLAS_NO_HANDS;

   private static String fileName = "Models/GFE/models/drc_robot/gfe.sdf";
   private static String modelName = "drc_robot";
   private static String resourceDirectory = "Models/GFE/models/";

   public static enum RobotModel {ATLAS_NO_HANDS, ATLAS_IROBOT_HANDS, ATLAS_SANDIA_HANDS;}

   public static JaxbSDFLoader loadDRCRobot()
   {
      switch (selectedModel)
      {
         case ATLAS_NO_HANDS :
            fileName = "Models/GFE/models/drc_robot/gfe.sdf";
            modelName = "drc_robot";
            resourceDirectory = "Models/GFE/models/";
            break;

         case ATLAS_IROBOT_HANDS :
            fileName = "Models/GFE/atlas_irobot_hands.sdf";
            modelName = "atlas";
            resourceDirectory = "Models/GFE/models/";
            break;

         case ATLAS_SANDIA_HANDS :
            fileName = "Models/GFE/atlas_sandia_hands.sdf";
            modelName = "atlas";
            resourceDirectory = "Models/GFE/models/";
            
            //modelName = "atlas";
            //fileName = "Models/GFE/models/atlas_sandia_hand/atlas.sdf";
            //resourceDirectory = "Models/GFE/models/";

            break;
      }

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
