package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.net.URL;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotSDFLoader
{
   private final DRCRobotModel selectedModel;

   
   public DRCRobotSDFLoader(DRCRobotModel selectedModel)
   {
      this.selectedModel = selectedModel;
   }


   public JaxbSDFLoader loadDRCRobot()
   {
      URL fileURL;
      String modelName;
      URL resourceDirectory;
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
      switch (selectedModel)
      {
         case ATLAS_NO_HANDS :
            fileURL = myClass.getResource("models/GFE/models/drc_robot/gfe.sdf");
            modelName = "drc_robot";
            
            resourceDirectory = myClass.getResource("models/GFE/models/");
            break;

         case ATLAS_IROBOT_HANDS :
            fileURL = myClass.getResource("models/GFE/atlas_irobot_hands.sdf");
            modelName = "atlas";
            resourceDirectory = myClass.getResource("models/GFE/models/");
            break;

         case ATLAS_SANDIA_HANDS :
            fileURL = myClass.getResource("models/GFE/atlas_sandia_hands.sdf");
            modelName = "atlas";
            resourceDirectory = myClass.getResource("models/GFE/models/");

            break;
         default:
            throw new RuntimeException("DRCRobotSDFLoader: Unimplemented enumeration case : " +selectedModel);
      }

      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(fileURL, modelName, resourceDirectory, new DRCRobotJointMap(selectedModel));
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
