package us.ihmc.darpaRoboticsChallenge;

import java.io.File;
import java.io.FileNotFoundException;
import java.net.URL;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotSDFLoader
{
   public JaxbSDFLoader loadDRCRobot(DRCRobotJointMap jointMap)
   {
      URL fileURL;
      String modelName;
      ArrayList<String> resourceDirectories = new ArrayList<String>();
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
      DRCRobotModel selectedModel = jointMap.getSelectedModel();
      
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/atlas_description").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/multisense_sl_description").getFile());
      
      switch (selectedModel)
      {
         case ATLAS_NO_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
            fileURL = myClass.getResource("models/GFE/atlas.sdf");
            modelName = "atlas";
            
            break;

         case ATLAS_IROBOT_HANDS :
            fileURL = myClass.getResource("models/GFE/atlas_irobot_hands.sdf");
            resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/irobot_hand_description").getFile());
            modelName = "atlas";
            break;

         case ATLAS_SANDIA_HANDS :
            fileURL = myClass.getResource("models/GFE/atlas_sandia_hands.sdf");
            modelName = "atlas";

            break;

         default:
            throw new RuntimeException("DRCRobotSDFLoader: Unimplemented enumeration case : " + selectedModel);
      }

      JaxbSDFLoader jaxbSDFLoader;
      try
      {
         jaxbSDFLoader = new JaxbSDFLoader(new File(fileURL.getFile()), modelName, resourceDirectories, jointMap);
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
