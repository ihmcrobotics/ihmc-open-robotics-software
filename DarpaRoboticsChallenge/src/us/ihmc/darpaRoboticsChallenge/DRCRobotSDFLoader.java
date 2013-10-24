package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotSDFLoader
{
   
   public static JaxbSDFLoader loadDRCRobot(DRCRobotJointMap jointMap)
   {
      return loadDRCRobot(jointMap ,false);
   }
   
   public static JaxbSDFLoader loadDRCRobot(DRCRobotJointMap jointMap, boolean headless)
   {
      InputStream fileInputStream;
      ArrayList<String> resourceDirectories = new ArrayList<String>();
      Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
      DRCRobotModel selectedModel = jointMap.getSelectedModel();
      
      if(!headless)
      {
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo").getFile());
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/atlas_description").getFile());
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/multisense_sl_description").getFile());
         resourceDirectories.add(myClass.getResource("models").getFile());
      }
      
      switch (selectedModel)
      {
         case ATLAS_NO_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas.sdf");
            
            break;

         case ATLAS_IROBOT_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands.sdf");
            if(!headless)
            {
               resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/irobot_hand_description").getFile());
            }
            break;

         case ATLAS_SANDIA_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_sandia_hands.sdf");
            break;
         case ATLAS_NO_HANDS_ADDED_MASS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_addedmass.sdf");
            break;
         case V1 :
            fileInputStream = myClass.getResourceAsStream("models/V1/urdf/V1_gazebo.sdf");
            break;
         default:
            throw new RuntimeException("DRCRobotSDFLoader: Unimplemented enumeration case : " + selectedModel);
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
   
   public static void main(String[] args)
   {
//      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_SANDIA_HANDS, false);
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.V1, false);
      JaxbSDFLoader loader = loadDRCRobot(jointMap, false);
      System.out.println(loader.createRobot(jointMap, true).getName());
      
   }
   
}
