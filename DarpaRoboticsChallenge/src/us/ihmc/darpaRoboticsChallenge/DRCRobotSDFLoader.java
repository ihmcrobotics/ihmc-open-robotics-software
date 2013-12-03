package us.ihmc.darpaRoboticsChallenge;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;

import javax.xml.bind.JAXBException;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

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
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo").getFile());
         resourceDirectories.add(myClass.getResource("models/GFE/").getFile());
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
        	 break;
         case ATLAS_IROBOT_HANDS_ADDED_MASS :
    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass.sdf");
        	 break;
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_hands_extension.sdf");
            break;
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_hands_extension_rotated_added_mass.sdf");
            break;
         case ATLAS_IROBOT_LEFT_HAND_WITH_EXTENSION_ROTATED_RIGHT_HAND_HOOK_ADDED_MASS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_left_hand_extension_rotated_added_mass.sdf");
            break;
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass_comxz.sdf");
        	 break;
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass_comxyz.sdf");
        	 break;

         case ATLAS_IHMC_PARAMETERS :
            throw new RuntimeException("Fixme: redo atlas_ihmc_parameters.sdf based on new models");
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_ihmc_parameters.sdf");

         case ATLAS_SANDIA_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_sandia_hands.sdf");

            break;

         case ATLAS_NO_HANDS_ADDED_MASS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_addedmass.sdf");

            break;

         case ATLAS_CALIBRATION :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_calibration.sdf");
            
         case ATLAS_RHOOK_HAND :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_and_rhook_addedmass.sdf");

            break;

         default :
            throw new RuntimeException("DRCRobotSDFLoader: Unimplemented enumeration case : " + selectedModel);
      }
      
      if (!headless && selectedModel.hasIRobotHands())
      {
         resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/irobot_hand_description").getFile());
      }
      
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

   public static void main(String[] args)
   {
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS, false);
      JaxbSDFLoader loader = loadDRCRobot(jointMap, false);
      System.out.println(loader.createRobot(jointMap, true).getName());
      
      SimulationConstructionSet scs = new SimulationConstructionSet(loader.createRobot(jointMap, false));
      scs.startOnAThread();

   }

}
