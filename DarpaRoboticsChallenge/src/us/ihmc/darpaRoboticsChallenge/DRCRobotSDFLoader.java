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
//         case ATLAS_NO_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas.sdf");
            break;
//
//         case ATLAS_IROBOT_HANDS :
//        	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands.sdf");
//        	 break;
//         case ATLAS_IROBOT_HANDS_ADDED_MASS :
//    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass.sdf");
//        	 break;
//         case ATLAS_IROBOT_HANDS_WITH_EXTENSION :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_hands_extension.sdf");
//            break;
//         case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_hands_extension_rotated_added_mass.sdf");
//            break;
//         case ATLAS_IROBOT_LEFT_HAND_WITH_EXTENSION_ROTATED_RIGHT_HAND_HOOK_ADDED_MASS :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_left_hand_extension_rotated_added_mass.sdf");
//            break;
//         case ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS:
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_v3_irobot_hands_right_8_inch_extension_left_4_inch_rotated_added_mass.sdf");
//         case DEBRIS_TASK_MODEL:
//            fileInputStream = myClass.getResourceAsStream("models/GFE/debris_model.sdf");
//            break;
//         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
//    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass_comxz.sdf");
//        	 break;
//         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
//    	 	 fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_addedmass_comxyz.sdf");
//        	 break; 
//
//         case ATLAS_IHMC_PARAMETERS :
//            throw new RuntimeException("Fixme: redo atlas_ihmc_parameters.sdf based on new models");
////            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_ihmc_parameters.sdf");
//
         case ATLAS_SANDIA_HANDS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_sandia_hands.sdf");
            break;
//
         case ATLAS_NO_HANDS_ADDED_MASS :
            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_addedmass.sdf");

            break;
//
//         case ATLAS_CALIBRATION :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_calibration.sdf");
//            break;
//            
//         case ATLAS_RHOOK_HAND :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_irobot_hands_and_rhook_addedmass.sdf");
//
//            break;
//         case ATLAS_HOOK_HANDS :
//            fileInputStream = myClass.getResourceAsStream("models/GFE/atlas_hook_hands_addedmass.sdf");
            
//            break;
            
         case DRC_NO_HANDS:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_no_hands.sdf");
            break;
         case DRC_HANDS:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_hands.sdf");
            break;
         case DRC_EXTENDED_HANDS:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_extended_hands.sdf");
            break;

         case DRC_HOOKS:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_hooks.sdf");
            break;
         case DRC_TASK_HOSE:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_task_hose.sdf");
            break;
         case DRC_EXTENDED_HOOKS:
            fileInputStream = myClass.getResourceAsStream("models/GFE/drc_extended_hooks.sdf");
            break;
         case VALKYRIE:
            fileInputStream = myClass.getResourceAsStream("models/V1/sdf/V1_sim.sdf");
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
      DRCRobotJointMap jointMap = DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS.getJointMap(false, false);
      JaxbSDFLoader loader = loadDRCRobot(jointMap, false);
      System.out.println(loader.createRobot(jointMap, true).getName());
      
      SimulationConstructionSet scs = new SimulationConstructionSet(loader.createRobot(jointMap, false));
      scs.startOnAThread();

   }

}
