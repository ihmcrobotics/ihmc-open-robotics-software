package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.DRCRobotInterface;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;

public abstract class DRCWorld implements CommonAvatarEnvironmentInterface, DRCRobotInterface
{
   protected final ArrayList<String> resourceDirectories = new ArrayList<String>();
   protected final Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
   
   public DRCWorld()
   {

      
      // DRC resources
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/atlas_description").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/multisense_sl_description").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/irobot_hand_description").getFile());
      
      
      // Gazebo resources
      resourceDirectories.add(myClass.getResource("models/GFE/models").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/drcsim").getFile());
      
      
      // Graphics
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo/media/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/construction_barrel/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/lamp_post/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/speed_limit_sign/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_1/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_2/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_3/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_4/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_5/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_1/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_2/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_3/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_4/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/stepping_block/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/cordless_drill/materials/textures/").getFile());
      resourceDirectories.add(myClass.getResource("models//GFE/models/starting_pen/materials/textures/").getFile());
      
      
      
      
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>();
   }

   public void createAndSetContactControllerToARobot()
   {
   }

   public void addContactPoints(ExternalForcePoint[] externalForcePoints)
   {
   }

   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }

}
