package us.ihmc.darpaRoboticsChallenge.environment;

import java.util.ArrayList;
import java.util.List;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;

public abstract class DRCWorld 
{
   private static final Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
   protected final ArrayList<String> resourceDirectories = new ArrayList<String>();

   public DRCWorld()
   {

      // DRC resources
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/").getFile());
      resourceDirectories.add(myClass.getResource("models").getFile());

      // Gazebo resources
      resourceDirectories.add(myClass.getResource("models/GFE/models").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/drcsim").getFile());
      
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/").getFile());

      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/golf_cart/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_heightmap_3/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_1/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/drc_terrain/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_heightmap_2/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_standpipe/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_firehose_long/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_3/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_valve/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_2/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/cutout_wall/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_4/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_5/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vehicle_gate/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/mud_atlas/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_4/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_heightmap_1/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate_number_2/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_1/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_heightmap_4/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/platform_3/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/powerplant/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_valve_a/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_firehose_long_a/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/grass_plane/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/stepping_block/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_standpipe_a/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/pavement_plane/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/atlas_description/atlas/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/multisense_sl_description/multisense_sl_description/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/asphalt_plane/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/beer/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_valve_wall/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_3/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/gas_station/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/cordless_drill/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/jersey_barrier/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/mailbox/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_maze_wall_triple_holes_120/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/turtlebot/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_handle_wheel_valve/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_wood_slats/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_2/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/lamp_post/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/polaris_ranger_ev/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_elevated_floor_120/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/coke_can/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_2x6/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/cinder_block/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/kinect/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_4x4x40/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/stop_sign/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/mud_box/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/pioneer2dx/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/construction_barrel/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/cinder_block_2/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/create/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/starting_pen/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/hammer/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_simple_ramp_120/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/speed_limit_sign/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_1/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/dumpster/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/kitchen_dining/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/brick_box_3x1x3/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/grey_wall/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_maze_wall_240/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/powerplant/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_truss/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/willowgarage/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/construction_cone/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_4x4x20/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_wheel_valve_large/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_2x4/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/fast_food/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/drc_practice_ball_valve/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_stairs_120/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/nist_maze_wall_120/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/atlas_description/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/drcsim/media/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/multisense_sl_description/materials/textures").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo/media/materials/textures").getFile());

      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_firehose_long/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/gate/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_valve/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/cutout_wall/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/mud_atlas/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_valve_a/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/vrc_firehose_long_a/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/grass_plane/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo_models/environments/pavement_plane/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/asphalt_plane/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/beer/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/youbot/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_3/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/gas_station/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/mailbox/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_2/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/polaris_ranger_ev/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/stop_sign/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/mud_box/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/starting_pen/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/house_1/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/dumpster/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/brick_box_3x1x3/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/grey_wall/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/models/fast_food/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/drcsim/media/materials/scripts").getFile());
      resourceDirectories.add(myClass.getResource("models/GFE/gazebo/media/materials/scripts").getFile());
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
