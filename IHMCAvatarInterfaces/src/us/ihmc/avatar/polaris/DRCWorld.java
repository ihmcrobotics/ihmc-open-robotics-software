package us.ihmc.avatar.polaris;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;

public abstract class DRCWorld 
{
   private static final Class<DRCRobotSDFLoader> myClass = DRCRobotSDFLoader.class;
   protected final ArrayList<String> resourceDirectories = new ArrayList<String>();

   public DRCWorld()
   {

      // DRC resources
      resourceDirectories.add("models/GFE/gazebo");
      resourceDirectories.add("models/GFE/");
      resourceDirectories.add("models");
      resourceDirectories.add("models/polaris_ranger_xp900_no_roll_cage/");
      resourceDirectories.add("models/polaris_ranger_xp900_no_roll_cage/meshes/");
      resourceDirectories.add("models/polaris_ranger_xp900_no_roll_cage/materials/");
      resourceDirectories.add("models/polaris_ranger_xp900_no_roll_cage/scripts/");
      resourceDirectories.add("models/polaris_ranger_xp900/");
      resourceDirectories.add("models/polaris_ranger_xp900/meshes/");
      resourceDirectories.add("models/polaris_ranger_xp900/materials/");
      resourceDirectories.add("models/polaris_ranger_xp900/materials/scripts/");
      resourceDirectories.add("models/polaris_ranger_ev/");
      resourceDirectories.add("models/polaris_ranger_ev/meshes/");
      resourceDirectories.add("models/polaris_ranger_ev/materials/");
      resourceDirectories.add("models/polaris_ranger_ev/materials/scripts/");

      // Gazebo resources
      resourceDirectories.add("models/GFE/models");
      resourceDirectories.add("models/GFE/drcsim");
      
      resourceDirectories.add("models/GFE/gazebo_models/environments/");

      resourceDirectories.add("models/GFE/gazebo_models/environments/golf_cart/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_heightmap_3/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate_number_1/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/drc_terrain/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_heightmap_2/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_standpipe/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_firehose_long/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate_number_3/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_valve/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/platform_2/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/cutout_wall/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate_number_4/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate_number_5/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vehicle_gate/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/mud_atlas/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/platform_4/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_heightmap_1/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate_number_2/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/platform_1/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_heightmap_4/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/platform_3/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/powerplant/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_valve_a/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_firehose_long_a/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/grass_plane/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/stepping_block/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_standpipe_a/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/environments/pavement_plane/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/atlas_description/atlas/materials/textures");
      resourceDirectories.add("models/GFE/gazebo_models/multisense_sl_description/multisense_sl_description/materials/textures");
      resourceDirectories.add("models/models/asphalt_plane/materials/textures");
      resourceDirectories.add("models/models/beer/materials/textures");
      resourceDirectories.add("models/models/drc_practice_valve_wall/materials/textures");
      resourceDirectories.add("models/models/house_3/materials/textures");
      resourceDirectories.add("models/models/gas_station/materials/textures");
      resourceDirectories.add("models/models/cordless_drill/materials/textures");
      resourceDirectories.add("models/models/jersey_barrier/materials/textures");
      resourceDirectories.add("models/models/mailbox/materials/textures");
      resourceDirectories.add("models/models/nist_maze_wall_triple_holes_120/materials/textures");
      resourceDirectories.add("models/models/turtlebot/materials/textures");
      resourceDirectories.add("models/models/drc_practice_handle_wheel_valve/materials/textures");
      resourceDirectories.add("models/models/drc_practice_wood_slats/materials/textures");
      resourceDirectories.add("models/models/house_2/materials/textures");
      resourceDirectories.add("models/models/lamp_post/materials/textures");
      resourceDirectories.add("models/models/polaris_ranger_ev/materials/textures");
      resourceDirectories.add("models/models/nist_elevated_floor_120/materials/textures");
      resourceDirectories.add("models/models/coke_can/materials/textures");
      resourceDirectories.add("models/models/drc_practice_2x6/materials/textures");
      resourceDirectories.add("models/models/cinder_block/materials/textures");
      resourceDirectories.add("models/models/kinect/materials/textures");
      resourceDirectories.add("models/models/drc_practice_4x4x40/materials/textures");
      resourceDirectories.add("models/models/stop_sign/materials/textures");
      resourceDirectories.add("models/models/mud_box/materials/textures");
      resourceDirectories.add("models/models/pioneer2dx/materials/textures");
      resourceDirectories.add("models/models/construction_barrel/materials/textures");
      resourceDirectories.add("models/models/cinder_block_2/materials/textures");
      resourceDirectories.add("models/models/create/materials/textures");
      resourceDirectories.add("models/models/starting_pen/materials/textures");
      resourceDirectories.add("models/models/hammer/materials/textures");
      resourceDirectories.add("models/models/nist_simple_ramp_120/materials/textures");
      resourceDirectories.add("models/models/speed_limit_sign/materials/textures");
      resourceDirectories.add("models/models/house_1/materials/textures");
      resourceDirectories.add("models/models/dumpster/materials/textures");
      resourceDirectories.add("models/models/kitchen_dining/materials/textures");
      resourceDirectories.add("models/models/brick_box_3x1x3/materials/textures");
      resourceDirectories.add("models/models/grey_wall/materials/textures");
      resourceDirectories.add("models/models/nist_maze_wall_240/materials/textures");
      resourceDirectories.add("models/models/powerplant/materials/textures");
      resourceDirectories.add("models/models/drc_practice_truss/materials/textures");
      resourceDirectories.add("models/models/willowgarage/materials/textures");
      resourceDirectories.add("models/models/construction_cone/materials/textures");
      resourceDirectories.add("models/models/drc_practice_4x4x20/materials/textures");
      resourceDirectories.add("models/models/drc_practice_wheel_valve_large/materials/textures");
      resourceDirectories.add("models/models/drc_practice_2x4/materials/textures");
      resourceDirectories.add("models/models/fast_food/materials/textures");
      resourceDirectories.add("models/models/drc_practice_ball_valve/materials/textures");
      resourceDirectories.add("models/models/nist_stairs_120/materials/textures");
      resourceDirectories.add("models/models/nist_maze_wall_120/materials/textures");
      resourceDirectories.add("models/GFE/atlas_description/materials/textures");
      resourceDirectories.add("models/drcsim/media/materials/textures");
      resourceDirectories.add("models/GFE/multisense_sl_description/materials/textures");
      resourceDirectories.add("models/gazebo/media/materials/textures");

      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_firehose_long/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/gate/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_valve/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/cutout_wall/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/mud_atlas/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_valve_a/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/vrc_firehose_long_a/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/grass_plane/materials/scripts");
      resourceDirectories.add("models/GFE/gazebo_models/environments/pavement_plane/materials/scripts");
      resourceDirectories.add("models/models/asphalt_plane/materials/scripts");
      resourceDirectories.add("models/models/beer/materials/scripts");
      resourceDirectories.add("models/models/youbot/materials/scripts");
      resourceDirectories.add("models/models/house_3/materials/scripts");
      resourceDirectories.add("models/models/gas_station/materials/scripts");
      resourceDirectories.add("models/models/mailbox/materials/scripts");
      resourceDirectories.add("models/models/house_2/materials/scripts");
      resourceDirectories.add("models/models/polaris_ranger_ev/materials/scripts");
      resourceDirectories.add("models/models/stop_sign/materials/scripts");
      resourceDirectories.add("models/models/mud_box/materials/scripts");
      resourceDirectories.add("models/models/starting_pen/materials/scripts");
      resourceDirectories.add("models/models/house_1/materials/scripts");
      resourceDirectories.add("models/models/dumpster/materials/scripts");
      resourceDirectories.add("models/models/brick_box_3x1x3/materials/scripts");
      resourceDirectories.add("models/models/grey_wall/materials/scripts");
      resourceDirectories.add("models/models/fast_food/materials/scripts");
      resourceDirectories.add("models/drcsim/media/materials/scripts");
      resourceDirectories.add("models/gazebo/media/materials/scripts");
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
