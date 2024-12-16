package us.ihmc.rdx.ui.processes;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.sensors.realsense.MapSensePlanarRegionROS1Bridge;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import java.util.function.Supplier;

public class MapSenseBridgeProcess extends RestartableProcess
{
   private final Supplier<DRCRobotModel> robotModelSupplier;
   private MapSensePlanarRegionROS1Bridge mapSensePlanarRegionsBridge;
   private RosMainNode mapsenseBridgeROS1Node;
   private ROS2Node mapSenseBridgeROS2Node;

   public MapSenseBridgeProcess(Supplier<DRCRobotModel> robotModelSupplier)
   {
      this.robotModelSupplier = robotModelSupplier;
   }

   @Override
   protected void startInternal()
   {
      mapsenseBridgeROS1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "mapsense_bridge");
      mapSenseBridgeROS2Node = new ROS2NodeBuilder().build("mapsense_bridge");
      mapSensePlanarRegionsBridge = new MapSensePlanarRegionROS1Bridge(robotModelSupplier.get(), mapsenseBridgeROS1Node, mapSenseBridgeROS2Node);
      mapsenseBridgeROS1Node.execute();
   }

   @Override
   protected void stopInternal()
   {
      mapsenseBridgeROS1Node.shutdown();
      mapSenseBridgeROS2Node.destroy();
      mapSensePlanarRegionsBridge.destroy();
   }

   @Override
   public String getName()
   {
      return "MapSense Bridge";
   }
}
