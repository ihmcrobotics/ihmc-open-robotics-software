package us.ihmc.avatar;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoGraphicPlanarRegionsController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final AtomicReference<PlanarRegionsList> planarRegionsListReference = new AtomicReference<>();
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("region", 100, 150, registry);
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsListInput;

   public YoGraphicPlanarRegionsController(ROS2NodeInterface ros2Node, YoGraphicsListRegistry yoGraphicsListRegistry, ROS2Topic<PlanarRegionsListMessage> topicName)
   {
      yoGraphicsListRegistry.registerYoGraphic("Regions", yoGraphicPlanarRegionsList);
      planarRegionsListInput = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, topicName);

      planarRegionsListInput.addCallback(new Consumer<PlanarRegionsListMessage>()
      {
         @Override
         public void accept(PlanarRegionsListMessage planarRegionsListMessage)
         {
            planarRegionsListReference.set(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));

         }
      });
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public void doControl()
   {
      PlanarRegionsList newPlanarRegionsList = planarRegionsListReference.getAndSet(null);
      if (newPlanarRegionsList != null)
      {
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(newPlanarRegionsList);
      }
      yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
      yoGraphicPlanarRegionsList.update();
   }

}
