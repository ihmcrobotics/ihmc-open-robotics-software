package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.RosBasedPlannerListener;
import us.ihmc.footstepPlanning.postProcessing.AreaSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.PositionSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.SwingOverRegionsPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.YoVariablesForFootstepPostProcessingParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import static us.ihmc.footstepPlanning.FootstepPlannerStatus.*;
import static us.ihmc.footstepPlanning.FootstepPlannerStatus.IDLE;

public class FootstepPlanPostProcessingModule implements CloseableAndDisposable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String name;
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final FootstepPostProcessingParametersBasics parameters;
   private final CompositeFootstepPlanPostProcessing postProcessing = new CompositeFootstepPlanPostProcessing();

   public static final String MODULE_NAME = "footstep_post_processor";

   private Consumer<FootstepPostProcessingPacket> statusCallback = result -> {
   };

   private Ros2Node ros2Node;

   public FootstepPlanPostProcessingModule(DRCRobotModel robotModel)
   {
      this(robotModel.getSimpleRobotName(),
           robotModel.getFootstepPostProcessingParameters(),
           robotModel.getWalkingControllerParameters(),
           robotModel.getContactPointParameters(),
           robotModel.getCapturePointPlannerParameters());
   }

   public FootstepPlanPostProcessingModule(String name,
                                           FootstepPostProcessingParametersBasics parameters,
                                           WalkingControllerParameters walkingControllerParameters,
                                           RobotContactPointParameters<RobotSide> contactPointParameters,
                                           ICPPlannerParameters cmpPlannerParameters)
   {
      this.name = name;
      if (parameters != null)
         this.parameters = parameters;
      else
         this.parameters = new DefaultFootstepPostProcessingParameters();

      PositionSplitFractionPostProcessingElement positionSplitFractionPostProcessingElement = new PositionSplitFractionPostProcessingElement(parameters,
                                                                                                                                             cmpPlannerParameters);
      AreaSplitFractionPostProcessingElement areaSplitFractionPostProcessingElement = new AreaSplitFractionPostProcessingElement(parameters,
                                                                                                                                 cmpPlannerParameters,
                                                                                                                                 contactPointParameters
                                                                                                                                       .getFootContactPoints());
      SwingOverRegionsPostProcessingElement swingOverRegionsPostProcessingElement = new SwingOverRegionsPostProcessingElement(parameters,
                                                                                                                              walkingControllerParameters,
                                                                                                                              registry, null);

      postProcessing.addPostProcessingElement(positionSplitFractionPostProcessingElement); // make sure this one comes before area
      postProcessing.addPostProcessingElement(areaSplitFractionPostProcessingElement); // make sure this one comes after position
      postProcessing.addPostProcessingElement(swingOverRegionsPostProcessingElement);

      isDone.set(true);
   }

   public void handleRequestPacket(FootstepPostProcessingPacket latestOutput)
   {
      FootstepPostProcessingPacket processedOutputStatus = postProcessing.postProcessFootstepPlan(latestOutput);

      statusCallback.accept(processedOutputStatus);

      isDone.set(true);
   }

   public void addStatusCallback(Consumer<FootstepPostProcessingPacket> callback)
   {
      statusCallback = statusCallback.andThen(callback);
   }

   @Override
   public void closeAndDispose()
   {
      if (ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }
   }

   public void setupWithRos(DomainFactory.PubSubImplementation pubSubImplementation)
   {
      if (ros2Node != null)
         return;

      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, MODULE_NAME);
      setupWithRos(ros2Node);
   }

   public void setupWithRos(Ros2Node ros2Node)
   {
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = ROS2Tools
            .getTopicNameGenerator(name, ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = ROS2Tools
            .getTopicNameGenerator(name, ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);

      // Parameters callback
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPostProcessingParametersPacket.class, subscriberTopicNameGenerator,
                                           s -> parameters.set(s.readNextData()));

      // Planner request callback
      ROS2Tools.createCallbackSubscription(ros2Node, FootstepPostProcessingPacket.class, subscriberTopicNameGenerator, s -> {
         FootstepPostProcessingPacket requestPacket = s.takeNextData();
         new Thread(() -> handleRequestPacket(requestPacket)).start();
      });

      // Result publisher
      IHMCROS2Publisher<FootstepPostProcessingPacket> resultPublisher = ROS2Tools
            .createPublisher(ros2Node, FootstepPostProcessingPacket.class, publisherTopicNameGenerator);
      addStatusCallback(resultPublisher::publish);
   }
}
