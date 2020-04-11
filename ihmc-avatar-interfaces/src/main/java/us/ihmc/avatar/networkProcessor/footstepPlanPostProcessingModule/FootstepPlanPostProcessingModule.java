package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import controller_msgs.msg.dds.FootstepPostProcessingParametersPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.AreaSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.PositionSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.SwingOverRegionsPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.function.Consumer;

public class FootstepPlanPostProcessingModule implements CloseableAndDisposable
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String name;
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final FootstepPostProcessingParametersBasics parameters;
   private final CompositeFootstepPlanPostProcessing postProcessing = new CompositeFootstepPlanPostProcessing();

   private Consumer<FootstepPostProcessingPacket> statusCallback = result -> {};

   private Ros2Node ros2Node;

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

   public String getName()
   {
      return name;
   }

   public FootstepPostProcessingPacket handleRequestPacket(FootstepPostProcessingPacket latestOutput)
   {
      FootstepPostProcessingPacket processedOutputStatus = postProcessing.postProcessFootstepPlan(latestOutput);

      statusCallback.accept(processedOutputStatus);

      isDone.set(true);

      return processedOutputStatus;
   }

   public void addStatusCallback(Consumer<FootstepPostProcessingPacket> callback)
   {
      statusCallback = statusCallback.andThen(callback);
   }

   public FootstepPostProcessingParametersBasics getParameters()
   {
      return parameters;
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

   public boolean registerRosNode(Ros2Node ros2Node)
   {
      if (this.ros2Node != null)
         return false;
      this.ros2Node = ros2Node;
      return true;
   }
}
