package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.*;
import us.ihmc.footstepPlanning.postProcessing.AreaSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.PositionSplitFractionPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.SwingOverRegionsPostProcessingElement;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.footstepPlanning.postProcessing.parameters.YoVariablesForFootstepPostProcessingParameters;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlanPostProcessingToolboxController extends ToolboxController
{
   private final AtomicReference<FootstepPostProcessingPacket> latestFootstepPlan = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final FootstepPostProcessingParametersBasics parameters;
   private final CompositeFootstepPlanPostProcessing postProcessing = new CompositeFootstepPlanPostProcessing();

   public FootstepPlanPostProcessingToolboxController(FootstepPostProcessingParametersBasics parameters,
                                                      WalkingControllerParameters walkingControllerParameters,
                                                      RobotContactPointParameters<RobotSide> contactPointParameters,
                                                      ICPPlannerParameters cmpPlannerParameters, StatusMessageOutputManager statusOutputManager,
                                                      YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.parameters = parameters;

      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);
      PositionSplitFractionPostProcessingElement positionSplitFractionPostProcessingElement = new PositionSplitFractionPostProcessingElement(parameters, cmpPlannerParameters);
      AreaSplitFractionPostProcessingElement areaSplitFractionPostProcessingElement = new AreaSplitFractionPostProcessingElement(parameters, cmpPlannerParameters,
                                                                                                                                 contactPointParameters.getFootContactPoints());
      SwingOverRegionsPostProcessingElement swingOverRegionsPostProcessingElement = new SwingOverRegionsPostProcessingElement(parameters,
                                                                                                                              walkingControllerParameters,
                                                                                                                              registry, graphicsListRegistry);

      new YoVariablesForFootstepPostProcessingParameters(registry, parameters);

      postProcessing.addPostProcessingElement(positionSplitFractionPostProcessingElement); // make sure this one comes before area
      postProcessing.addPostProcessingElement(areaSplitFractionPostProcessingElement); // make sure this one comes after position
      postProcessing.addPostProcessingElement(swingOverRegionsPostProcessingElement);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      isDone.set(true);
   }

   @Override
   public void updateInternal()
   {
      FootstepPostProcessingPacket latestOutput = latestFootstepPlan.getAndSet(null);

      PlanarRegionsListMessage planarRegionsListMessage = latestOutput.getPlanarRegionsList();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }


      if (planarRegionsList.isPresent())
      {
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList.get());
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
      }
      else
      {
         yoGraphicPlanarRegionsList.clear();
      }

      FootstepPostProcessingPacket processedOutputStatus = postProcessing.postProcessFootstepPlan(latestOutput);

      reportMessage(processedOutputStatus);

      finishUp();
   }

   public void finishUp()
   {
      if (DEBUG)
         LogTools.info("Finishing up the planner");
      isDone.set(true);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);

      return true;
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   public void processPostProcessingPacket(FootstepPostProcessingPacket outputStatus)
   {
      latestFootstepPlan.set(outputStatus);
   }

   public void processFootstepPostProcessingParameters(FootstepPostProcessingParametersPacket packet)
   {
      parameters.set(packet);
   }
}
