package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters.FootstepPostProcessingParametersReadOnly;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.parameters.YoVariablesForFootstepPostProcessingParameters;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphMessagesConverter;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlanPostProcessingToolboxController extends ToolboxController
{
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> latestFootstepPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestPacket = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final FootstepPostProcessingParametersReadOnly parameters;
   private final CompositeFootstepPlanPostProcessing postProcessing = new CompositeFootstepPlanPostProcessing();

   public FootstepPlanPostProcessingToolboxController(FootstepPostProcessingParametersBasics parameters, StatusMessageOutputManager statusOutputManager,
                                                      YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.parameters = parameters;

      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);

      StepSplitFractionPostProcessingElement splitFractionPostProcessingElement = new StepSplitFractionPostProcessingElement(parameters, registry);

      new YoVariablesForFootstepPostProcessingParameters(registry, parameters);

      postProcessing.addPostProcessingElement(splitFractionPostProcessingElement);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      isDone.set(true);
   }

   @Override
   public void updateInternal()
   {
      FootstepPlanningToolboxOutputStatus latestOutput = latestFootstepPlan.getAndSet(null);
      FootstepPlanningRequestPacket latestRequest = latestRequestPacket.getAndSet(null);

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

      FootstepPlanningToolboxOutputStatus processedOutputStatus = postProcessing.postProcessFootstepPlan(latestRequest, latestOutput);

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

   public void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus outputStatus)
   {
      latestFootstepPlan.set(outputStatus);
   }

   public void processFootstepPlanningRequest(FootstepPlanningRequestPacket requestPacket)
   {
      latestRequestPacket.set(requestPacket);
   }
}
