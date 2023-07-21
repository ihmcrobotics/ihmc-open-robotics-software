package us.ihmc.rdx.ui.affordances;

import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.SwingPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.List;

public class RDXSwingPlanningModule
{
   private final SwingPlanningModule swingPlanningModule;
   private final ROS2SyncedRobotModel syncedRobot;
   private PlanarRegionsListMessage planarRegionsListMessage;
   private PlanarRegionsList planarRegionsList;
   private HeightMapMessage heightMapMessage;
   private SwingPlannerParametersReadOnly swingPlannerParameters;

   private final SideDependentList<FramePose3DBasics> startFootPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean isCurrentlyPlanning = false;

   public RDXSwingPlanningModule(ROS2SyncedRobotModel syncedRobot,
                                 FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.syncedRobot = syncedRobot;

      swingPlanningModule = new SwingPlanningModule(footstepPlannerParameters, swingPlannerParameters, walkingControllerParameters, footPolygons);
   }

   public void setPlanarRegionList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setHeightMapData(HeightMapMessage heightMapData)
   {
      this.heightMapMessage = heightMapData;
   }

   public void setSwingPlannerParameters(SwingPlannerParametersReadOnly swingPlannerParameters)
   {
      this.swingPlannerParameters = swingPlannerParameters;
   }

   public void setInitialFeet()
   {
      startFootPoses.forEach((side, pose) -> pose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side)));
   }

   public void updateAysnc(List<RDXInteractableFootstep> footstepPlan, SwingPlannerType swingPlannerType)
   {
      executorService.clearQueueAndExecute(() -> update(footstepPlan, swingPlannerType));
   }

   public synchronized void update(List<RDXInteractableFootstep> footstepPlan, SwingPlannerType swingPlannerType)
   {
      isCurrentlyPlanning = true;
      setInitialFeet();
      FootstepPlan tempPlan = createFakeFootstepPlan(footstepPlan);

      if (planarRegionsList == null && planarRegionsListMessage != null)
      {
         planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      }
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      swingPlanningModule.getSwingPlannerParameters().set(swingPlannerParameters);
      swingPlanningModule.computeSwingWaypoints(planarRegionsList,
                                                heightMapData,
                                                tempPlan,
                                                startFootPoses,
                                                swingPlannerType);

      for (int i = 0; i < footstepPlan.size() && i < swingPlanningModule.getSwingTrajectories().size(); i++)
      {
         footstepPlan.get(i).updatePlannedTrajectory(Pair.of(tempPlan.getFootstep(i), swingPlanningModule.getSwingTrajectories().get(i)));
      }
      isCurrentlyPlanning = false;
   }

   private FootstepPlan createFakeFootstepPlan(List<RDXInteractableFootstep> footstepPlan)
   {
      FootstepPlan fakeFootstepPlan = new FootstepPlan();
      footstepPlan.forEach(footstep -> fakeFootstepPlan.addFootstep(new PlannedFootstep(footstep.getPlannedFootstep())));

      return fakeFootstepPlan;
   }

   public void destroy()
   {
      executorService.destroy();
   }

   public boolean getIsCurrentlyPlanning()
   {
      return isCurrentlyPlanning;
   }
}
