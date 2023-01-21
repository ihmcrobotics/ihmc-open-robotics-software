package us.ihmc.rdx.ui.affordances;

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
   private HeightMapMessage heightMapMessage;

   private final SideDependentList<FramePose3DBasics> startFootPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);


   public RDXSwingPlanningModule(ROS2SyncedRobotModel syncedRobot,
                                 FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.syncedRobot = syncedRobot;

      swingPlanningModule = new SwingPlanningModule(footstepPlannerParameters, swingPlannerParameters, walkingControllerParameters, footPolygons);
   }

   public void setPlanarRegionList(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.planarRegionsListMessage = planarRegionsListMessage;
   }

   public void setHeightMapData(HeightMapMessage heightMapData)
   {
      this.heightMapMessage = heightMapData;
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
      setInitialFeet();
      FootstepPlan tempPlan = createFakeFootstepPlan(footstepPlan);

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      swingPlanningModule.computeSwingWaypoints(planarRegionsList,
                                                heightMapData,
                                                tempPlan,
                                                startFootPoses,
                                                swingPlannerType);

      for (int i = 0; i < footstepPlan.size(); i++)
      {
         footstepPlan.get(i).updatePlannedTrajectory(tempPlan.getFootstep(i));
      }
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

}
