package us.ihmc.footstepPlanning.narrowPassage;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.List;

public class NarrowPassageBodyPathVisualizer
{
   public NarrowPassageBodyPathVisualizer(DataSetName dataSetName, boolean useVisibilityMap)
   {
      FootstepPlannerParametersReadOnly footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.changeBufferSize(64000);
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      Graphics3DObject planarRegionsGraphic = new Graphics3DObject();
      Graphics3DObjectTools.addPlanarRegionsList(planarRegionsGraphic, planarRegionsList, YoAppearance.LightGray());
      scs.addStaticLinkGraphics(planarRegionsGraphic);

      Pose3D startPose = new Pose3D();
      Pose3D endPose = new Pose3D();

      startPose.getPosition().set(dataSet.getPlannerInput().getStartPosition());
      startPose.getOrientation().setToYawOrientation(dataSet.getPlannerInput().getStartYaw());
      endPose.getPosition().set(dataSet.getPlannerInput().getGoalPosition());
      endPose.getOrientation().setToYawOrientation(dataSet.getPlannerInput().getGoalYaw());

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      if (!useVisibilityMap)
      {
         NarrowPassageBodyPathPlanner narrowPassageBodyPathPlanner = new NarrowPassageBodyPathPlanner(footstepPlannerParameters,
                                                                                                      graphicsListRegistry,
                                                                                                      scs,
                                                                                                      scs.getRootRegistry());
         narrowPassageBodyPathPlanner.setPlanarRegionsList(planarRegionsList);
         narrowPassageBodyPathPlanner.computePlan(startPose, endPose);
      }
      else
      {
         // Calculated path from visibility graph calculator
         VisibilityGraphsParametersReadOnly visibilityGraphParameters = new DefaultVisibilityGraphParameters();
         BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
         VisibilityGraphPathPlanner bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor, scs.getRootRegistry());
         bodyPathPlanner.setPlanarRegionsList(planarRegionsList);
         bodyPathPlanner.setStart(startPose);
         bodyPathPlanner.setGoal(endPose);
         bodyPathPlanner.planWaypoints();
         List<FramePose3D> waypointsList = bodyPathPlanner.getWaypointsAsFramePoseList();

         // Adjust the path for narrow passages
         NarrowPassageAdjuster narrowPassageAdjuster = new NarrowPassageAdjuster(footstepPlannerParameters,
                                                                                 new FramePose3D[0],
                                                                                 scs,
                                                                                 graphicsListRegistry,
                                                                                 scs.getRootRegistry());
         narrowPassageAdjuster.setWaypoints(waypointsList);
         narrowPassageAdjuster.setPlanarRegionsList(planarRegionsList);
         narrowPassageAdjuster.makeAdjustments();
      }

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();
      scs.setGroundVisible(false);
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
//      DataSetName dataSetName = DataSetName._20191007_200400_Corridor1Wall;
      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_JSC_78cm;
//      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_IHMC_65cm;
//      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_JSC_60cm;
//      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_IHMC_55cm;

      new NarrowPassageBodyPathVisualizer(dataSetName, true);
   }
}
