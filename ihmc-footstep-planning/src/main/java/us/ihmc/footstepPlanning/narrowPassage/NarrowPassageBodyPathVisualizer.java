package us.ihmc.footstepPlanning.narrowPassage;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class NarrowPassageBodyPathVisualizer
{
   public NarrowPassageBodyPathVisualizer(DataSetName dataSetName)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.changeBufferSize(64000);
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      Graphics3DObject planarRegionsGraphic = new Graphics3DObject();
      Graphics3DObjectTools.addPlanarRegionsList(planarRegionsGraphic, planarRegionsList, YoAppearance.LightGray());
      scs.addStaticLinkGraphics(planarRegionsGraphic);

      VisibilityGraphsParametersReadOnly parameters = new DefaultVisibilityGraphParameters();
      FootstepPlannerParametersReadOnly footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      NarrowPassageBodyPathPlanner narrowPassageBodyPathPlanner = new NarrowPassageBodyPathPlanner(parameters,
                                                                                                   footstepPlannerParameters,
                                                                                                   graphicsListRegistry,
                                                                                                   scs,
                                                                                                   scs.getRootRegistry());
      narrowPassageBodyPathPlanner.setPlanarRegionsList(planarRegionsList);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      Pose3D startPose = new Pose3D();
      Pose3D endPose = new Pose3D();

      startPose.getPosition().set(dataSet.getPlannerInput().getStartPosition());
      startPose.getOrientation().setToYawOrientation(dataSet.getPlannerInput().getStartYaw());
      endPose.getPosition().set(dataSet.getPlannerInput().getGoalPosition());
      endPose.getOrientation().setToYawOrientation(dataSet.getPlannerInput().getGoalYaw());

      narrowPassageBodyPathPlanner.computePlan(startPose, endPose);

      scs.startOnAThread();
      scs.setGroundVisible(false);

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
//      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_JSC_78cm;
      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_IHMC_65cm;
//      DataSetName dataSetName = DataSetName._20190220_172417_Jersey_Barriers_IHMC_55cm;

      new NarrowPassageBodyPathVisualizer(dataSetName);
   }
}
