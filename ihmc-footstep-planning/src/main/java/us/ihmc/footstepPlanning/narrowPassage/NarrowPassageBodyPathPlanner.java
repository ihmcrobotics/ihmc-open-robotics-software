package us.ihmc.footstepPlanning.narrowPassage;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NarrowPassageBodyPathPlanner
{
   // can make this dependent on the path length later
   private static int numberOfWaypoints = 20;

   private static final double bodyDepth = 0.4;
   private static final double bodyWidth = 0.7;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry;
   private final VisibilityGraphsParametersReadOnly parameters;
   private final boolean visualize;

   private PlanarRegionsList planarRegionsList;

   private final YoFramePose3D startPose = new YoFramePose3D("startPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D endPose = new YoFramePose3D("endPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D[] waypoints = new YoFramePose3D[numberOfWaypoints];

   // for visualization only
   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameVector3D startOrientation = new YoFrameVector3D("startOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D endOrientation = new YoFrameVector3D("endOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final BagOfBalls waypointPointGraphic;

   public NarrowPassageBodyPathPlanner(VisibilityGraphsParametersReadOnly parameters)
   {
      this(parameters, null, null, null);
   }

   public NarrowPassageBodyPathPlanner(VisibilityGraphsParametersReadOnly parameters,
                                       YoGraphicsListRegistry graphicsListRegistry,
                                       TickAndUpdatable tickAndUpdatable,
                                       YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.graphicsListRegistry = graphicsListRegistry;
      this.tickAndUpdatable = tickAndUpdatable;

      visualize = graphicsListRegistry != null;
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         waypoints[i] = new YoFramePose3D("waypoint" + i, ReferenceFrame.getWorldFrame(), registry);
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (visualize)
      {
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());

         YoGraphicPosition startPosition = new YoGraphicPosition("startPositionGraphic", startPose.getPosition(), 0.05, YoAppearance.Green());
         YoGraphicPosition endPosition = new YoGraphicPosition("endPositionGraphic", endPose.getPosition(), 0.05, YoAppearance.Red());

         YoGraphicVector startOrientationGraphic = new YoGraphicVector("startOrientationGraphic", startPose.getPosition(), startOrientation, 0.3);
         YoGraphicVector endOrientationGraphic = new YoGraphicVector("endOrientationGraphic", endPose.getPosition(), endOrientation, 0.3);

         waypointPointGraphic = new BagOfBalls(numberOfWaypoints, 0.03, YoAppearance.White(), registry, graphicsListRegistry);
         for (int i = 0; i < numberOfWaypoints; i++)
         {
            YoFramePose3D waypointPose = waypoints[i];
            YoGraphicCoordinateSystem waypointOrientation = new YoGraphicCoordinateSystem("waypointOrientation" + i, waypointPose.getPosition(), waypointPose.getOrientation(), 0.15);
            graphicsList.add(waypointOrientation);
         }

         graphicsList.add(startPosition);
         graphicsList.add(endPosition);
         graphicsList.add(startOrientationGraphic);
         graphicsList.add(endOrientationGraphic);

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
      }
      else
      {
         waypointPointGraphic = null;
      }
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public List<Pose3DReadOnly> computePlan(Pose3DReadOnly startPose, Pose3DReadOnly endPose)
   {
      this.startPose.set(startPose);
      this.endPose.set(endPose);

      if (visualize)
      {
         setStartAndEndGraphics();
      }

      // initialize the waypoints with simple interpolation
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double alpha = (i + 1) / (double) (numberOfWaypoints + 1);
         waypoints[i].interpolate(startPose, endPose, alpha);
      }
      updateWaypointPointGraphics();

      // TODO adjust waypoints to avoid regions
      // iteratively adjust waypoint positions and call updateWaypointPointGraphics

      List<Pose3DReadOnly> bodyPath = new ArrayList<>();
      bodyPath.add(startPose);
      bodyPath.addAll(Arrays.asList(waypoints));
      bodyPath.add(endPose);
      return bodyPath;
   }

   private void setStartAndEndGraphics()
   {
      Vector3D forwardDirection = new Vector3D(1.0, 0.0, 0.0);
      startPose.getOrientation().transform(forwardDirection);
      startOrientation.set(forwardDirection);

      forwardDirection.set(1.0, 0.0, 0.0);
      endPose.getOrientation().transform(forwardDirection);
      endOrientation.set(forwardDirection);

      tickAndUpdatable.tickAndUpdate();
   }

   private void updateWaypointPointGraphics()
   {
      if (!visualize)
      {
         return;
      }

      waypointPointGraphic.reset();
      for (int i = 0; i < waypoints.length; i++)
      {
         waypointPointGraphic.setBall(waypoints[i].getPosition());
      }

      tickAndUpdatable.tickAndUpdate();
   }
}
