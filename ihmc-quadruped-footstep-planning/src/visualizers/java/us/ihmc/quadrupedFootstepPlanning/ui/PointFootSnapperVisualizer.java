package us.ihmc.quadrupedFootstepPlanning.ui;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PointFootSnapperVisualizer
{
   private final SimulationConstructionSet scs;

   public PointFootSnapperVisualizer(PlanarRegionsList planarRegionsList)
   {
      Robot robot = new Robot("Robot");
      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.1, 1);
      addPlanarRegionsList(planarRegionsList);

      double minX = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         ConvexPolygon2D convexHull = new ConvexPolygon2D(planarRegion.getConvexHull());

         RigidBodyTransform transform = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transform);
         convexHull.translate(transform.getTranslationX(), transform.getTranslationY());

         if(convexHull.getMinX() < minX)
            minX = convexHull.getMinX();
         if(convexHull.getMaxX() > maxX)
            maxX = convexHull.getMaxX();
         if(convexHull.getMinY() < minY)
            minY = convexHull.getMinY();
         if(convexHull.getMaxY() > maxY)
            maxY = convexHull.getMaxY();
      }

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      Graphics3DObject snappedNodeGraphics = new Graphics3DObject();
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion, true);
      snapper.setPlanarRegions(planarRegionsList);

      int minXIndex = (int) Math.round(minX / FootstepNode.gridSizeXY);
      int maxXIndex = (int) Math.round(maxX / FootstepNode.gridSizeXY);
      int minYIndex = (int) Math.round(minY / FootstepNode.gridSizeXY);
      int maxYIndex = (int) Math.round(maxY / FootstepNode.gridSizeXY);

      for (int i = minXIndex; i <= maxXIndex; i++)
      {
         for (int j = minYIndex; j <= maxYIndex; j++)
         {
            FootstepNodeSnapData snapData = snapper.snapFootstepNode(i, j);
            Vector3DReadOnly snapTranslation = snapData.getSnapTransform().getTranslationVector();

            if(snapTranslation.containsNaN())
               continue;

            Point3D snappedPosition = new Point3D(i * FootstepNode.gridSizeXY, j * FootstepNode.gridSizeXY, 0.0);
            snappedPosition.add(snapTranslation);

            snappedNodeGraphics.identity();
            snappedNodeGraphics.translate(snappedPosition);
            snappedNodeGraphics.addSphere(0.01, YoAppearance.Green());
         }
      }

      scs.addStaticLinkGraphics(snappedNodeGraphics);
      scs.tickAndUpdate();

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   public void addPlanarRegionsList(PlanarRegionsList planarRegions)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegions, YoAppearance.Grey());
      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   public static void main(String[] args)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190219_182005_Random);
      new PointFootSnapperVisualizer(dataSet.getPlanarRegionsList());
   }
}
