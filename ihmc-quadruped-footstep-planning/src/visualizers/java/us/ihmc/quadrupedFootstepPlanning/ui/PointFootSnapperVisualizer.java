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
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotQuadrant;
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

      DefaultPawStepPlannerParameters parameters = new DefaultPawStepPlannerParameters();
      Graphics3DObject snappedNodeGraphics = new Graphics3DObject();
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, parameters::getProjectInsideDistance,
                                                                                      parameters::getProjectInsideUsingConvexHull, true);
      snapper.setPlanarRegions(planarRegionsList);

      int minXIndex = (int) Math.round(minX / PawNode.gridSizeXY);
      int maxXIndex = (int) Math.round(maxX / PawNode.gridSizeXY);
      int minYIndex = (int) Math.round(minY / PawNode.gridSizeXY);
      int maxYIndex = (int) Math.round(maxY / PawNode.gridSizeXY);

      for (int i = minXIndex; i <= maxXIndex; i++)
      {
         for (int j = minYIndex; j <= maxYIndex; j++)
         {
            PawNodeSnapData snapData = snapper.snapPawNode(RobotQuadrant.FRONT_LEFT, i, j, 0.0);
            Vector3DReadOnly snapTranslation = snapData.getSnapTransform().getTranslationVector();

            if(snapTranslation.containsNaN())
               continue;

            Point3D snappedPosition = new Point3D(i * PawNode.gridSizeXY, j * PawNode.gridSizeXY, 0.0);
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
