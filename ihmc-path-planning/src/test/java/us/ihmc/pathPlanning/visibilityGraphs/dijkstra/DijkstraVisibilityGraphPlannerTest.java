package us.ihmc.pathPlanning.visibilityGraphs.dijkstra;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;

import java.util.ArrayList;
import java.util.List;

import static junit.framework.TestCase.assertTrue;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class DijkstraVisibilityGraphPlannerTest
{
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 3000)
   public void testSimplePlanarGraph()
   {
      DijkstraVisibilityGraphPlanner planner = new DijkstraVisibilityGraphPlanner();

      ConnectionPoint3D p1 = new ConnectionPoint3D(0.0, 0.0, 0.0, 0);
      ConnectionPoint3D p2 = new ConnectionPoint3D(0.5, 0.0, 0.0, 0);
      ConnectionPoint3D p3 = new ConnectionPoint3D(0.25, 0.25, 0.0, 1);
      ConnectionPoint3D p4 = new ConnectionPoint3D(0.5, 0.5, 0.0, 0);
      ConnectionPoint3D p5 = new ConnectionPoint3D(1.0, 0.0, 0.0, 1);

      VisibilityMap visibilityMap = new VisibilityMap();
      visibilityMap.addConnection(new Connection(p1, p2));
      visibilityMap.addConnection(new Connection(p1, p3));
      visibilityMap.addConnection(new Connection(p2, p4));
      visibilityMap.addConnection(new Connection(p3, p4));
      visibilityMap.addConnection(new Connection(p4, p5));
      VisibilityMapHolder visibilityMapHolder = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return 0;
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            return null;
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return visibilityMap;
         }
      };
      List<VisibilityMapHolder> visibilityMapHolders = new ArrayList<>();
      visibilityMapHolders.add(visibilityMapHolder);

      List<Point3DReadOnly> path = planner.calculatePath(p1, p5, visibilityMapHolders);

      assertTrue(path.size() == 4);
      assertTrue(path.get(0).equals(p1));
      assertTrue(path.get(1).equals(p3));
      assertTrue(path.get(2).equals(p4));
      assertTrue(path.get(3).equals(p5));
   }
}
