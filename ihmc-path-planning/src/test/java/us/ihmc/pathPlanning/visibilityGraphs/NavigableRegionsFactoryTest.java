package us.ihmc.pathPlanning.visibilityGraphs;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class NavigableRegionsFactoryTest
{
   // Whether to start the UI or not.
   private static boolean VISUALIZE = true;

   // For enabling helpful prints.
   private static boolean DEBUG = true;

   private static VisibilityGraphsTestVisualizerApplication visualizerApplication = null;
   // Because we use JavaFX, there will be two instance of VisibilityGraphsFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;

   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      DEBUG = (VISUALIZE || (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()));

      if (VISUALIZE)
      {
         visualizerApplication = new VisibilityGraphsTestVisualizerApplication();
         visualizerApplication.startOnAThread();

         messager = visualizerApplication.getMessager();
      }
   }

   @Test
   public void testComplexProjection()
   {
      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(-5.0, 5.0);
      groundPolygon.addVertex(5.0, -5.0);
      groundPolygon.addVertex(5.0, 5.0);
      groundPolygon.addVertex(-5.0, -5.0);
      groundPolygon.update();

      List<ConvexPolygon2D> groundPolygons = new ArrayList<>();
      groundPolygons.add(groundPolygon);

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon.getVertexBufferView().toArray(new Point2D[0]), groundPolygons);

      ConvexPolygon2D aboveRegion0 = new ConvexPolygon2D();
      aboveRegion0.addVertex(0.1, -0.5);
      aboveRegion0.addVertex(0.0, -0.6);
      aboveRegion0.addVertex(-0.2, -0.4);
      aboveRegion0.addVertex(-0.1, -0.3);
      aboveRegion0.update();

      ConvexPolygon2D aboveRegion1 = new ConvexPolygon2D();
      aboveRegion1.addVertex(0.1, -0.3);
      aboveRegion1.addVertex(0.0, -0.4);
      aboveRegion1.addVertex(-0.2, -0.2);
      aboveRegion1.addVertex(-0.1, -0.1);
      aboveRegion1.update();

      ConvexPolygon2D aboveRegion3 = new ConvexPolygon2D();
      aboveRegion3.addVertex(0.1, -0.1);
      aboveRegion3.addVertex(0.0, -0.2);
      aboveRegion3.addVertex(-0.2, -0.0);
      aboveRegion3.addVertex(-0.1, 0.1);
      aboveRegion3.update();

      ConvexPolygon2D aboveRegion4 = new ConvexPolygon2D();
      aboveRegion4.addVertex(0.1, 0.1);
      aboveRegion4.addVertex(0.0, -0.0);
      aboveRegion4.addVertex(-0.2, 0.2);
      aboveRegion4.addVertex(-0.1, 0.3);
      aboveRegion4.update();

      List<ConvexPolygon2D> abovePolygons = new ArrayList<>();
      abovePolygons.add(aboveRegion0);
      abovePolygons.add(aboveRegion1);
      abovePolygons.add(aboveRegion3);
      abovePolygons.add(aboveRegion4);

      RigidBodyTransform aboveTransform = new RigidBodyTransform();
      aboveTransform.setTranslation(0.0, 0.0, 0.15);

      Point2D[] aboveConcaveHull = new Point2D[16];

      int counter = 0;
      for (ConvexPolygon2D polygon : abovePolygons)
      {
         for (Point2DReadOnly point : polygon.getVertexBufferView())
         {
            aboveConcaveHull[counter] = new Point2D(point);
            counter++;
         }
      }

      PlanarRegion aboveRegion = new PlanarRegion(aboveTransform, aboveConcaveHull, abovePolygons);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      planarRegionsList.addPlanarRegion(groundRegion);
      planarRegionsList.addPlanarRegion(aboveRegion);

      VisibilityGraphsParameters visibilityGraphsParameters = new DefaultVisibilityGraphParameters();
      List<NavigableRegion> navigableRegions = NavigableRegionsFactory.createNavigableRegions(planarRegionsList.getPlanarRegionsAsList(), visibilityGraphsParameters);

      List<VisibilityMapWithNavigableRegion> visibilityMapWithNavigableRegions = new ArrayList<>();
      for (NavigableRegion navigableRegion : navigableRegions)
      {
         visibilityMapWithNavigableRegions.add(new VisibilityMapWithNavigableRegion(navigableRegion));
      }

      messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, planarRegionsList);
      messager.submitMessage(UIVisibilityGraphsTopics.NavigableRegionData, visibilityMapWithNavigableRegions);
      ThreadTools.sleepForever();
   }
}
