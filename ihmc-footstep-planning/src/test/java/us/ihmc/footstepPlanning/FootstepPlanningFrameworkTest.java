package us.ihmc.footstepPlanning;

import javafx.application.Application;
import javafx.stage.Stage;
import javafx.util.Pair;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Ellipsoid3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.ui.FootstepPlannerUI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools.VisibilityGraphsUnitTestDataset;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisibilityGraphsDataExporter;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

public class FootstepPlanningFrameworkTest extends Application
{
   // Set that to MAX_VALUE when visualizing. Before pushing, it has to be reset to a reasonable value.
   private static final long TIMEOUT = 100000; // Long.MAX_VALUE; // 
   // Threshold used to assert that the body path starts and ends where we asked it to.
   private static final double START_GOAL_EPSILON = 1.0e-2;

   // Whether to start the UI or not.
   private static boolean VISUALIZE = false;
   // For enabling helpful prints.
   private static boolean DEBUG = true;

   // Because we use JavaFX, there will be two instance of FootstepPlanningFrameworkTest, one for running the test and one starting the ui. The messager has to be static so both the ui and test use the same instance.
   private static JavaFXMessager messager = null;
   // Because JavaFX will create a fresh new instance of FootstepPlanningFrameworkTest, the ui has to be static so there is only one instance and we can refer to it in the test part.
   private static FootstepPlannerUI ui;

   // Default UI parameters which should be changeable on the fly
   private static final boolean showBodyPath = true;
   private static final boolean showClusterRawPoints = false;
   private static final boolean showClusterNavigableExtrusions = false;
   private static final boolean showClusterNonNavigableExtrusions = false;
   private static final boolean showRegionInnerConnections = false;
   private static final boolean showRegionInterConnections = false;

   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   private static final double walkerMarchingSpeed = 0.05;

   // For the occlusion test
   private static final int rays = 5000;
   private static final double rayLengthSquared = MathTools.square(5.0);

   public FootstepPlanningFrameworkTest()
   {
      // Did not find a better solution for starting JavaFX and still be able to move on.
//      new Thread(() -> launch()).start();


      while (ui == null)
         ThreadTools.sleep(200);

      messager = ui.getMessager();

      messager.submitMessage(UIVisibilityGraphsTopics.ShowBodyPath, showBodyPath);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterRawPoints, showClusterRawPoints);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions, showClusterNavigableExtrusions);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusions);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowNavigableRegionVisibilityMaps, showRegionInnerConnections);
      messager.submitMessage(UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap, showRegionInterConnections);

   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
         ui = new FootstepPlannerUI(primaryStage);//, messager);
         ui.show();
   }

   @Override
   public void stop() throws Exception
   {

         ui.stop();
   }

}
