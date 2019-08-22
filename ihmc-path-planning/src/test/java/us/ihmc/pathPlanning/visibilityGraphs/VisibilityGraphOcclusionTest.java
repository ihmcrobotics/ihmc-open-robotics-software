package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class VisibilityGraphOcclusionTest
{
   private static final long timeout = 30000;
   private boolean visualize = false;
   private PlanarRegionsList occludedEnvironmentWithAGoalPlane;
   private PlanarRegionsList occludedEnvironmentWithoutAGoalPlane;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      occludedEnvironmentWithAGoalPlane = simpleOccludedEnvironment(true);
      occludedEnvironmentWithoutAGoalPlane = simpleOccludedEnvironment(false);
   }

   @Test
   public void testVisibilityGraphWithOcclusion()
   {
      runTest(occludedEnvironmentWithAGoalPlane);
   }

   @Test
   public void testVisibilityGraphWithOcclusionAndNoGoalPlane()
   {
      runTest(occludedEnvironmentWithoutAGoalPlane);
   }

   private void runTest(PlanarRegionsList planarRegionsList)
   {
      Point3D start = new Point3D();
      Point3D goal = new Point3D(2.0, -1.0, 0.0);

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(planarRegionsList.getPlanarRegionsAsList());
      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPathWithOcclusions(start, goal);

      if(visualize)
      {
         visualize(path, planarRegionsList, start, goal);
      }
   }

   private static void visualize(List<Point3DReadOnly> path, PlanarRegionsList planarRegionsList, Point3D start, Point3D goal)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegionsList, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
      scs.setGroundVisible(false);

      graphics3DObject.identity();
      graphics3DObject.translate(start);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Green());

      graphics3DObject.identity();
      graphics3DObject.translate(goal);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Red());

      if(path != null)
      {
         for (int i = 0; i < path.size(); i++)
         {
            Point3DReadOnly point = path.get(i);

            graphics3DObject.identity();
            graphics3DObject.translate(point);
            graphics3DObject.addSphere(0.1, YoAppearance.Orange());

            if(i != path.size() - 1)
            {
               Point3DReadOnly nextPoint = path.get(i + 1);
               Vector3D direction = new Vector3D(nextPoint);
               direction.sub(point);
               int pathPoints = (int) Math.round(point.distance(nextPoint) / 0.05);

               for (int j = 1; j < pathPoints; j++)
               {
                  Vector3D offset = new Vector3D(direction);
                  offset.scaleAdd(((double) j) / pathPoints, point);

                  graphics3DObject.identity();
                  graphics3DObject.translate(offset);
                  graphics3DObject.addSphere(0.025, YoAppearance.Orange());
               }
            }
         }
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraPosition(-7.0, -1.0, 25.0);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   private static PlanarRegionsList simpleOccludedEnvironment(boolean includeGoalPlane)
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(1.0, -1.0, 0.5);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(0.9, 1.9);

      if (includeGoalPlane)
      {
         generator.identity();
         generator.translate(2.0, -1.0, 0.0);
         generator.addRectangle(1.0, 1.0);
      }

      return generator.getPlanarRegionsList();
   }

   private class TestParameters extends DefaultVisibilityGraphParameters
   {
      @Override
      public double getSearchHostRegionEpsilon()
      {
         return 0;
      }
   }
}
