package us.ihmc.pathPlanning.visibilityGraphs;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.util.List;

public class VisibilityGraphOcclusionTest
{
   private static final long timeout = 30000;
   private boolean visualize = true;
   private PlanarRegionsList simpleOccludedEnvironment;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      simpleOccludedEnvironment = simpleOccludedEnvironment();
   }

   @Test(timeout = timeout)
   public void testVisibilityGraphWithNoGoalPlane()
   {
      Point3D start = new Point3D();
      Point3D goal = new Point3D(2.0, 0.0, 0.0);

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(simpleOccludedEnvironment.getPlanarRegionsAsList());
      List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPathWithOcclussions(start, goal);

      if(visualize)
      {
         visualize(path, start, goal);
      }
   }

   private void visualize(List<Point3DReadOnly> path, Point3D start, Point3D goal)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.8);
      graphics3DObject.addPlanarRegionsList(simpleOccludedEnvironment, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
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
            graphics3DObject.addSphere(0.1, YoAppearance.OldLace());

            if(i != path.size() - 1)
            {
               Point3DReadOnly nextPoint = path.get(i);
               Vector3D direction = new Vector3D(nextPoint);
               direction.sub(point);

               for (int j = 1; j < 25; j++)
               {
                  Vector3D offset = new Vector3D(direction);
                  offset.scaleAdd(j / 25.0, point);

                  graphics3DObject.identity();
                  graphics3DObject.translate(offset);
                  graphics3DObject.addSphere(0.025, YoAppearance.OldLace());
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

   private static PlanarRegionsList simpleOccludedEnvironment()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0, 4.0);

      generator.translate(1.0, 0.0, 0.5);
      generator.rotate(0.5 * Math.PI, Axis.Y);

      generator.addRectangle(0.9, 0.9);
      return generator.getPlanarRegionsList();
   }
}
