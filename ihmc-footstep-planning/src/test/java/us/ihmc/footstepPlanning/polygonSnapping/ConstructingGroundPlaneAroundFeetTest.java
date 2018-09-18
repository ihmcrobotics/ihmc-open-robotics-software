package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.After;
import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import static junit.framework.TestCase.assertTrue;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class ConstructingGroundPlaneAroundFeetTest
{
   private final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructingGroundPlaneAroundFeet()
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      Vector3D translation = new Vector3D(2.1, -0.3, 0.4);
      RigidBodyTransform transformToWorld = new RigidBodyTransform(new Quaternion(), translation);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.5, 0.5);
      polygon.addVertex(0.5, -0.5);
      polygon.addVertex(1.5, 0.5);
      polygon.addVertex(1.5, -0.5);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, polygon);
      planarRegionsList.addPlanarRegion(planarRegion);

      double stepWidth = 0.2;
      FootstepNode stanceFootNode = new FootstepNode(translation.getX(), translation.getY() + 0.5 * stepWidth, 0.0, RobotSide.LEFT);
      RigidBodyTransform snapTransform = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, translation.getZ()));

      FootstepNodeSnappingTools.constructGroundPlaneAroundFeet(planarRegionsList, stanceFootNode, snapTransform, stepWidth, 0.3, 0.2, 0.4);

      if(visualize)
      {
         SimulationConstructionSet scs = new SimulationConstructionSet();
         Graphics3DObject graphics = new Graphics3DObject();

         Graphics3DObjectTools.addPlanarRegionsList(graphics, planarRegionsList);
         graphics.identity();
         graphics.translate(translation);
         graphics.addCoordinateSystem(0.2);

         scs.addStaticLinkGraphics(graphics);
         scs.setGroundVisible(false);
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }
}
