package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionsListPointSnapperTest;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class SimplePlanarRegionFootstepNodeSnapperTest
{
   private final Random random = new Random(1209L);
   private final double epsilon = 1e-8;
   private final SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(new DefaultFootstepPlannerParameters());
   private final ConvexPolygon2D unitSquare = new ConvexPolygon2D();

   private static final int iters = 100;

   @Before
   public void setup()
   {
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.update();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIdentity()
   {
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, -0.3, 2.2, -0.35, 1.9, -0.85, 2.1, -0.8, 1.8);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVerticalTranslation()
   {
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, 2.5, -0.5, 2.3, -0.9, 1.5, -0.4, 1.4, -0.8);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setTranslationZ(-1.0);

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleRotation()
   {
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, 0.25, 0.25, 0.25, -0.25, -0.25, 0.25, -0.25, -0.25);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleTranslationAndRotation()
   {
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, 1.1, 0.0, 1.1, -0.5, 0.1, 0.0, 0.1, -0.5);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));
         transformToWorld.setTranslationZ(-1.0);

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomFullFootholds()
   {
      RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();

      for (int i = 0; i < iters; i++)
      {
         QuadrantDependentList<RigidBodyTransform> regionToWorldTransforms = new QuadrantDependentList<>();

         FootstepNode node = FootstepNode.generateRandomFootstepNode(random, 5.0);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            FootstepNodeTools.getNodeTransform(robotQuadrant, node, nodeToWorldTransform);

            RigidBodyTransform regionToWorldTransform = new RigidBodyTransform(nodeToWorldTransform);
            double xRotation = EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI);
            double yRotation = EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI);
            regionToWorldTransform.setRotationEuler(xRotation, yRotation, 0.0);
            regionToWorldTransform.setTranslationZ(EuclidCoreRandomTools.nextDouble(random, 2.0));

            regionToWorldTransforms.put(robotQuadrant, regionToWorldTransform);
         }

         doAFullFootholdTest(regionToWorldTransforms, node);
      }
   }

   private void doAFullFootholdTest(QuadrantDependentList<RigidBodyTransform> regionToWorldFrameTransforms, FootstepNode nodeToSnap)
   {
      QuadrantDependentList<Point2D> footPositions = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(robotQuadrant, nodeToSnap, nodeTransform);

         Point2D footPosition = new Point2D();
         footPosition.applyTransform(nodeTransform, false);

         footPositions.put(robotQuadrant, footPosition);
      }

      List<PlanarRegion> planarRegions = createPlanarRegion(regionToWorldFrameTransforms, footPositions);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(nodeToSnap);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         PlanarRegionsListPointSnapperTest
               .assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapData.getSnapTransform(robotQuadrant), regionToWorldFrameTransforms.get(robotQuadrant));
      }
   }

   private List<PlanarRegion> createPlanarRegion(QuadrantDependentList<RigidBodyTransform> regionToWorldFrameTransforms,
                                                 QuadrantDependentList<Point2D> footPositions)
   {
      List<PlanarRegion> planarRegionList = new ArrayList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
         RigidBodyTransform worldToRegionTransform = new RigidBodyTransform(regionToWorldFrameTransforms.get(robotQuadrant));
         worldToRegionTransform.invert();

         Point2DReadOnly vertex = footPositions.get(robotQuadrant);
         double zHeight = getPlaneZGivenXY(regionToWorldFrameTransforms.get(robotQuadrant), vertex.getX(), vertex.getY());
         Vector4D transformPoint = new Vector4D(vertex.getX(), vertex.getY(), zHeight, 1.0);
         worldToRegionTransform.transform(transformPoint);
         planarRegionPolygon.addVertex(transformPoint.getX(), transformPoint.getY());
         planarRegionPolygon.update();

         planarRegionList.add(new PlanarRegion(regionToWorldFrameTransforms.get(robotQuadrant), planarRegionPolygon));
      }

      return planarRegionList;
   }

   // taken from PlanarRegion, should probably be made a public static method
   public double getPlaneZGivenXY(RigidBodyTransform planeToWorldTransform, double xWorld, double yWorld)
   {
      // The three components of the plane origin
      double x0 = planeToWorldTransform.getM03();
      double y0 = planeToWorldTransform.getM13();
      double z0 = planeToWorldTransform.getM23();
      // The three components of the plane normal
      double a = planeToWorldTransform.getM02();
      double b = planeToWorldTransform.getM12();
      double c = planeToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }
}
