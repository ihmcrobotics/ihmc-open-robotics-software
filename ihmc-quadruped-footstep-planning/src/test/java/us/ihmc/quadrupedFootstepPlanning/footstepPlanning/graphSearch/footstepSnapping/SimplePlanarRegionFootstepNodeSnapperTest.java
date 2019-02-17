package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.FootstepPlanningRandomTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionsListPointSnapperTest;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SimplePlanarRegionFootstepNodeSnapperTest
{
   private final Random random = new Random(1209L);
   private final double epsilon = 1e-8;
   private final SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(new DefaultFootstepPlannerParameters());
   private final ConvexPolygon2D unitSquare = new ConvexPolygon2D();

   private static final int iters = 100;

   @BeforeEach
   public void setup()
   {
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.update();
   }

   @Test
   public void testIdentity()
   {
      double frontLeftX = -0.3;
      double frontLeftY = 2.2;
      double frontRightX = -0.35;
      double frontRightY = 1.9;
      double hindLeftX = -0.85;
      double hindLeftY = 2.1;
      double hindRightX = -0.8;
      double hindRightY = 1.8;
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                 hindRightY, frontLeftX - hindRightX, frontLeftY - hindRightY);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @Test
   public void testVerticalTranslation()
   {
      double frontLeftX = 2.5;
      double frontLeftY = -0.5;
      double frontRightX = 2.3;
      double frontRightY = -0.9;
      double hindLeftX = 1.5;
      double hindLeftY = -0.4;
      double hindRightX = 1.4;
      double hindRightY = -0.8;
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                 hindRightY, frontLeftX - hindRightX, frontLeftY - hindRightY);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setTranslationZ(-1.0);

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @Test
   public void testSimpleRotation()
   {
      double frontLeftX = 0.25;
      double frontLeftY = 0.25;
      double frontRightX = 0.25;
      double frontRightY = -0.25;
      double hindLeftX = -0.25;
      double hindLeftY = 0.25;
      double hindRightX = -0.25;
      double hindRightY = -0.25;
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                 hindRightY, frontLeftX - hindRightX, frontLeftY - hindRightY);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @Test
   public void testSimpleTranslationAndRotation()
   {
      double frontLeftX = 1.1;
      double frontLeftY = 0.0;
      double frontRightX = 1.1;
      double frontRightY = -0.5;
      double hindLeftX = 0.1;
      double hindLeftY = 0.0;
      double hindRightX = 0.1;
      double hindRightY = -0.5;
      FootstepNode nodeToSnap = new FootstepNode(RobotQuadrant.FRONT_LEFT, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                 hindRightY, frontLeftX - hindRightX, frontLeftY - hindRightY);

      QuadrantDependentList<RigidBodyTransform> nodeTransforms = new QuadrantDependentList<>();
      QuadrantDependentList<RigidBodyTransform> transformsToWorld = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform nodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, nodeToSnap, nodeTransform);

         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         transformToWorld.setRotation(new AxisAngle(0.0, 1.0, 0.0, 0.25 * Math.PI));
         transformToWorld.setTranslationZ(-1.0);

         nodeTransforms.put(robotQuadrant, nodeTransform);
         transformsToWorld.put(robotQuadrant, transformToWorld);
      }

      doAFullFootholdTest(transformsToWorld, nodeToSnap);
   }

   @Test
   public void testRandomFullFootholds()
   {
      RigidBodyTransform nodeToWorldTransform = new RigidBodyTransform();

      for (int i = 0; i < iters; i++)
      {
         QuadrantDependentList<RigidBodyTransform> regionToWorldTransforms = new QuadrantDependentList<>();

         FootstepNode node = FootstepPlanningRandomTools.createRandomFootstepNode(random, 5.0);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, node, nodeToWorldTransform);

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
         FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, nodeToSnap, nodeTransform);

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
