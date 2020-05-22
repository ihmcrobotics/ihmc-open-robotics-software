package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.concaveHull.GeometryPolygonTestTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.PolygonClippingAndMerging;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.*;

public class SteppableRegionsCalculatorTest
{
   @Test
   public void testEasyJustGround()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();


      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);

      calculator.setPlanarRegions(listOfRegions);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      StepConstraintRegion expectedRegion = new StepConstraintRegion(new RigidBodyTransform(), groundPolygon);

      assertEquals(1, constraintRegions.size());
      assertTrue(expectedRegion.epsilonEquals(constraintRegions.get(0), 1e-7));
   }

   @Test
   public void testEasyGroundWithABlock()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(blockRegion);

      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      ConcavePolygon2D expectedHole =  SteppableRegionsCalculator.createObstacleExtrusion(groundRegion,
                                                                                          blockRegion,
                                                                                          extrusionDistanceCalculator,
                                                                                          Math.cos(orthogonalAngle));

      List<ConcavePolygon2D> holes = new ArrayList<>();
      holes.add(expectedHole);
      StepConstraintRegion expectedGroundRegion = new StepConstraintRegion(new RigidBodyTransform(), groundPolygon, holes);
      StepConstraintRegion expectedBlockRegion = new StepConstraintRegion(blockTransform, blockPolygon);



      assertEquals(2, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion = constraintRegions.get(0);
      StepConstraintRegion returnedBlockRegion = constraintRegions.get(1);

      assertEquals(1, returnedGroundRegion.getNumberOfHolesInRegion());
      assertTrue(expectedHole.epsilonEquals(returnedGroundRegion.getHoleInConstraintRegion(0), 1e-7));

      assertTrue(expectedGroundRegion.epsilonEquals(returnedGroundRegion, 1e-7));
      assertTrue(expectedBlockRegion.epsilonEquals(returnedBlockRegion, 1e-7));
   }

   @Test
   public void testEasyGroundWithWallThatSplitsInHalf()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(1.0, 1.0);
      blockPolygon.addVertex(1.0, -1.0);
      blockPolygon.addVertex(-1.0, 1.0);
      blockPolygon.addVertex(-1.0, -1.0);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendPitchRotation(Math.toRadians(90));

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(blockRegion);

      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      ConcavePolygon2D splittingHole =  SteppableRegionsCalculator.createObstacleExtrusion(groundRegion,
                                                                                          blockRegion,
                                                                                          extrusionDistanceCalculator,
                                                                                          Math.cos(orthogonalAngle));


      ConcavePolygon2D splitPolygon1 = new ConcavePolygon2D();
      splitPolygon1.addVertex(-1.0, -0.1);
      splitPolygon1.addVertex(1.0, -0.1);
      splitPolygon1.addVertex(1.0, -1.0);
      splitPolygon1.addVertex(-1.0, -1.0);
      ConcavePolygon2D splitPolygon2 = new ConcavePolygon2D();
      splitPolygon2.addVertex(-1.0, 0.1);
      splitPolygon2.addVertex(-1.0, 1.0);
      splitPolygon2.addVertex(1.0, 1.0);
      splitPolygon2.addVertex(1.0, 0.1);

      StepConstraintRegion expectedGroundRegion1 = new StepConstraintRegion(new RigidBodyTransform(), splitPolygon1);
      StepConstraintRegion expectedGroundRegion2 = new StepConstraintRegion(new RigidBodyTransform(), splitPolygon2);

      assertEquals(2, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion1 = constraintRegions.get(0);
      StepConstraintRegion returnedGroundRegion2 = constraintRegions.get(1);

      assertEquals(0, returnedGroundRegion1.getNumberOfHolesInRegion());
      assertEquals(0, returnedGroundRegion2.getNumberOfHolesInRegion());

      assertTrue(expectedGroundRegion2.epsilonEquals(returnedGroundRegion2, 1e-7));
      assertTrue(expectedGroundRegion1.epsilonEquals(returnedGroundRegion1, 1e-7));
   }

   @Test
   public void testEasyGroundWithABlockBelow()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, -0.2);

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(blockRegion);


      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      StepConstraintRegion expectedGroundRegion = new StepConstraintRegion(new RigidBodyTransform(), groundPolygon);

      assertEquals(1, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion = constraintRegions.get(0);
      assertTrue(expectedGroundRegion.epsilonEquals(returnedGroundRegion, 1e-7));
   }

   @Test
   public void testEasyGroundWithBlockThatCrops()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(1.1, 0.95);
      blockPolygon.addVertex(1.1, -0.95);
      blockPolygon.addVertex(0.5, 0.95);
      blockPolygon.addVertex(0.5, -0.95);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(blockRegion);


      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      ConcavePolygon2D obstacle =  SteppableRegionsCalculator.createObstacleExtrusion(groundRegion,
                                                                                          blockRegion,
                                                                                          extrusionDistanceCalculator,
                                                                                          Math.cos(orthogonalAngle));

      ConcavePolygon2D groundConcavePOlygon = new ConcavePolygon2D(groundPolygon);
      ConcavePolygon2DBasics croppedGroundPolygon = PolygonClippingAndMerging.removeAreaInsideClip(obstacle, groundConcavePOlygon).get(0);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      StepConstraintRegion expectedGroundRegion = new StepConstraintRegion(new RigidBodyTransform(), croppedGroundPolygon);
      StepConstraintRegion expectedBlockRegion = new StepConstraintRegion(blockTransform, blockPolygon);


      assertEquals(2, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion = constraintRegions.get(0);
      StepConstraintRegion returnedBlockRegion = constraintRegions.get(1);

      assertTrue(expectedGroundRegion.epsilonEquals(returnedGroundRegion, 1e-7));
      assertTrue(expectedBlockRegion.epsilonEquals(returnedBlockRegion, 1e-7));
   }

   @Test
   public void testEasyGroundWithBlockThatMergesWithAHole()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(1.1, 0.95);
      blockPolygon.addVertex(1.1, -0.95);
      blockPolygon.addVertex(0.5, 0.95);
      blockPolygon.addVertex(0.5, -0.95);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);

      ConvexPolygon2D blockPolygon2 = new ConvexPolygon2D();
      blockPolygon2.addVertex(0.0, 0.15);
      blockPolygon2.addVertex(0.0, -0.15);
      blockPolygon2.addVertex(0.45, 0.15);
      blockPolygon2.addVertex(0.45, -0.15);
      blockPolygon2.update();

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);
      PlanarRegion blockRegion2 = new PlanarRegion(blockTransform, blockPolygon2);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(blockRegion);
      listOfRegions.add(blockRegion2);


      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      ConcavePolygon2D obstacle1 = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));
      ConcavePolygon2D obstacle2 = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion2, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      ConcavePolygon2D obstacle = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(obstacle1, obstacle2, obstacle);

      ConcavePolygon2D groundConcavePOlygon = new ConcavePolygon2D(groundPolygon);
      ConcavePolygon2DBasics croppedGroundPolygon = PolygonClippingAndMerging.removeAreaInsideClip(obstacle, groundConcavePOlygon).get(0);

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      StepConstraintRegion expectedGroundRegion = new StepConstraintRegion(new RigidBodyTransform(), croppedGroundPolygon);
      StepConstraintRegion expectedBlockRegion = new StepConstraintRegion(blockTransform, blockPolygon);
      StepConstraintRegion expectedBlockRegion2 = new StepConstraintRegion(blockTransform, blockPolygon2);


      assertEquals(3, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion = constraintRegions.get(0);
      StepConstraintRegion returnedBlockRegion = constraintRegions.get(1);
      StepConstraintRegion returnedBlockRegion2 = constraintRegions.get(2);

      assertEquals(0, returnedBlockRegion.getNumberOfHolesInRegion());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(expectedGroundRegion.getConcaveHull(), returnedGroundRegion.getConcaveHull(), 1e-7);
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(expectedBlockRegion.getConcaveHull(), returnedBlockRegion.getConcaveHull(), 1e-7);

      assertTrue(expectedGroundRegion.epsilonEquals(returnedGroundRegion, 1e-7));
      assertTrue(expectedBlockRegion.epsilonEquals(returnedBlockRegion, 1e-7));
      assertTrue(expectedBlockRegion2.epsilonEquals(returnedBlockRegion2, 1e-7));
   }




   @Test
   public void testSplitThatTurnsHoleIntoAClip()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();
      RigidBodyTransformReadOnly groundTransform = new RigidBodyTransform();

      ConvexPolygon2D wallPolygon = new ConvexPolygon2D();
      wallPolygon.addVertex(1.0, 1.0);
      wallPolygon.addVertex(1.0, -1.0);
      wallPolygon.addVertex(-1.0, 1.0);
      wallPolygon.addVertex(-1.0, -1.0);
      wallPolygon.update();
      RigidBodyTransform wallTransform = new RigidBodyTransform();
      wallTransform.appendRollRotation(Math.toRadians(90));

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(1.1, 0.95);
      blockPolygon.addVertex(1.1, -0.95);
      blockPolygon.addVertex(0.5, 0.95);
      blockPolygon.addVertex(0.5, -0.95);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);


      PlanarRegion groundRegion = new PlanarRegion(groundTransform, groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);
      PlanarRegion wallRegion = new PlanarRegion(wallTransform, wallPolygon);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(wallRegion);
      listOfRegions.add(blockRegion);


      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      ConcavePolygon2D wallObstacle = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, wallRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));
      ConcavePolygon2D holeObstacle = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      List<ConcavePolygon2DBasics> groundConcavePolygons = PolygonClippingAndMerging.removeAreaInsideClip(wallObstacle, new ConcavePolygon2D(groundPolygon));
      ConcavePolygon2DBasics groundConcavePolygon1 = PolygonClippingAndMerging.removeAreaInsideClip(holeObstacle, groundConcavePolygons.get(0)).get(0);
      ConcavePolygon2DBasics groundConcavePolygon2 = PolygonClippingAndMerging.removeAreaInsideClip(holeObstacle, groundConcavePolygons.get(1)).get(0);

      List<ConcavePolygon2DBasics> blockConcavePolygons = PolygonClippingAndMerging.removeAreaInsideClip(wallObstacle, new ConcavePolygon2D(blockPolygon));

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();

      StepConstraintRegion expectedGroundRegion1 = new StepConstraintRegion(groundTransform, groundConcavePolygon1);
      StepConstraintRegion expectedGroundRegion2 = new StepConstraintRegion(groundTransform, groundConcavePolygon2);
      StepConstraintRegion expectedBlockRegion1 = new StepConstraintRegion(blockTransform, blockConcavePolygons.get(0));
      StepConstraintRegion expectedBlockRegion2 = new StepConstraintRegion(blockTransform, blockConcavePolygons.get(1));

      assertEquals(4, constraintRegions.size());

      StepConstraintRegion returnedGroundRegion1 = constraintRegions.get(0);
      StepConstraintRegion returnedGroundRegion2 = constraintRegions.get(1);
      StepConstraintRegion returnedBlockRegion1 = constraintRegions.get(2);
      StepConstraintRegion returnedBlockRegion2 = constraintRegions.get(3);

      assertTrue(expectedGroundRegion1.epsilonEquals(returnedGroundRegion1, 1e-7));
      assertTrue(expectedGroundRegion2.epsilonEquals(returnedGroundRegion2, 1e-7));
      assertTrue(expectedBlockRegion1.epsilonEquals(returnedBlockRegion1, 1e-7));
      assertTrue(expectedBlockRegion2.epsilonEquals(returnedBlockRegion2, 1e-7));
   }

   @Test
   public void testTwoWalls()
   {
      SteppableRegionsCalculator calculator = new SteppableRegionsCalculator(1.0, new YoVariableRegistry("test"));

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();
      RigidBodyTransformReadOnly groundTransform = new RigidBodyTransform();

      ConvexPolygon2D wallPolygon1 = new ConvexPolygon2D();
      wallPolygon1.addVertex(1.0, 1.0);
      wallPolygon1.addVertex(1.0, -1.0);
      wallPolygon1.addVertex(-1.0, 1.0);
      wallPolygon1.addVertex(-1.0, -1.0);
      wallPolygon1.update();
      RigidBodyTransform wallTransform1 = new RigidBodyTransform();
      wallTransform1.appendRollRotation(Math.toRadians(90));

      ConvexPolygon2D wallPolygon2 = new ConvexPolygon2D();
      wallPolygon2.addVertex(-1.0, 1.0);
      wallPolygon2.addVertex(0.0, 1.0);
      wallPolygon2.addVertex(0.0, -1.0);
      wallPolygon2.addVertex(-1.0, -1.0);
      wallPolygon2.update();
      RigidBodyTransform wallTransform2 = new RigidBodyTransform();
      wallTransform2.appendPitchRotation(Math.toRadians(90));


      PlanarRegion groundRegion = new PlanarRegion(groundTransform, groundPolygon);
      PlanarRegion wallRegion1 = new PlanarRegion(wallTransform1, wallPolygon1);
      PlanarRegion wallRegion2 = new PlanarRegion(wallTransform2, wallPolygon2);

      List<PlanarRegion> listOfRegions = new ArrayList<>();
      listOfRegions.add(groundRegion);
      listOfRegions.add(wallRegion1);
      listOfRegions.add(wallRegion2);


      double minimumDistanceFromCliffBottoms = 0.1;
      double canEasilyStepOverHeight = 0.1;
      double orthogonalAngle = Math.toRadians(75.0);

      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = getExtrusionCalculator(canEasilyStepOverHeight, minimumDistanceFromCliffBottoms);

      calculator.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms);
      calculator.setCanEasilyStepOverHeight(canEasilyStepOverHeight);
      calculator.setOrthogonalAngle(orthogonalAngle);
      calculator.setPlanarRegions(listOfRegions);

      ConcavePolygon2D wallObstacle1 = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, wallRegion1, extrusionDistanceCalculator, Math.cos(orthogonalAngle));
      ConcavePolygon2D wallObstacle2 = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, wallRegion2, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      List<ConcavePolygon2DBasics> groundConcavePolygons = PolygonClippingAndMerging.removeAreaInsideClip(wallObstacle2, new ConcavePolygon2D(groundPolygon));
      groundConcavePolygons = PolygonClippingAndMerging.removeAreaInsideClip(wallObstacle1, groundConcavePolygons.get(0));

      List<StepConstraintRegion> constraintRegions = calculator.computeSteppableRegions();


      assertEquals(3, constraintRegions.size());

      StepConstraintRegion expectedGroundRegion1 = new StepConstraintRegion(groundTransform, groundConcavePolygons.get(0));
      StepConstraintRegion expectedGroundRegion2 = new StepConstraintRegion(groundTransform, groundConcavePolygons.get(1));
      StepConstraintRegion expectedGroundRegion3 = new StepConstraintRegion(groundTransform, groundConcavePolygons.get(2));

      StepConstraintRegion returnedGroundRegion1 = constraintRegions.get(0);
      StepConstraintRegion returnedGroundRegion2 = constraintRegions.get(1);
      StepConstraintRegion returnedGroundRegion3 = constraintRegions.get(2);

      assertTrue(expectedGroundRegion1.epsilonEquals(returnedGroundRegion1, 1e-7));
      assertTrue(expectedGroundRegion2.epsilonEquals(returnedGroundRegion2, 1e-7));
      assertTrue(expectedGroundRegion3.epsilonEquals(returnedGroundRegion3, 1e-7));
   }



   @Test
   public void testCreateObstacleExtrusionForHorizontalObstacle()
   {
      double extrusionDistance1 = 0.1;
      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = (p, d) -> extrusionDistance1;

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);
      double orthogonalAngle = Math.toRadians(75.0);

      ConcavePolygon2D extrusion = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      double angleDistance = extrusionDistance1 * Math.sqrt(2) / 2.0;

      ConcavePolygon2D expectedExtrusion = new ConcavePolygon2D();
      expectedExtrusion.addVertex(-0.1 - extrusionDistance1, 0.1);
      expectedExtrusion.addVertex(-0.1 - angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(-0.1, 0.1 + extrusionDistance1);
      expectedExtrusion.addVertex(0.1, 0.1 + extrusionDistance1);
      expectedExtrusion.addVertex(0.1 + angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(0.1 + extrusionDistance1, 0.1);
      expectedExtrusion.addVertex(0.1 + extrusionDistance1, -0.1);
      expectedExtrusion.addVertex(0.1 + angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(0.1, -0.1 - extrusionDistance1);
      expectedExtrusion.addVertex(-0.1, -0.1 - extrusionDistance1);
      expectedExtrusion.addVertex(-0.1 - angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(-0.1 - extrusionDistance1, -0.1);
      expectedExtrusion.update();

      assertTrue(expectedExtrusion.epsilonEquals(extrusion, 1e-7));



      double extrusionDistance2 = 0.1;
      extrusionDistanceCalculator = (p, d) -> extrusionDistance2;

      groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      blockTransform = new RigidBodyTransform();
      blockTransform.appendTranslation(0, 0, 0.2);

      groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      extrusion = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      angleDistance = extrusionDistance2 * Math.sqrt(2) / 2.0;

      expectedExtrusion = new ConcavePolygon2D();
      expectedExtrusion.addVertex(-0.1 - extrusionDistance2, 0.1);
      expectedExtrusion.addVertex(-0.1 - angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(-0.1, 0.1 + extrusionDistance2);
      expectedExtrusion.addVertex(0.1, 0.1 + extrusionDistance2);
      expectedExtrusion.addVertex(0.1 + angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(0.1 + extrusionDistance2, 0.1);
      expectedExtrusion.addVertex(0.1 + extrusionDistance2, -0.1);
      expectedExtrusion.addVertex(0.1 + angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(0.1, -0.1 - extrusionDistance2);
      expectedExtrusion.addVertex(-0.1, -0.1 - extrusionDistance2);
      expectedExtrusion.addVertex(-0.1 - angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(-0.1 - extrusionDistance2, -0.1);
      expectedExtrusion.update();

      assertTrue(expectedExtrusion.epsilonEquals(extrusion, 1e-7));
   }

   @Test
   public void testCreateObstacleExtrusionForVerticalObstacle()
   {
      double extrusionDistance1 = 0.1;
      ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator = (p, d) -> extrusionDistance1;

      ConvexPolygon2D groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      ConvexPolygon2D blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      RigidBodyTransform blockTransform = new RigidBodyTransform();
      blockTransform.appendPitchRotation(Math.toRadians(90.0));

      PlanarRegion groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      PlanarRegion blockRegion = new PlanarRegion(blockTransform, blockPolygon);
      double orthogonalAngle = Math.toRadians(75.0);

      ConcavePolygon2D extrusion = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      double angleDistance = extrusionDistance1 * Math.sqrt(2) / 2.0;

      ConcavePolygon2D expectedExtrusion = new ConcavePolygon2D();
      expectedExtrusion.addVertex(-extrusionDistance1, 0.1);
      expectedExtrusion.addVertex(-angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(0.0, 0.1 + extrusionDistance1);
      expectedExtrusion.addVertex(angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(extrusionDistance1, 0.1);
      expectedExtrusion.addVertex(extrusionDistance1, -0.1);
      expectedExtrusion.addVertex(angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(0.0, -0.1 - extrusionDistance1);
      expectedExtrusion.addVertex(-angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(-extrusionDistance1, -0.1);
      expectedExtrusion.update();

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(expectedExtrusion, extrusion, 1e-7);

      double extrusionDistance2 = 0.15;
      extrusionDistanceCalculator = (p, d) -> extrusionDistance2;

      groundPolygon = new ConvexPolygon2D();
      groundPolygon.addVertex(1.0, 1.0);
      groundPolygon.addVertex(1.0, -1.0);
      groundPolygon.addVertex(-1.0, 1.0);
      groundPolygon.addVertex(-1.0, -1.0);
      groundPolygon.update();

      blockPolygon = new ConvexPolygon2D();
      blockPolygon.addVertex(0.1, 0.1);
      blockPolygon.addVertex(0.1, -0.1);
      blockPolygon.addVertex(-0.1, 0.1);
      blockPolygon.addVertex(-0.1, -0.1);
      blockPolygon.update();
      blockTransform = new RigidBodyTransform();
      blockTransform.appendPitchRotation(Math.toRadians(90.0));

      groundRegion = new PlanarRegion(new RigidBodyTransform(), groundPolygon);
      blockRegion = new PlanarRegion(blockTransform, blockPolygon);

      extrusion = SteppableRegionsCalculator.createObstacleExtrusion(groundRegion, blockRegion, extrusionDistanceCalculator, Math.cos(orthogonalAngle));

      angleDistance = extrusionDistance2 * Math.sqrt(2) / 2.0;

      expectedExtrusion = new ConcavePolygon2D();
      expectedExtrusion.addVertex(-extrusionDistance2, 0.1);
      expectedExtrusion.addVertex(-angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(0.0, 0.1 + extrusionDistance2);
      expectedExtrusion.addVertex(angleDistance, 0.1 + angleDistance);
      expectedExtrusion.addVertex(extrusionDistance2, 0.1);
      expectedExtrusion.addVertex(extrusionDistance2, -0.1);
      expectedExtrusion.addVertex(angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(0.0, -0.1 - extrusionDistance2);
      expectedExtrusion.addVertex(-angleDistance, -0.1 - angleDistance);
      expectedExtrusion.addVertex(-extrusionDistance2, -0.1);
      expectedExtrusion.update();

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(expectedExtrusion, extrusion, 1e-7);
   }

   private static ObstacleExtrusionDistanceCalculator getExtrusionCalculator(double canEasilyStepOverHeight, double minimumDistanceFromCliffBottoms)
   {
      return new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            if (obstacleHeight < 0.0)
            {
               return 0.0;
            }
            else if (obstacleHeight < canEasilyStepOverHeight)
            {
               double alpha = obstacleHeight / canEasilyStepOverHeight;
               return InterpolationTools.linearInterpolate(0.0, minimumDistanceFromCliffBottoms, alpha);
            }
            else
            {
               return minimumDistanceFromCliffBottoms;
            }
         }
      };
   }
}
