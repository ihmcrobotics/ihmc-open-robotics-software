package us.ihmc.simulationConstructionSetTools.util.ground;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject.DecompositionParameters;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MeshTerrainObjectTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final boolean SHOW_VISUALIZATION = true;
   private static SimulationConstructionSet scs = null;

   @AfterEach
   public void tearDown()
   {
      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   public void testWithVisualization()
   {
      if (SHOW_VISUALIZATION)
      {

         RigidBodyTransform configuration = new RigidBodyTransform();
         configuration.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, -Math.PI / 2.0));
         
         
         
         
         MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/SmallWalkway/Walkway.obj", configuration);
         DecompositionParameters params = meshTerrainObject.new DecompositionParameters(0, 0);
         
         scs = new SimulationConstructionSet(new Robot("dummy"));
         scs.addStaticLinkGraphics(meshTerrainObject.getLinkGraphics());

         // Red is the point being tested - it will be a cube if heightAt is negative
         // infinity, otherwise it will be a sphere
         // Yellow ball is heightAt value if it exists
         Graphics3DObject viz = new Graphics3DObject();

         // Adding testing points
         double xStart = -1.5;
         double xEnd = 1.5;
         double yStart = -0.75;
         double yEnd = 1.75;
         double increment = 0.3;

         for (double i = xStart; i < xEnd; i = i + increment)
         {
            for (double j = yStart; j < yEnd; j = j + increment)
            {
               addGraphicWithTestPoints(viz, meshTerrainObject, new Point3D(i, j, 2.0), YoAppearance.DarkSalmon());
            }
         }

         scs.addStaticLinkGraphics(viz);
         scs.setGroundVisible(false);
         scs.startOnAThread();
         ThreadTools.sleepForever();

      }
   }

   @Test
   public void testHeightAt()
   {

      // Testing using a Stool with bounding box dimensions as X,Y,Z = 0.375, 0.375,0.575
      // Stool dimensions can be found here "https://cad.onshape.com/documents/ed770d6e6aa0b5ca25885953/w/d5578c1a76bb6f06ae50c7b9/e/ebb25271027fb47ebaec1c9c"

      String relativeFilePath = "models/Stool/Stool.obj";
      MeshTerrainObject meshTerrainObject = new MeshTerrainObject(relativeFilePath);

      Double heightAt;
      Double maxError = 0.001;
      // Point A is above the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will intersect the stool
      Point3D pointA = new Point3D(0.25, 0.25, 0.6);
      heightAt = meshTerrainObject.heightAt(pointA.getX(), pointA.getY(), pointA.getZ());
      assertEquals(heightAt, 0.575, maxError);

      // Point B is under the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still intersect the stool
      Point3D pointB = new Point3D(0.18, 0.18, 0.25);
      heightAt = meshTerrainObject.heightAt(pointB.getX(), pointB.getY(), pointB.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point C is inside the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still intersect the stool
      Point3D pointC = new Point3D(0.18, 0.18, 0.5);
      heightAt = meshTerrainObject.heightAt(pointC.getX(), pointC.getY(), pointC.getZ());
      assertEquals(heightAt, 0.575, maxError);

      // Point D is above the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still NOT INTERSECT the stool
      Point3D pointD = new Point3D(0.5, 0.5, 0.75);
      heightAt = meshTerrainObject.heightAt(pointD.getX(), pointD.getY(), pointD.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point E is below the stool. If an infinite line is drawn parallel to the Z axis of this point, the line will still NOT INTERSECT the stool
      Point3D pointE = new Point3D(-0.25, -0.25, -0.25);
      heightAt = meshTerrainObject.heightAt(pointE.getX(), pointE.getY(), pointE.getZ());
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);

      // Point F is on the top surface of the stool. If an infinite line is drawn and parallel to the Z axis of this point, the line will INTERSECT the stool
      Point3D pointF = new Point3D(0.18, 0.18, 0.575);
      heightAt = meshTerrainObject.heightAt(pointF.getX(), pointF.getY(), pointF.getZ());
      assertEquals(heightAt, 0.575, maxError);

      Graphics3DObject viz = new Graphics3DObject();
      scs = new SimulationConstructionSet(new Robot("dummy"));
      scs.addStaticLinkGraphics(meshTerrainObject.getLinkGraphics());

      scs.setGroundVisible(false);

      addGraphicWithTestPoints(viz, meshTerrainObject, pointA, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointB, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointC, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointD, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointE, YoAppearance.DarkSalmon());
      addGraphicWithTestPoints(viz, meshTerrainObject, pointF, YoAppearance.DarkSalmon());

      // Red is the point being tested - it will be a cube if heightAt is negative
      // infinity, otherwise it will be a sphere
      // Yellow ball is heightAt value if it exists

      scs.addStaticLinkGraphics(viz);
      scs.startOnAThread();
      ThreadTools.sleepForever();

   }

   @Test
   public void testCheckIfInside()
   {
      Random random = new Random(148547);

      Vector3D vectorNormal = new Vector3D(0.0, 0.0, 1.0);
      Point3D intersectionToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);

         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);
         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(normalToPack, convexPolytope);

         // check point inside the convexPolytope
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertTrue(convexPolytopeTerraian.checkIfInside(x, y, z, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(convexPolytopeTerraian.heightAt(x, y, z), intersectionToPack.getZ());
         assertEquals(convexPolytope.getClosestFace(intersectionToPack).getNormal(), normalToPack);

         // check point above the convexPolytope
         Double zAboveConvexPolytope = z + convexPolytopeTerraian.getBoundingBox().getMaxZ() + 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(x, y, zAboveConvexPolytope, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         // Since only the z value was changed to a point above the shape, it should
         // equal the same heightAt as the point inside the shape
         assertEquals(convexPolytopeTerraian.heightAt(x, y, z), intersectionToPack.getZ(), EPSILON);
         assertEquals(convexPolytope.getClosestFace(intersectionToPack).getNormal(), normalToPack);

         // check point below the convexPolytope
         Double zBelowConvexPolytope = z + convexPolytopeTerraian.getBoundingBox().getMinZ() - 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(x, y, zBelowConvexPolytope, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(Double.NEGATIVE_INFINITY, intersectionToPack.getZ());
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);

         // check point on side of the convexPolytope
         Double xOutsideConvexPolytope = x + convexPolytopeTerraian.getBoundingBox().getMaxX() + 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(xOutsideConvexPolytope, y, z, intersectionToPack, normalToPack));
         assertEquals(xOutsideConvexPolytope, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(Double.NEGATIVE_INFINITY, intersectionToPack.getZ());
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);

         // verify checkIfInside can be called with intersectionToPack and/or
         // NormalToPack are null
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertTrue(convexPolytopeTerraian.checkIfInside(x, y, z, null, normalToPack));
         intersectionToPack.set(x, y, convexPolytopeTerraian.heightAt(x, y, z));
         assertEquals(convexPolytope.getClosestFace(intersectionToPack).getNormal(), normalToPack);

         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertTrue(convexPolytopeTerraian.checkIfInside(x, y, z, intersectionToPack, null));
         intersectionToPack.set(x, y, convexPolytopeTerraian.heightAt(x, y, z));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(convexPolytopeTerraian.heightAt(x, y, z), intersectionToPack.getZ());

         assertTrue(convexPolytopeTerraian.checkIfInside(x, y, z, null, null));
      }
   }

   @Test
   public void testHeightAndNormalAt()
   {
      Random random = new Random(52141);

      Vector3D vectorNormal = new Vector3D(0.0, 0.0, 1.0);

      Vector3D normalToPack = new Vector3D();

      MeshTerrainObject meshTerrainObject = new MeshTerrainObject("models/SmallWalkway/Walkway.obj");

      for (int i = 0; i < ITERATIONS; i++)
      {
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);

         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);
         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(normalToPack, convexPolytope);

         // check point inside the convexPolytope
         normalToPack.set(0, 0, 0);
         Double heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, z, normalToPack);
         assertEquals(heightAt, convexPolytopeTerraian.heightAt(x, y, z));
         assertEquals(convexPolytope.getClosestFace(new Point3D(x, y, heightAt)).getNormal(), normalToPack);

         // check point above the convexPolytope
         Double zAboveConvexPolytope = z + convexPolytopeTerraian.getBoundingBox().getMaxZ() + 1.0;
         normalToPack.set(0, 0, 0);
         heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, zAboveConvexPolytope, normalToPack);
         assertNotEquals(heightAt, Double.NEGATIVE_INFINITY);
         // Since only z was changed above the shape, it should equal the same heightAt
         // as the point inside the shape
         assertEquals(heightAt, convexPolytopeTerraian.heightAt(x, y, z), EPSILON);
         assertEquals(convexPolytope.getClosestFace(new Point3D(x, y, heightAt)).getNormal(), normalToPack);

         // check point below the convexPolytope
         Double zBelowConvexPolytope = z + convexPolytopeTerraian.getBoundingBox().getMinZ() - 1.0;
         normalToPack.set(0, 0, 0);
         heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, zBelowConvexPolytope, normalToPack);
         assertEquals(heightAt, Double.NEGATIVE_INFINITY);
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);

         // check point on side of the convexPolytope
         Double xOutsideConvexPolytope = x + convexPolytopeTerraian.getBoundingBox().getMaxX() + 1.0;
         normalToPack.set(0, 0, 0);
         heightAt = convexPolytopeTerraian.heightAndNormalAt(xOutsideConvexPolytope, y, z, normalToPack);
         assertEquals(heightAt, Double.NEGATIVE_INFINITY);
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);
      }
   }

   @Test
   public void testIsClose()
   {
      Random random = new Random(22512);
      Vector3D vectorNormal = new Vector3D(0.0, 0.0, 1.0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         // Generate a random ConvexPolytope
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);

         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);

         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(vectorNormal, convexPolytope);

         // Verify a point inside the convexPolytopeTerraian's bounding box returns true
         assertTrue(convexPolytopeTerraian.isClose(x, y, z));

         // Verify a point on the the bounding box returns true
         assertTrue(convexPolytopeTerraian.isClose(convexPolytope.getBoundingBox().getMaxX(),
                                                   convexPolytope.getBoundingBox().getMaxY(),
                                                   convexPolytope.getBoundingBox().getMaxZ()));

         // Verify a point on the outside of the ConvexPolytope's bounding box returns
         // false
         assertFalse(convexPolytopeTerraian.isClose(x, y, convexPolytope.getBoundingBox().getMaxZ() + 1.0));
         assertFalse(convexPolytopeTerraian.isClose(x, y, convexPolytope.getBoundingBox().getMinZ() - 1.0));
         assertFalse(convexPolytopeTerraian.isClose(convexPolytope.getBoundingBox().getMaxX() + 1.0, y, z));
         assertFalse(convexPolytopeTerraian.isClose(convexPolytope.getBoundingBox().getMinX() - 1.0, y, z));
         assertFalse(convexPolytopeTerraian.isClose(x, convexPolytope.getBoundingBox().getMaxY() + 1.0, z));
         assertFalse(convexPolytopeTerraian.isClose(x, convexPolytope.getBoundingBox().getMinY() - 1.0, z));
      }
   }

   private static Boolean isPointInsideOneOfTheFaces(ConvexPolytope3D convexPolytope, Double x, Double y, Double z)
   {
      Point3D point = new Point3D(x, y, z);
      for (Face3DReadOnly face : convexPolytope.getFaces())
      {
         if (face.isPointInside(point, EPSILON))
            return true;
      }

      return false;
   }

   public static void addArrowForNormal(Double xPoint, Double yPoint, Double heightAt, Vector3D normal)
   {

      YoRegistry robotsYoVariableRegistry = new YoRegistry("surfaceNormals");
      YoFramePoint3D planarRegionPointInWorld = new YoFramePoint3D("arrow", ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
      Point3D translation = new Point3D(xPoint, yPoint, heightAt);
      planarRegionPointInWorld.set(translation);

      YoFrameVector3D surfaceNormal = new YoFrameVector3D("NormalVector", ReferenceFrame.getWorldFrame(), robotsYoVariableRegistry);
      surfaceNormal.set(normal);

      YoGraphicVector surfaceNormalGraphic = new YoGraphicVector("PlanarRegionSurfaceNormalGraphic",
                                                                 planarRegionPointInWorld,
                                                                 surfaceNormal,
                                                                 YoAppearance.Aqua());
      scs.addYoGraphic(surfaceNormalGraphic);

   }

   private static void addGraphicWithTestPoints(Graphics3DObject viz,
                                                MeshTerrainObject convexPolytopeTerrainObject,
                                                Point3D point,
                                                AppearanceDefinition lineAppearance)
   {

      Double xPoint = point.getX();
      Double yPoint = point.getY();
      Double zPoint = point.getZ();

      Vector3D normalToPack = new Vector3D();
      normalToPack.set(0.0, 0.0, 0.0);
      double result = convexPolytopeTerrainObject.heightAndNormalAt(xPoint, yPoint, zPoint, normalToPack);

      if (result != Double.NEGATIVE_INFINITY)
      {
         addArrowForNormal(xPoint, yPoint, result, normalToPack);

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addSphere(0.04, YoAppearance.Red());

         viz.identity();
         viz.translate(xPoint, yPoint, result);
         viz.addSphere(0.04, YoAppearance.Yellow());
      }
      else
      {

         viz.identity();
         viz.translate(xPoint, yPoint, zPoint);
         viz.addCube(0.08, 0.08, 0.08, YoAppearance.Red());
      }
      viz.identity();
      viz.translate(xPoint, yPoint, -500.0);
      viz.addCylinder(1000.0, 0.008, lineAppearance);

   }

}