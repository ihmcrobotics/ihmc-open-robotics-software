package us.ihmc.simulationConstructionSetTools.util.ground;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ConvexPolytopeTerrainObjectTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final boolean SHOW_VISUALIZATION = false;

   private SimulationConstructionSet scs = null;

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
         ConvexPolytopeTerrainObject convexPolytopeTerrainObject = new ConvexPolytopeTerrainObject(EuclidPolytopeFactories.newIcosahedron(1.0),
                                                                                                   YoAppearance.AliceBlue());
         scs = new SimulationConstructionSet(new Robot("dummy"));
         scs.addStaticLinkGraphics(convexPolytopeTerrainObject.getLinkGraphics());

         //Red is the point being test - it will be a circle if heightAt is not negative infinity 
         //Yellow ball is heightAt
         Graphics3DObject viz = new Graphics3DObject();
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.0, 0.0, 2.0, YoAppearance.DarkSalmon());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, -0.3, 0.0, 1.2, YoAppearance.CadetBlue());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.4, -0.4, -0.2, YoAppearance.GreenYellow());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.1, 0.1, -2.0, YoAppearance.Green());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 1.0, 1.0, 1.0, YoAppearance.Coral());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, -1.0, -1.0, 0.5, YoAppearance.Gold());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.5, 0.5, -2.0, YoAppearance.Black());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.5, -0.5, 1.0, YoAppearance.Azure());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, -0.5, -0.5, -0.25, YoAppearance.DarkViolet());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.5, 0.25, 0.5, YoAppearance.Chartreuse());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.85065080835204, 0.0, 1.0, YoAppearance.DarkSalmon());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.5257311121191336, -0.85065080835204, 1.0, YoAppearance.Purple());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, -0.5257311121191336, 0.85065080835204, 0.25, YoAppearance.Red());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.0, 0.5257311121191336, 1.0, YoAppearance.Black());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, 0.0, 0.85065080835204, 0.25, YoAppearance.Gray());
         addGraphicWithTestPoints(viz, convexPolytopeTerrainObject, -0.85065080835204, 0.0, -0.25, YoAppearance.Green());

         scs.addStaticLinkGraphics(viz);
         scs.setGroundVisible(false);
         scs.startOnAThread();

         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testHeightAt()
   {
      Random random = new Random(83432);
      Vector3D vectorNormal = new Vector3D(0.0, 0.0, 1.0);

      //For a box, verify that a point along one of the edges of the box returns the top edge of the box. If it under the box, it should return
      //negative infinity
      ConvexPolytopeTerrainObject convexPolytopeTerrainObject = new ConvexPolytopeTerrainObject(EuclidPolytopeFactories.newCube(1.0));
      Double heightAt = convexPolytopeTerrainObject.heightAt(0.5, 0.5, -5.0);
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);
      heightAt = convexPolytopeTerrainObject.heightAt(0.5, -0.5, 1.0);
      assertEquals(heightAt, 0.5);
      heightAt = convexPolytopeTerrainObject.heightAt(-0.5, 0.5, 0.5);
      assertEquals(heightAt, 0.5);
      heightAt = convexPolytopeTerrainObject.heightAt(-0.5, 0.2, -1.0);
      assertEquals(heightAt, Double.NEGATIVE_INFINITY);
      heightAt = convexPolytopeTerrainObject.heightAt(-0.5, -0.5, -0.5);
      assertEquals(heightAt, 0.5);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);

         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);

         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(convexPolytope);

         //Find heightAt for a point inside the convexPolytope
         Double heightAtPointInsideConvexPolytope = convexPolytopeTerraian.heightAt(x, y, z);

         //Verify heightAt returned a value and the point x, y, z = heightAtPointInsidePolytope is on one of the faces of the ConvexPolytope
         assertNotEquals(heightAtPointInsideConvexPolytope, 0.0);
         assertTrue(isPointInsideOneOfTheFaces(convexPolytope, x, y, heightAtPointInsideConvexPolytope));

         //Next get heightAt for a point above the ConvexPolytope
         Double heightAtPointAboveConvexPolytope = convexPolytopeTerraian.heightAt(x, y, convexPolytopeTerraian.getBoundingBox().getMaxZ() + 1.0);

         //Verify the point x, y, z = heightAtPointInsidePolytope is on one of the faces and verify the heightAt of the point inside 
         //the convexPolytope is less than the heightAt of the point above the convexPolytope.
         assertTrue(isPointInsideOneOfTheFaces(convexPolytope, x, y, heightAtPointAboveConvexPolytope));
         assertEquals(heightAtPointInsideConvexPolytope, heightAtPointAboveConvexPolytope, EPSILON);

         //Verify point on one of the faces of the convexPolytope returns the Z-Axis of the point
         Point3D pointOnFace = new Point3D(x, y, heightAtPointAboveConvexPolytope);
         Double heightAtPointOnFaceOfConvexPolytope = convexPolytopeTerraian.heightAt(pointOnFace.getX(), pointOnFace.getY(), pointOnFace.getZ());
         assertEquals(heightAtPointOnFaceOfConvexPolytope, pointOnFace.getZ(), EPSILON);

         //Verify a point under the ConvexPolytope returns negative infinity
         Double heightAtPointUnderConvexPolytope = convexPolytopeTerraian.heightAt(x, y, convexPolytopeTerraian.getBoundingBox().getMinZ() - 1.0);
         assertEquals(heightAtPointUnderConvexPolytope, Double.NEGATIVE_INFINITY);

         //Verify points on outside of each side of ConvexPolytope returns negative infinity
         Double heightAtPointOutsideOfConvexPolytope = convexPolytopeTerraian.heightAt(x, convexPolytopeTerraian.getBoundingBox().getMaxY() + 1.0, z);
         assertEquals(heightAtPointOutsideOfConvexPolytope, Double.NEGATIVE_INFINITY);

         heightAtPointOutsideOfConvexPolytope = convexPolytopeTerraian.heightAt(x, convexPolytopeTerraian.getBoundingBox().getMinY() - 1.0, z);
         assertEquals(heightAtPointOutsideOfConvexPolytope, Double.NEGATIVE_INFINITY);

         heightAtPointOutsideOfConvexPolytope = convexPolytopeTerraian.heightAt(convexPolytopeTerraian.getBoundingBox().getMaxX() + 1.0, y, z);
         assertEquals(heightAtPointOutsideOfConvexPolytope, Double.NEGATIVE_INFINITY);

         heightAtPointOutsideOfConvexPolytope = convexPolytopeTerraian.heightAt(convexPolytopeTerraian.getBoundingBox().getMinX() - 1.0, y, z);
         assertEquals(heightAtPointOutsideOfConvexPolytope, Double.NEGATIVE_INFINITY);
      }
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
         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(convexPolytope);
         
         //check point inside the convexPolytope
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertTrue(convexPolytopeTerraian.checkIfInside(x, y, z, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(convexPolytopeTerraian.heightAt(x, y, z), intersectionToPack.getZ());
         assertEquals(convexPolytope.getClosestFace(intersectionToPack).getNormal(), normalToPack);
         
         //check point above the convexPolytope
         Double zAboveConvexPolytope = z +  convexPolytopeTerraian.getBoundingBox().getMaxZ() + 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(x, y, zAboveConvexPolytope, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         //Since only the z value was changed to a point above the shape, it should equal the same heightAt as the point inside the shape
         assertEquals(convexPolytopeTerraian.heightAt(x, y, z), intersectionToPack.getZ(), EPSILON);
         assertEquals(convexPolytope.getClosestFace(intersectionToPack).getNormal(), normalToPack);
         
         //check point below the convexPolytope
         Double zBelowConvexPolytope = z +  convexPolytopeTerraian.getBoundingBox().getMinZ() - 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(x, y, zBelowConvexPolytope, intersectionToPack, normalToPack));
         assertEquals(x, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(Double.NEGATIVE_INFINITY, intersectionToPack.getZ());
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);
         
         //check point on side of the convexPolytope
         Double xOutsideConvexPolytope = x +  convexPolytopeTerraian.getBoundingBox().getMaxX() + 1.0;
         intersectionToPack.set(0, 0, 0);
         normalToPack.set(0, 0, 0);
         assertFalse(convexPolytopeTerraian.checkIfInside(xOutsideConvexPolytope, y, z, intersectionToPack, normalToPack));
         assertEquals(xOutsideConvexPolytope, intersectionToPack.getX());
         assertEquals(y, intersectionToPack.getY());
         assertEquals(Double.NEGATIVE_INFINITY, intersectionToPack.getZ());
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);
         
         //verify checkIfInside can be called with intersectionToPack and/or NormalToPack are null
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
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);
   
         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);
         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(convexPolytope);
         
         //check point inside the convexPolytope
         normalToPack.set(0, 0, 0);
         Double heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, z, normalToPack);
         assertEquals(heightAt, convexPolytopeTerraian.heightAt(x, y, z));
         assertEquals(convexPolytope.getClosestFace(new Point3D(x, y, heightAt)).getNormal(), normalToPack);
         
         //check point above the convexPolytope
         Double zAboveConvexPolytope = z +  convexPolytopeTerraian.getBoundingBox().getMaxZ() + 1.0;
         normalToPack.set(0, 0, 0);
         heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, zAboveConvexPolytope, normalToPack);
         assertNotEquals(heightAt, Double.NEGATIVE_INFINITY);
         //Since only z was changed above the shape, it should equal the same heightAt as the point inside the shape
         assertEquals(heightAt, convexPolytopeTerraian.heightAt(x, y, z), EPSILON);
         assertEquals(convexPolytope.getClosestFace(new Point3D(x, y, heightAt)).getNormal(), normalToPack);
         
         //check point below the convexPolytope
         Double zBelowConvexPolytope = z +  convexPolytopeTerraian.getBoundingBox().getMinZ() - 1.0;
         normalToPack.set(0, 0, 0);
         heightAt = convexPolytopeTerraian.heightAndNormalAt(x, y, zBelowConvexPolytope, normalToPack);
         assertEquals(heightAt, Double.NEGATIVE_INFINITY);
         assertEquals(normalToPack.getX(), 0.0);
         assertEquals(normalToPack.getY(), 0.0);
         assertEquals(normalToPack.getZ(), 0.0);
         
         //check point on side of the convexPolytope
         Double xOutsideConvexPolytope = x +  convexPolytopeTerraian.getBoundingBox().getMaxX() + 1.0;
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
         //Generate a random ConvexPolytope
         Double x = random.nextDouble();
         Double y = random.nextDouble();
         Double z = random.nextDouble();
         Point3D point = new Point3D(x, y, z);

         ConvexPolytope3D convexPolytope = EuclidShapeRandomTools.nextTetrahedronContainingPoint3D(random, point);

         ConvexPolytopeTerrainObject convexPolytopeTerraian = new ConvexPolytopeTerrainObject(convexPolytope);

         //Verify a point inside the convexPolytopeTerraian's bounding box returns true
         assertTrue(convexPolytopeTerraian.isClose(x, y, z));

         //Verify a point on the the bounding box returns true
         assertTrue(convexPolytopeTerraian.isClose(convexPolytope.getBoundingBox().getMaxX(),
                                                   convexPolytope.getBoundingBox().getMaxY(),
                                                   convexPolytope.getBoundingBox().getMaxZ()));

         //Verify a point on the outside of the ConvexPolytope's bounding box returns false
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

   private static void addGraphicWithTestPoints(Graphics3DObject viz,
                                                ConvexPolytopeTerrainObject convexPolytopeTerrainObject,
                                                Double xPoint,
                                                Double yPoint,
                                                Double zPoint,
                                                AppearanceDefinition lineAppearance)
   {

      double result = convexPolytopeTerrainObject.heightAt(xPoint, yPoint, zPoint);

      if (result != Double.NEGATIVE_INFINITY)
      {
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
