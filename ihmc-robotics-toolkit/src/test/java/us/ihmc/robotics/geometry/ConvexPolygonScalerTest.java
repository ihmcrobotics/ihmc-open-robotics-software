package us.ihmc.robotics.geometry;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;

import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ConvexPolygonScalerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testScaleSquareExteriorPolygonToContainSquareInteriorPolygon()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(1.0, 1.0);
      Point2D exteriorVertex1 = new Point2D(-1.0, 1.0);
      Point2D exteriorVertex2 = new Point2D(1.0, -1.0);
      Point2D exteriorVertex3 = new Point2D(-1.0, -1.0);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.addVertex(exteriorVertex2);
      exteriorPolygon.addVertex(exteriorVertex3);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.25, 0.25);
      Point2D interiorVertex1 = new Point2D(-0.25, 0.25);
      Point2D interiorVertex2 = new Point2D(0.25, -0.25);
      Point2D interiorVertex3 = new Point2D(-0.25, -0.25);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(0.75, 0.75);
      Point2D expectedVertex1 = new Point2D(-0.75, 0.75);
      Point2D expectedVertex2 = new Point2D(0.75, -0.75);
      Point2D expectedVertex3 = new Point2D(-0.75, -0.75);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);


      // shrink a little more inside
      scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);

      expectedVertex0 = new Point2D(0.55, 0.55);
      expectedVertex1 = new Point2D(-0.55, 0.55);
      expectedVertex2 = new Point2D(0.55, -0.55);
      expectedVertex3 = new Point2D(-0.55, -0.55);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      // expand a little more outside
      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.2, scaledPolygon);

      expectedVertex0 = new Point2D(0.95, 0.95);
      expectedVertex1 = new Point2D(-0.95, 0.95);
      expectedVertex2 = new Point2D(0.95, -0.95);
      expectedVertex3 = new Point2D(-0.95, -0.95);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);

      // shrink to a point
      interiorVertex0 = new Point2D(1.1, 1.1);
      interiorVertex1 = new Point2D(-1.1, 1.1);
      interiorVertex2 = new Point2D(1.1, -1.1);
      interiorVertex3 = new Point2D(-1.1, -1.1);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);

      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(new Point2D(0, 0));
      scaledPolygonExpected.update();

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);




      // move to a different origin


      exteriorVertex0 = new Point2D(2.5, 2.5);
      exteriorVertex1 = new Point2D(2.5, 0.5);
      exteriorVertex2 = new Point2D(0.5, 0.5);
      exteriorVertex3 = new Point2D(0.5, 2.5);
      exteriorPolygon.clear();
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.addVertex(exteriorVertex2);
      exteriorPolygon.addVertex(exteriorVertex3);
      exteriorPolygon.update();

      interiorVertex0 = new Point2D(0.25, 0.25);
      interiorVertex1 = new Point2D(-0.25, 0.25);
      interiorVertex2 = new Point2D(0.25, -0.25);
      interiorVertex3 = new Point2D(-0.25, -0.25);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      expectedVertex0 = new Point2D(2.25, 2.25);
      expectedVertex1 = new Point2D(2.25, 0.75);
      expectedVertex2 = new Point2D(0.75, 0.75);
      expectedVertex3 = new Point2D(0.75, 2.25);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);


      // shrink a little more inside
      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);

      expectedVertex0 = new Point2D(2.05, 2.05);
      expectedVertex1 = new Point2D(2.05, 0.95);
      expectedVertex2 = new Point2D(0.95, 0.95);
      expectedVertex3 = new Point2D(0.95, 2.05);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);

      // expand a little more outside
      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.2, scaledPolygon);

      expectedVertex0 = new Point2D(2.45, 2.45);
      expectedVertex1 = new Point2D(2.45, 0.55);
      expectedVertex2 = new Point2D(0.55, 0.55);
      expectedVertex3 = new Point2D(0.55, 2.45);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);

      // shrink to a point
      interiorVertex0 = new Point2D(1.1, 1.1);
      interiorVertex1 = new Point2D(-1.1, 1.1);
      interiorVertex2 = new Point2D(1.1, -1.1);
      interiorVertex3 = new Point2D(-1.1, -1.1);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);

      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(exteriorPolygon.getCentroid());
      scaledPolygonExpected.update();

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testScaleSquareExteriorPolygonToContainQuadrulateralInteriorPolygon()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(1.0, 1.0);
      Point2D exteriorVertex1 = new Point2D(-1.0, 1.0);
      Point2D exteriorVertex2 = new Point2D(1.0, -1.0);
      Point2D exteriorVertex3 = new Point2D(-1.0, -1.0);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.addVertex(exteriorVertex2);
      exteriorPolygon.addVertex(exteriorVertex3);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.25, 0.25);
      Point2D interiorVertex1 = new Point2D(-0.5, 0.5);
      Point2D interiorVertex2 = new Point2D(0.25, -0.25);
      Point2D interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(0.75, 0.5);
      Point2D expectedVertex1 = new Point2D(-0.5, 0.5);
      Point2D expectedVertex2 = new Point2D(0.75, -0.5);
      Point2D expectedVertex3 = new Point2D(-0.5, -0.5);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);

      expectedVertex0 = new Point2D(0.55, 0.3);
      expectedVertex1 = new Point2D(-0.3, 0.3);
      expectedVertex2 = new Point2D(0.55, -0.3);
      expectedVertex3 = new Point2D(-0.3, -0.3);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testScaleHexagonExteriorPolygonToContainSquareInteriorPolygon()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(1.0, 1.0);
      Point2D exteriorVertex1 = new Point2D(-1.0, 1.0);
      Point2D exteriorVertex2 = new Point2D(1.0, -1.0);
      Point2D exteriorVertex3 = new Point2D(-1.0, -1.0);
      Point2D exteriorVertex4 = new Point2D(0.0, 2.0);
      Point2D exteriorVertex5 = new Point2D(0.0, -2.0);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.addVertex(exteriorVertex2);
      exteriorPolygon.addVertex(exteriorVertex3);
      exteriorPolygon.addVertex(exteriorVertex4);
      exteriorPolygon.addVertex(exteriorVertex5);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.5, 0.5);
      Point2D interiorVertex1 = new Point2D(-0.5, 0.5);
      Point2D interiorVertex2 = new Point2D(0.5, -0.5);
      Point2D interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(0.5, 0.5);
      Point2D expectedVertex1 = new Point2D(-0.5, 0.5);
      Point2D expectedVertex2 = new Point2D(0.5, -0.5);
      Point2D expectedVertex3 = new Point2D(-0.5, -0.5);
      Point2D expectedVertex4 = new Point2D(0.0, 1.0);
      Point2D expectedVertex5 = new Point2D(0.0, -1.0);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.addVertex(expectedVertex4);
      scaledPolygonExpected.addVertex(expectedVertex5);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);

      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithPointScaleExteriorPolygon()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(0.5, 0.5);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.5, 0.5);
      Point2D interiorVertex1 = new Point2D(-0.5, 0.5);
      Point2D interiorVertex2 = new Point2D(0.5, -0.5);
      Point2D interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(0.5, 0.5);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.2, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);


      expectedVertex0 = new Point2D(0.6, 0.6);
      Point2D expectedVertex1 = new Point2D(0.4, 0.6);
      Point2D expectedVertex2 = new Point2D(0.6, 0.4);
      Point2D expectedVertex3 = new Point2D(0.4, 0.4);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.6, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);



      interiorVertex0 = new Point2D(0.2, 0.2);
      interiorVertex1 = new Point2D(-0.5, 0.5);
      interiorVertex2 = new Point2D(0.2, -0.2);
      interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      expectedVertex0 = new Point2D(0.9, 0.6);
      expectedVertex1 = new Point2D(0.4, 0.6);
      expectedVertex2 = new Point2D(0.9, 0.4);
      expectedVertex3 = new Point2D(0.4, 0.4);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.6, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithLineScaleExteriorPolygon()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(0.5, 0.5);
      Point2D exteriorVertex1 = new Point2D(0.5, 1.0);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.5, 0.5);
      Point2D interiorVertex1 = new Point2D(-0.5, 0.5);
      Point2D interiorVertex2 = new Point2D(0.5, -0.5);
      Point2D interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(0.5, 0.75);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.2, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.2, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);

      exteriorVertex0.set(0.5, 0.25);
      exteriorVertex1.set(0.5, 1.35);
      exteriorPolygon.clear();
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.update();

      expectedVertex0 = new Point2D(0.6, 0.75);
      Point2D expectedVertex1 = new Point2D(0.6, 0.85);
      Point2D expectedVertex2 = new Point2D(0.5, 0.95);
      Point2D expectedVertex3 = new Point2D(0.5, 0.65);
      Point2D expectedVertex4 = new Point2D(0.4, 0.75);
      Point2D expectedVertex5 = new Point2D(0.4, 0.85);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.addVertex(expectedVertex4);
      scaledPolygonExpected.addVertex(expectedVertex5);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.6, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);



      exteriorVertex0.set(0.5, 0.25);
      exteriorVertex1.set(0.5, 1.25);
      exteriorPolygon.clear();
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.update();

      expectedVertex0 = new Point2D(0.6, 0.75);
      expectedVertex1 = new Point2D(0.5, 0.85);
      expectedVertex2 = new Point2D(0.5, 0.65);
      expectedVertex3 = new Point2D(0.4, 0.75);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.6, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);


      exteriorVertex0.set(0.5, 0.25);
      exteriorVertex1.set(0.5, 1.35);
      exteriorPolygon.clear();
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.update();

      interiorVertex0 = new Point2D(0.5, 0.5);
      interiorVertex1 = new Point2D(-0.5, 0.5);
      interiorVertex2 = new Point2D(0.5, -0.5);
      interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      expectedVertex0 = new Point2D(0.4, 0.85);
      expectedVertex1 = new Point2D(0.5, 0.95);
      expectedVertex2 = new Point2D(0.6, 0.85);
      expectedVertex3 = new Point2D(0.6, 0.75);
      expectedVertex4 = new Point2D(0.5, 0.65);
      expectedVertex5 = new Point2D(0.4, 0.75);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.addVertex(expectedVertex2);
      scaledPolygonExpected.addVertex(expectedVertex3);
      scaledPolygonExpected.addVertex(expectedVertex4);
      scaledPolygonExpected.addVertex(expectedVertex5);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.6, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);




      exteriorVertex0.set(0.5, 0.25);
      exteriorVertex1.set(0.5, 1.25);
      exteriorPolygon.clear();
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.update();


      interiorVertex0 = new Point2D(0.5, 0.5);
      interiorVertex1 = new Point2D(-0.5, 0.5);
      interiorVertex2 = new Point2D(0.5, -0.5);
      interiorVertex3 = new Point2D(-0.5, -0.5);
      interiorPolygon.clear();
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      expectedVertex0 = new Point2D(0.5, 0.75);
      scaledPolygonExpected.clear();
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.update();

      success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, -0.2, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertFalse(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublingCollapseExteriorRectangleToLine()
   {
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();

      ConvexPolygon2D exteriorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D interiorPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      ConvexPolygon2D scaledPolygonExpected = new ConvexPolygon2D();

      Point2D exteriorVertex0 = new Point2D(0.7, -0.5);
      Point2D exteriorVertex1 = new Point2D(0.7, -0.4);
      Point2D exteriorVertex2 = new Point2D(-0.3, -0.5);
      Point2D exteriorVertex3 = new Point2D(-0.3, -0.5);
      exteriorPolygon.addVertex(exteriorVertex0);
      exteriorPolygon.addVertex(exteriorVertex1);
      exteriorPolygon.addVertex(exteriorVertex2);
      exteriorPolygon.addVertex(exteriorVertex3);
      exteriorPolygon.update();

      Point2D interiorVertex0 = new Point2D(0.11, -0.085 / 2.0);
      Point2D interiorVertex1 = new Point2D(0.11, 0.085 / 2.0);
      Point2D interiorVertex2 = new Point2D(-0.11, 0.055);
      Point2D interiorVertex3 = new Point2D(-0.11, 0.055);
      interiorPolygon.addVertex(interiorVertex0);
      interiorPolygon.addVertex(interiorVertex1);
      interiorPolygon.addVertex(interiorVertex2);
      interiorPolygon.addVertex(interiorVertex3);
      interiorPolygon.update();

      Point2D expectedVertex0 = new Point2D(-0.3 + 0.11, -0.45);
      Point2D expectedVertex1 = new Point2D(0.7 - 0.11, -0.45);
      scaledPolygonExpected.addVertex(expectedVertex0);
      scaledPolygonExpected.addVertex(expectedVertex1);
      scaledPolygonExpected.update();

      boolean success = scaler.scaleConvexPolygonToContainInteriorPolygon(exteriorPolygon, interiorPolygon, 0.0, scaledPolygon);
      assertTrue(scaledPolygonExpected.epsilonEquals(scaledPolygon, 1e-7));
      assertTrue(success);
   }
}
