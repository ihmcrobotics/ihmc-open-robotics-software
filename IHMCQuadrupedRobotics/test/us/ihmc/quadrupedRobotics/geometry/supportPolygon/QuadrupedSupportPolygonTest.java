package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.google.caliper.Benchmark;
import com.google.caliper.api.VmOptions;
import com.google.caliper.runner.CaliperMain;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverter;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.RunnableThatThrows;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
@VmOptions("-XX:-TieredCompilation")
public class QuadrupedSupportPolygonTest
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructorsGettersAndSetters()
   {
      QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
      
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      quadrupedSupportPolygon = new QuadrupedSupportPolygon(framePoints);
      
      quadrupedSupportPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, framePoints.get(RobotQuadrant.FRONT_LEFT));
      quadrupedSupportPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, quadrupedSupportPolygon.getFootstep(RobotQuadrant.FRONT_LEFT));
      
      quadrupedSupportPolygon = new QuadrupedSupportPolygon(quadrupedSupportPolygon);
      quadrupedSupportPolygon.set(quadrupedSupportPolygon);
      for (RobotQuadrant robotQuadrant : quadrupedSupportPolygon.getSupportingQuadrantsInOrder())
      {
         assertTrue("not equals", framePoints.get(robotQuadrant).epsilonEquals(quadrupedSupportPolygon.getFootstep(robotQuadrant), 1e-7));
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         quadrupedSupportPolygon.setFootstep(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      quadrupedSupportPolygon.printOutPolygon(getClass().getMethods()[0].getName());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3d tuple3dToPack = new Point3d();
         quadrupedSupportPolygon.getFootstep(robotQuadrant).get(tuple3dToPack);
         assertEquals("Point not equal", footPoints.get(robotQuadrant), tuple3dToPack);
      }
      
      assertEquals("RefFrame getter not work", ReferenceFrame.getWorldFrame(), quadrupedSupportPolygon.getReferenceFrame());
      
      System.out.println(quadrupedSupportPolygon.toString());
      
      quadrupedSupportPolygon.changeFrame(ReferenceFrame.getWorldFrame());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVariousMethodsForCodeCoverage()
   {
      final QuadrupedSupportPolygon emptyPolygon = createEmptyPolygon();
      JUnitTools.assertExceptionThrown(EmptySupportPolygonException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            emptyPolygon.getFirstSupportingQuadrant();
         }
      });
      JUnitTools.assertExceptionThrown(EmptySupportPolygonException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            emptyPolygon.getLastSupportingQuadrant();
         }
      });
      
      final QuadrupedSupportPolygon fullPolygon = createSimplePolygon();
      JUnitTools.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            fullPolygon.getLastNonSupportingQuadrant();
         }
      });
      JUnitTools.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            fullPolygon.getFirstNonSupportingQuadrant();
         }
      });
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertEquals("not right", robotQuadrant, createPolygonWithoutLeg(robotQuadrant).getFirstNonSupportingQuadrant());
         assertEquals("not right", robotQuadrant, createPolygonWithoutLeg(robotQuadrant).getLastNonSupportingQuadrant());
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetLegPairs()
   {
      final QuadrupedSupportPolygon quadrupedSupportPolygon = createSimplePolygon();
      
      RobotQuadrant[][] legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 4", 4, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.FRONT_LEFT);
      
      legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 3", 3, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      
      legPairs = quadrupedSupportPolygon.getLegPairs();
      assertEquals("not 1", 1, legPairs.length);
      
      quadrupedSupportPolygon.removeFootstep(RobotQuadrant.HIND_LEFT);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSupportPolygon createPolygonWithoutLeg = createPolygonWithoutLeg(robotQuadrant);
         assertEquals("not 3", 3, createPolygonWithoutLeg.getLegPairs().length);
      }
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            quadrupedSupportPolygon.getLegPairs();
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceInside()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      assertEquals("not 0.0 inside", 0.0, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.0,  0.0, 0.0)), 1e-7);
      assertEquals("not 0.25 inside", 0.25, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.25,  0.5, 0.0)), 1e-7);
      assertTrue("should be inside", simplePolygon.isInside(new FramePoint(WORLD, 0.25,  0.5, 0.0)));
      
      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.5, 0.0);
      double distance = simplePolygon.getDistanceInsideInCircle2d(point);
      assertEquals("not 0.5 inside", 0.5, distance, 1e-7);
      point.set(0.4, 0.5, 1.0);
      distance = simplePolygon.getDistanceInsideInCircle2d(point);
      assertEquals("not 0.4 inside", 0.4, distance, 1e-7);
      
      simplePolygon.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      assertEquals("not 0.05 inside", 0.05, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.06,  0.05, 0.0)), 1e-7);
      
      simplePolygon.removeFootstep(RobotQuadrant.FRONT_LEFT);
      assertEquals("not 0.0 inside", 0.0, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.06,  0.0, 0.0)), 1e-7);
      assertEquals("not -0.1 inside", -0.1, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.06,  0.1, 0.0)), 1e-7);
      
      simplePolygon.removeFootstep(RobotQuadrant.HIND_RIGHT);
      assertEquals("not 0.0 inside", 0.0, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.0,  0.0, 0.0)), 1e-7);
      assertEquals("not -1.0 inside", -1.0, simplePolygon.getDistanceInside2d(new FramePoint(WORLD, 0.0,  1.0, 0.0)), 1e-7);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetInCircleRadius()
   {
      final QuadrupedSupportPolygon poly = createSimplePolygon();
      assertEquals("not 0.5 radius", 0.5, poly.getInCircleRadius2d(), 1e-7);
      
      poly.removeFootstep(RobotQuadrant.FRONT_LEFT);
      
      FramePoint inCircleCenterToPack = new FramePoint();
      double radius = poly.getInCircle2d(inCircleCenterToPack);
      
      assertEquals("not 0.292893 radius", 0.292893, radius, 1e-5);
      assertTrue("not equal", inCircleCenterToPack.epsilonEquals(new Vector3d(0.7071067, 0.292893, 0.0), 1e-5));

      poly.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            FramePoint intersectionToPack = new FramePoint();
            poly.getInCirclePoint2d(intersectionToPack);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSizeMethods()
   {
      QuadrupedSupportPolygon variableSizedPolygon = create3LegPolygon();
      assertEquals("not size 3", 3, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 2", 2, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 1", 1, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(variableSizedPolygon.getFirstSupportingQuadrant());
      assertEquals("not size 0", 0, variableSizedPolygon.size());
      variableSizedPolygon.removeFootstep(RobotQuadrant.FRONT_LEFT);
      variableSizedPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, null);
      assertEquals("not size 0", 0, variableSizedPolygon.size());
      
      variableSizedPolygon = createSimplePolygon();
      assertEquals("not size 4", 4, variableSizedPolygon.size());
      variableSizedPolygon.setFootstep(variableSizedPolygon.getLastSupportingQuadrant(), null);
      assertEquals("not size 3", 3, variableSizedPolygon.size());
      variableSizedPolygon.setFootstep(variableSizedPolygon.getLastSupportingQuadrant(), null);
      assertEquals("not size 2", 2, variableSizedPolygon.size());
      variableSizedPolygon.setFootstep(variableSizedPolygon.getLastSupportingQuadrant(), null);
      assertEquals("not size 1", 1, variableSizedPolygon.size());
      variableSizedPolygon.setFootstep(variableSizedPolygon.getLastSupportingQuadrant(), null);
      assertEquals("not size 0", 0, variableSizedPolygon.size());
      
      variableSizedPolygon = createSimplePolygon();
      variableSizedPolygon.clear();
      assertEquals("not size 0", 0, variableSizedPolygon.size());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPolygonOrdering()
   {
      QuadrupedSupportPolygon outOfOrderPolygon = createOutOfOrderPolygon();
      
      int i = 0;
      for (RobotQuadrant robotQuadrant : outOfOrderPolygon.getSupportingQuadrantsInOrder())
      {
         if (i == 0)
            assertEquals("wrong quad", RobotQuadrant.FRONT_LEFT, robotQuadrant);
         else if (i == 1)
            assertEquals("wrong quad", RobotQuadrant.FRONT_RIGHT, robotQuadrant);
         else if (i == 2)
            assertEquals("wrong quad", RobotQuadrant.HIND_RIGHT, robotQuadrant);
         else
            assertEquals("wrong quad", RobotQuadrant.HIND_LEFT, robotQuadrant);
         i++;
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCentroid()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint centroid = new FramePoint();
      simplePolygon.getCentroid(centroid);
      FramePoint expected = new FramePoint(WORLD, 0.5, 0.5, 0.0);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
      simplePolygon.getCentroid(centroid);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInCircle()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint inCircle = new FramePoint();
      double radius = simplePolygon.getInCircle2d(inCircle);
      FramePoint expected = new FramePoint(WORLD, 0.5, 0.5, 0.0);
      assertTrue("not equal expected " + expected + " actual " + inCircle, expected.epsilonEquals(inCircle, 1e-7));
      assertEquals("not correct radius", 0.5, radius, 1e-7);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatchingFootsteps()
   {
      QuadrupedSupportPolygon match1 = create3LegPolygon();
      QuadrupedSupportPolygon match2 = create3LegPolygon();

      assertEquals("3 legs don't match", 3, match1.getNumberOfEqualFootsteps(match2));

      match1 = createSimplePolygon();
      match2 = createSimplePolygon();

      assertEquals("4 legs don't match", 4, match1.getNumberOfEqualFootsteps(match2));

      match1.getFootstep(RobotQuadrant.FRONT_LEFT).add(2.0, 0.0, 0.0);
      assertEquals("3 legs don't match", 3, match1.getNumberOfEqualFootsteps(match2));
      match1.getFootstep(RobotQuadrant.FRONT_RIGHT).add(2.0, 0.0, 1.0);
      assertEquals("2 legs don't match", 2, match1.getNumberOfEqualFootsteps(match2));
      match1.getFootstep(RobotQuadrant.HIND_LEFT).add(2.0, 3.0, 1.0);
      assertEquals("1 legs don't match", 1, match1.getNumberOfEqualFootsteps(match2));
      match1.getFootstep(RobotQuadrant.HIND_RIGHT).add(2.0, -3.0, 1.0);
      assertEquals("0 legs don't match", 0, match1.getNumberOfEqualFootsteps(match2));
      
      match1 = createSimplePolygon();
      match2 = createSimplePolygon();
      
      match2.removeFootstep(RobotQuadrant.HIND_RIGHT);
      assertEquals("0 legs don't match", 3, match1.getNumberOfEqualFootsteps(match2));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetNextCounterClockwiseSupportingQuadrant()
   {
      QuadrupedSupportPolygon fourFootPoly = createSimplePolygon();
      assertEquals("not correct", RobotQuadrant.FRONT_LEFT, fourFootPoly.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.FRONT_RIGHT));
      assertEquals("not correct", RobotQuadrant.HIND_LEFT, fourFootPoly.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.FRONT_LEFT));
      assertEquals("not correct", RobotQuadrant.HIND_RIGHT, fourFootPoly.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.HIND_LEFT));
      assertEquals("not correct", RobotQuadrant.FRONT_RIGHT, fourFootPoly.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.HIND_RIGHT));

      assertEquals("not correct", RobotQuadrant.FRONT_RIGHT, fourFootPoly.getNextClockwiseSupportingQuadrant(RobotQuadrant.FRONT_LEFT));
      assertEquals("not correct", RobotQuadrant.HIND_RIGHT, fourFootPoly.getNextClockwiseSupportingQuadrant(RobotQuadrant.FRONT_RIGHT));
      assertEquals("not correct", RobotQuadrant.HIND_LEFT, fourFootPoly.getNextClockwiseSupportingQuadrant(RobotQuadrant.HIND_RIGHT));
      assertEquals("not correct", RobotQuadrant.FRONT_LEFT, fourFootPoly.getNextClockwiseSupportingQuadrant(RobotQuadrant.HIND_LEFT));
      
      QuadrupedSupportPolygon polyNoFL = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      assertEquals("not correct", RobotQuadrant.HIND_LEFT, polyNoFL.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.FRONT_RIGHT));
      assertEquals("not correct", RobotQuadrant.FRONT_RIGHT, polyNoFL.getNextClockwiseSupportingQuadrant(RobotQuadrant.HIND_LEFT));
      
      final QuadrupedSupportPolygon emptyPolygon = createEmptyPolygon();
      JUnitTools.assertExceptionThrown(EmptySupportPolygonException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            emptyPolygon.getNextCounterClockwiseSupportingQuadrant(RobotQuadrant.FRONT_LEFT);
         }
      });
      JUnitTools.assertExceptionThrown(EmptySupportPolygonException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            emptyPolygon.getNextClockwiseSupportingQuadrant(RobotQuadrant.FRONT_LEFT);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetOrCreateFootstep()
   {
      QuadrupedSupportPolygon noFL = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      FramePoint footstep1 = noFL.reviveFootstep(RobotQuadrant.FRONT_LEFT);
      assertNotNull("null", footstep1);
      FramePoint footstep2 = noFL.reviveFootstep(RobotQuadrant.FRONT_LEFT);
      assertTrue("not same ref", footstep1 == footstep2);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetWhichFootstepHasMoved()
   {
      RobotQuadrant swingLegFromHereToNextPolygon;
      final QuadrupedSupportPolygon firstPoly = createSimplePolygon();
      final QuadrupedSupportPolygon secondPoly = createSimplePolygon();
      
      JUnitTools.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            firstPoly.getWhichFootstepHasMoved(secondPoly);
         }
      });
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         secondPoly.getFootstep(robotQuadrant).add(1.0, 0.0, 0.0);
         swingLegFromHereToNextPolygon = firstPoly.getWhichFootstepHasMoved(secondPoly);
         assertEquals("not swing", robotQuadrant, swingLegFromHereToNextPolygon);
         firstPoly.getFootstep(robotQuadrant).add(1.0, 0.0, 0.0);
      }
      
      secondPoly.getFootstep(RobotQuadrant.FRONT_LEFT).add(1.0, 0.0, 0.0);
      secondPoly.getFootstep(RobotQuadrant.FRONT_RIGHT).add(1.0, 0.0, 0.0);
      JUnitTools.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            firstPoly.getWhichFootstepHasMoved(secondPoly);
         }
      });
      
      secondPoly.removeFootstep(RobotQuadrant.HIND_LEFT);
      JUnitTools.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            firstPoly.getWhichFootstepHasMoved(secondPoly);
         }
      });
      
      firstPoly.removeFootstep(RobotQuadrant.HIND_RIGHT);
      JUnitTools.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            firstPoly.getWhichFootstepHasMoved(secondPoly);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testContainsSameQuadrants()
   {
      QuadrupedSupportPolygon poly1 = createSimplePolygon();
      QuadrupedSupportPolygon poly2 = createSimplePolygon();
      assertTrue("not same feet", poly1.containsSameQuadrants(poly2));
      
      poly2.removeFootstep(RobotQuadrant.FRONT_LEFT);
      assertFalse("same feet", poly1.containsSameQuadrants(poly2));
      
      poly1.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      assertFalse("same feet", poly1.containsSameQuadrants(poly2));
      
      poly2.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      poly1.removeFootstep(RobotQuadrant.FRONT_LEFT);
      assertTrue("not same feet", poly1.containsSameQuadrants(poly2));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAndReplaceFootstep()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      FramePoint footstep = poly.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedSupportPolygon replaceFootstep = new QuadrupedSupportPolygon();
      poly.getAndReplaceFootstep(replaceFootstep, RobotQuadrant.FRONT_LEFT, new FramePoint(ReferenceFrame.getWorldFrame(), 2.0, 1.0, 1.0));
      assertFalse("equal", footstep.epsilonEquals(replaceFootstep.getFootstep(RobotQuadrant.FRONT_LEFT), 0.1));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAndRemoveFootstep()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      QuadrupedSupportPolygon removeFootstep = new QuadrupedSupportPolygon();
      poly.getAndRemoveFootstep(removeFootstep, RobotQuadrant.FRONT_LEFT);
      removeFootstep.getFootstep(RobotQuadrant.FRONT_LEFT);
      assertNull("not null", removeFootstep.getFootstep(RobotQuadrant.FRONT_LEFT));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAndSwapSameSideFootsteps()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      FramePoint footstepFL = poly.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint footstepHL = poly.getFootstep(RobotQuadrant.HIND_LEFT);
      QuadrupedSupportPolygon pack = new QuadrupedSupportPolygon();
      poly.getAndSwapSameSideFootsteps(pack, RobotSide.LEFT);
      assertTrue("not equal", footstepFL.epsilonEquals(pack.getFootstep(RobotQuadrant.HIND_LEFT), 1e-7));
      assertTrue("not equal", footstepHL.epsilonEquals(pack.getFootstep(RobotQuadrant.FRONT_LEFT), 1e-7));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTranslatePolygon()
   {
      QuadrupedSupportPolygon poly = createZeroedPolygon();
      Vector3d translateBy = new Vector3d(1.0, 2.0, -3.0);
      poly.translate(translateBy);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertTrue("not equal", poly.getFootstep(robotQuadrant).epsilonEquals(translateBy, 1e-7));
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYawAboutCentroid()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      Point3d origin = new Point3d(0.0, 0.0, 0.0);
      Point3d bottomRight = new Point3d(1.0, 0.0, 0.0);
      Point3d topLeft = new Point3d(0.0, 1.0, 0.0);
      Point3d topRight = new Point3d(1.0, 1.0, 0.0);
      footPoints.set(RobotQuadrant.HIND_LEFT, origin);
      footPoints.set(RobotQuadrant.HIND_RIGHT, bottomRight);
      footPoints.set(RobotQuadrant.FRONT_LEFT, topLeft);
      footPoints.set(RobotQuadrant.FRONT_RIGHT, topRight);
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      QuadrupedSupportPolygon polygon = new QuadrupedSupportPolygon(framePoints);
      
      polygon.yawAboutCentroid(Math.PI);
      
      assertTrue("not equal", polygon.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(topRight, 1e-7));
      assertTrue("not equal", polygon.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(topLeft, 1e-7));
      assertTrue("not equal", polygon.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(bottomRight, 1e-7));
      assertTrue("not equal", polygon.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(origin, 1e-7));
      
      polygon.yawAboutCentroid(-Math.PI / 2);
      
      String message = "not equal expected: " + bottomRight + " actual " + polygon.getFootstep(RobotQuadrant.HIND_LEFT).getPoint();
      assertTrue(message, polygon.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(bottomRight, 1e-7));
      String message2 = "not equal expected: " + topRight + " actual " + polygon.getFootstep(RobotQuadrant.HIND_RIGHT).getPoint();
      assertTrue(message2, polygon.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(topRight, 1e-7));
      String message3 = "not equal expected: " + origin + " actual " + polygon.getFootstep(RobotQuadrant.FRONT_LEFT).getPoint();
      assertTrue(message3, polygon.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(origin, 1e-7));
      
      polygon.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      polygon.yawAboutCentroid(Math.PI / 4);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetLowestAndHighestFootstep()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSupportPolygon poly = createExtremeFootPolygon(robotQuadrant, new Point3d(1.0, 1.0, 20.0));
         assertEquals("not highest", robotQuadrant, poly.getHighestFootstep());
      }
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSupportPolygon poly = createExtremeFootPolygon(robotQuadrant, new Point3d(1.0, 1.0, -20.0));
         assertEquals("not lowest", robotQuadrant, poly.getLowestFootstep());
         assertEquals("not correct", -20.0, poly.getLowestFootstepZHeight(), 1e-7);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestFootstep()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertEquals("not closest", robotQuadrant, poly.getClosestFootstep(poly.getFootstep(robotQuadrant)));
      }
      
      assertEquals("not closest", RobotQuadrant.FRONT_RIGHT, poly.getClosestFootstep(new FramePoint(ReferenceFrame.getWorldFrame(), 2.0, 2.0, 0.0)));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetCentroid()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      FramePoint centroidToPack2d = new FramePoint();
      FramePoint2d centroid2dToPack2d = new FramePoint2d();
      poly.getCentroid(centroidToPack2d);
      poly.getCentroid2d(centroid2dToPack2d);
      assertTrue("not centroid", centroidToPack2d.epsilonEquals(new Point3d(0.5, 0.5, 0.0), 1e-7));
      assertTrue("not centroid", centroid2dToPack2d.epsilonEquals(new Point2d(0.5, 0.5), 1e-7));
      
      poly.translate(new Vector3d(2.0, -2.0, 0.0));
      poly.getCentroid(centroidToPack2d);
      poly.getCentroid2d(centroid2dToPack2d);
      assertTrue("not centroid", centroidToPack2d.epsilonEquals(new Point3d(2.5, -1.5, 0.0), 1e-7));
      assertTrue("not centroid", centroid2dToPack2d.epsilonEquals(new Point2d(2.5, -1.5), 1e-7));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetShrunkenPolygon()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      
      poly.getShrunkenPolygon2d(poly, RobotQuadrant.FRONT_LEFT, 0.25);
      poly.getShrunkenPolygon2d(poly, RobotQuadrant.FRONT_RIGHT, 0.25);
      poly.getShrunkenPolygon2d(poly, RobotQuadrant.HIND_LEFT, 0.25);
      poly.getShrunkenPolygon2d(poly, RobotQuadrant.HIND_RIGHT, 0.25);
      
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.25, 0.25, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(0.75, 0.25, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.25, 0.75, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.75, 0.75, 0.0), 1e-7));
      
      poly.shrinkPolygon2d(0.05);
      
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.30, 0.30, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(0.70, 0.30, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.30, 0.70, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.70, 0.70, 0.0), 1e-7));
      
      poly.shrinkPolygon2d(RobotQuadrant.FRONT_LEFT, -0.05);
      poly.shrinkPolygon2d(RobotQuadrant.FRONT_RIGHT, -0.05);
      poly.shrinkPolygon2d(RobotQuadrant.HIND_RIGHT, -0.05);
      poly.shrinkPolygon2d(RobotQuadrant.HIND_LEFT, -0.05);
      
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.25, 0.25, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(0.75, 0.25, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.25, 0.75, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.75, 0.75, 0.0), 1e-7));
      
      poly.shrinkPolygon2d(RobotQuadrant.FRONT_LEFT, 0.05);
      poly.shrinkPolygon2d(RobotQuadrant.FRONT_RIGHT, -0.05);
      poly.shrinkPolygon2d(RobotQuadrant.HIND_RIGHT, 0.10);
      poly.shrinkPolygon2d(RobotQuadrant.HIND_LEFT, -0.15);
      
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.10, 0.35, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(0.80, 0.35, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.10, 0.7, 0.0), 1e-7));
      assertTrue("not shrunk correctly", poly.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.80, 0.7, 0.0), 1e-7));
      
      final QuadrupedSupportPolygon createEmptyPolygon = createEmptyPolygon();
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            createEmptyPolygon.getShrunkenPolygon2d(createEmptyPolygon, RobotQuadrant.FRONT_LEFT, 1.0);
         }
      });
      
      QuadrupedSupportPolygon poly3 = create3LegPolygon();
      
      poly3.shrinkPolygon2d(0.1);
      
      assertTrue("not shrunk correctly", poly3.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.97071, 0.92928, 0.0), 1e-5));
      assertTrue("not shrunk correctly", poly3.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(0.9, 0.1, 0.0), 1e-5));
      assertTrue("not shrunk correctly", poly3.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.070710, 0.029289, 0.0), 1e-5));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetShrunkenCommonPolygon2d()
   {
      QuadrupedSupportPolygon poly1 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      QuadrupedSupportPolygon poly2 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      QuadrupedSupportPolygon poly3 = new QuadrupedSupportPolygon();
      
      poly1.getCommonTriangle2d(poly2, poly3, RobotQuadrant.FRONT_RIGHT);
      
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.5, 0.5, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.0, 0.0, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(1.0, 0.0, 0.0), 1e-7));
      
      QuadrupedSupportPolygon tempPoly = new QuadrupedSupportPolygon();
      poly1.getShrunkenCommonTriangle2d(poly2, poly3, tempPoly, RobotQuadrant.FRONT_RIGHT, 0.1, 0.1, 0.1);
      
      Vector3d expected;
      FramePoint actual;
      actual = poly3.getFootstep(RobotQuadrant.HIND_LEFT);
      expected = new Vector3d(0.24142, 0.1, 0.0);
      assertTrue("not common expected: " + expected + " actual: " + actual.getPoint(), actual.epsilonEquals(expected, 1e-5));
      actual = poly3.getFootstep(RobotQuadrant.HIND_RIGHT);
      expected = new Vector3d(0.75858, 0.1, 0.0);
      assertTrue("not common expected: " + expected + " actual: " + actual.getPoint(), actual.epsilonEquals(expected, 1e-5));
      actual = poly3.getFootstep(RobotQuadrant.FRONT_RIGHT);
      expected = new Vector3d(0.5, 0.35858, 0.0);
      assertTrue("not common expected: " + expected + " actual: " + actual.getPoint(), actual.epsilonEquals(expected, 1e-5));
      
      poly1 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly2 = createPolygonWithoutLeg(RobotQuadrant.HIND_LEFT);
      poly3 = new QuadrupedSupportPolygon();
      
      poly1.getCommonTriangle2d(poly2, poly3, RobotQuadrant.HIND_LEFT);
      
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(1.0, 1.0, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.5, 0.5, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_RIGHT).epsilonEquals(new Vector3d(1.0, 0.0, 0.0), 1e-7));
      
      poly1 = createPolygonWithoutLeg(RobotQuadrant.HIND_RIGHT);
      poly2 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      poly3 = new QuadrupedSupportPolygon();
      
      poly1.getCommonTriangle2d(poly2, poly3, RobotQuadrant.FRONT_RIGHT);
      
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.0, 1.0, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.5, 0.5, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.0, 0.0, 0.0), 1e-7));
      
      poly1 = createPolygonWithoutLeg(RobotQuadrant.HIND_RIGHT);
      poly2 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      poly3 = new QuadrupedSupportPolygon();
      
      poly1.getCommonTriangle2d(poly2, poly3, RobotQuadrant.FRONT_RIGHT);
      
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_LEFT).epsilonEquals(new Vector3d(0.0, 1.0, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.FRONT_RIGHT).epsilonEquals(new Vector3d(0.5, 0.5, 0.0), 1e-7));
      assertTrue("not common", poly3.getFootstep(RobotQuadrant.HIND_LEFT).epsilonEquals(new Vector3d(0.0, 0.0, 0.0), 1e-7));
      
      final QuadrupedSupportPolygon poly4 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      final QuadrupedSupportPolygon poly5 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly5.translate(0.5, 0.0, 0.0);
      final QuadrupedSupportPolygon poly6 = new QuadrupedSupportPolygon();
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly4.getCommonTriangle2d(poly5, poly6, RobotQuadrant.FRONT_RIGHT);
         }
      });
      
      final QuadrupedSupportPolygon poly7 = createEmptyPolygon();
      final QuadrupedSupportPolygon poly8 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly8.translate(0.5, 0.0, 0.0);
      final QuadrupedSupportPolygon poly9 = new QuadrupedSupportPolygon();
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly7.getCommonTriangle2d(poly8, poly9, RobotQuadrant.FRONT_RIGHT);
         }
      });
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly8.getCommonTriangle2d(poly7, poly9, RobotQuadrant.FRONT_RIGHT);
         }
      });
      poly8.translate(-0.5, 0.0, 0.0);
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly8.getCommonTriangle2d(poly4, poly9, RobotQuadrant.HIND_LEFT);
         }
      });
      
      // Test diagonal swing legs throw exception
      final QuadrupedSupportPolygon poly10 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      final QuadrupedSupportPolygon poly11 = createPolygonWithoutLeg(RobotQuadrant.HIND_LEFT);
      final QuadrupedSupportPolygon poly12 = new QuadrupedSupportPolygon();
      
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly10.getCommonTriangle2d(poly11, poly12, RobotQuadrant.FRONT_RIGHT);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceFromP1ToTrotLine()
   {
      QuadrupedSupportPolygon poly = createDiamondPolygon();
      FramePoint flFoot = poly.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint hrFoot = poly.getFootstep(RobotQuadrant.HIND_RIGHT);
      double distance = poly.getDistanceFromP1ToTrotLineInDirection2d(RobotQuadrant.FRONT_RIGHT, flFoot, hrFoot);
      assertEquals("not 0.5", 0.5, distance, 1e-7);
      flFoot.add(0.25, 0.0, 0.0);
      distance = poly.getDistanceFromP1ToTrotLineInDirection2d(RobotQuadrant.FRONT_RIGHT, flFoot, hrFoot);
      assertEquals("not 0.25", 0.25, distance, 1e-7);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetBounds()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      Point2d minToPack = new Point2d();
      Point2d maxToPack = new Point2d();
      poly.getBounds(minToPack , maxToPack);
      
      assertEquals("not correct", minToPack.getX(), 0.0, 1e-7);
      assertEquals("not correct", minToPack.getY(), 0.0, 1e-7);
      assertEquals("not correct", maxToPack.getX(), 1.0, 1e-7);
      assertEquals("not correct", maxToPack.getY(), 1.0, 1e-7);
      
      poly = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly.getBounds(minToPack , maxToPack);
      
      assertEquals("not correct", minToPack.getX(), 0.0, 1e-7);
      assertEquals("not correct", minToPack.getY(), 0.0, 1e-7);
      assertEquals("not correct", maxToPack.getX(), 1.0, 1e-7);
      assertEquals("not correct", maxToPack.getY(), 1.0, 1e-7);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInside()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      assertTrue("not correct", poly.isInside(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.5, 0.0)));
      assertFalse("not correct", poly.isInside(new FramePoint(ReferenceFrame.getWorldFrame(), 1.5, 0.5, 0.0)));
      assertTrue("not correct", poly.isInside(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.5, 1.0)));
      assertTrue("not correct", poly.isInside(new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.5, -1.0)));
      assertTrue("not correct", poly.isInside(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.5, 0.5)));
      assertFalse("not correct", poly.isInside(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.5, -0.5)));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetNominalYawPitchRoll()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      QuadrupedSupportPolygon poly = new QuadrupedSupportPolygon(framePoints);
      assertEquals("not 90", 90.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalYaw()), 1e-7);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalPitch()), 1e-7);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalRoll()), 1e-7);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalYawHindLegs()), 1e-7);
      poly.yawAboutCentroid(-Math.PI / 2);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalYaw()), 1e-7);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalPitch()), 1e-7);
      assertEquals("not 0", 0.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalRoll()), 1e-7);
      assertEquals("not -90", -90.0, BodeUnitsConverter.convertRadianToDegrees(poly.getNominalYawHindLegs()), 1e-7);
      
      QuadrupedSupportPolygon poly1 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      QuadrupedSupportPolygon poly2 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      final QuadrupedSupportPolygon poly3 = new QuadrupedSupportPolygon();
      
      poly1.getCommonTriangle2d(poly2, poly3, RobotQuadrant.FRONT_RIGHT);
      assertEquals("not 135", 135.0, BodeUnitsConverter.convertRadianToDegrees(poly3.getNominalYaw()), 1e-7);
      poly3.yawAboutCentroid(Math.PI / 2);
      assertEquals("not -135", -135.0, BodeUnitsConverter.convertRadianToDegrees(poly3.getNominalYaw()), 1e-7);
      
      poly3.removeFootstep(RobotQuadrant.FRONT_RIGHT);
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getNominalYaw();
         }
      });
      poly3.removeFootstep(RobotQuadrant.HIND_LEFT);
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getNominalYawHindLegs();
         }
      });
      poly3.removeFootstep(RobotQuadrant.HIND_RIGHT);
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getNominalYawHindLegs();
         }
      });
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getNominalRoll();
         }
      });JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getNominalPitch();
         }
      });
      
      QuadrupedSupportPolygon pitchedUp = createPitchedUpPolygon();
      assertEquals("not -45", -45.0, BodeUnitsConverter.convertRadianToDegrees(pitchedUp.getNominalPitch()), 1e-7);
      QuadrupedSupportPolygon pitchedDown = createPitchedDownPolygon();
      assertEquals("not 45", 45.0, BodeUnitsConverter.convertRadianToDegrees(pitchedDown.getNominalPitch()), 1e-7);
      QuadrupedSupportPolygon rolled = createRolledPolygon();
      assertEquals("not -45", -45.0, BodeUnitsConverter.convertRadianToDegrees(rolled.getNominalRoll()), 1e-7);
      
      final QuadrupedSupportPolygon zeroedPolygon = createZeroedPolygon();
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            zeroedPolygon.getNominalRoll();
         }
      });
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            zeroedPolygon.getNominalPitch();
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFootstepMidpoints()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint framePointToPack = new FramePoint();
      simplePolygon.getFrontMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(0.5, 1.0, 0.0), 1e-7));
      simplePolygon.getHindMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(0.5, 0.0, 0.0), 1e-7));
      
      QuadrupedSupportPolygon poly1;
      poly1 = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly1.getFrontMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(1.0, 1.0, 0.0), 1e-7));
      poly1 = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      poly1.getFrontMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(0.0, 1.0, 0.0), 1e-7));
      poly1.removeFootstep(RobotQuadrant.FRONT_LEFT);
      final QuadrupedSupportPolygon poly2 = poly1;
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly2.getFrontMidpoint(null);
         }
      });
      poly1 = createPolygonWithoutLeg(RobotQuadrant.HIND_LEFT);
      poly1.getHindMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(1.0, 0.0, 0.0), 1e-7));
      poly1 = createPolygonWithoutLeg(RobotQuadrant.HIND_RIGHT);
      poly1.getHindMidpoint(framePointToPack);
      assertTrue("not midpoint", framePointToPack.epsilonEquals(new Point3d(0.0, 0.0, 0.0), 1e-7));
      poly1.removeFootstep(RobotQuadrant.HIND_LEFT);
      final QuadrupedSupportPolygon poly3 = poly1;
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly3.getHindMidpoint(null);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStanceLength()
   {
      QuadrupedSupportPolygon create3LegPolygon;
      create3LegPolygon = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      assertEquals("not 1.0", 1.0, create3LegPolygon.getStanceLength(RobotSide.RIGHT), 1e-7);
      create3LegPolygon = createPolygonWithoutLeg(RobotQuadrant.FRONT_RIGHT);
      assertEquals("not 1.0", 1.0, create3LegPolygon.getStanceLength(RobotSide.LEFT), 1e-7);
      create3LegPolygon = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      assertEquals("not 1.0", 1.0, create3LegPolygon.getStanceLength(RobotSide.RIGHT), 1e-7);
      create3LegPolygon = createPolygonWithoutLeg(RobotQuadrant.HIND_RIGHT);
      assertEquals("not 1.0", 1.0, create3LegPolygon.getStanceLength(RobotSide.LEFT), 1e-7);
      
      final QuadrupedSupportPolygon createEmptyPolygon = createEmptyPolygon();
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            createEmptyPolygon.getStanceLength(RobotSide.LEFT);
         }
      });
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testEpsilonEquals()
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      assertTrue("not correct", createSimplePolygon.epsilonEquals(createSimplePolygon));
      QuadrupedSupportPolygon create3LegPolygon = create3LegPolygon();
      assertFalse("not correct", createSimplePolygon.epsilonEquals(create3LegPolygon));
      assertFalse("not correct", createSimplePolygon.epsilonEquals(null));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAreLegsCrossing()
   {
      QuadrupedSupportPolygon simple = createSimplePolygon();
      assertFalse("cross", simple.areLegsCrossing());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         double negateIfLeftSide = robotQuadrant.getSide().negateIfLeftSide(-20.0);
         QuadrupedSupportPolygon extreme = createExtremeFootPolygon(robotQuadrant, new Point3d(negateIfLeftSide, negateIfLeftSide, 0.0));
         assertTrue("not cross", extreme.areLegsCrossing());
         
         extreme.removeFootstep(robotQuadrant.getSameSideQuadrant());
         assertTrue("not cross", extreme.areLegsCrossing());
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testValidTrotPolygon()
   {
      final QuadrupedSupportPolygon createPitchedDownPolygon = createPitchedDownPolygon();
      assertFalse("trot", createPitchedDownPolygon.isValidTrotPolygon());
      
      JUnitTools.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            createPitchedDownPolygon.getRightTrotLeg();
         }
      });JUnitTools.assertExceptionThrown(RuntimeException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            createPitchedDownPolygon.getLeftTrotLeg();
         }
      });
      
      QuadrupedSupportPolygon trot = createTrotPolygon(RobotSide.LEFT);
      assertTrue("not trot", trot.isValidTrotPolygon());
      assertEquals("not trot", RobotQuadrant.HIND_RIGHT, trot.getRightTrotLeg());
      assertEquals("not trot", RobotQuadrant.FRONT_LEFT, trot.getLeftTrotLeg());
      trot = createTrotPolygon(RobotSide.RIGHT);
      assertTrue("not trot", trot.isValidTrotPolygon());
      assertEquals("not trot", RobotQuadrant.FRONT_RIGHT, trot.getRightTrotLeg());
      assertEquals("not trot", RobotQuadrant.HIND_LEFT, trot.getLeftTrotLeg());
      QuadrupedSupportPolygon side = createSidePolygon(RobotSide.LEFT);
      assertFalse("trot", side.isValidTrotPolygon());
      side = createSidePolygon(RobotSide.RIGHT);
      assertFalse("trot", side.isValidTrotPolygon());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetCenterOfCircleOfRadiusInCornerOfPolygon()
   {
      final QuadrupedSupportPolygon poly = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      poly.setFootstep(RobotQuadrant.FRONT_RIGHT, new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 1.0, 0.0));
      final FramePoint2d centerToPack = new FramePoint2d();
      poly.getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant.HIND_LEFT, 0.309015, centerToPack);
      Vector2d expected = new Vector2d(0.5, 0.309);
      assertTrue("not correct expected: " + expected + " actual: " + centerToPack, centerToPack.epsilonEquals(expected, 1e-3));
      boolean success = poly.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant.HIND_LEFT, 0.309015, centerToPack);
      assertTrue("not correct expected: " + expected + " actual: " + centerToPack, centerToPack.epsilonEquals(expected, 1e-3));
      assertTrue("success should be true", success);
      // test put larger radius in
      success = poly.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant.HIND_LEFT, 0.5, centerToPack);
      assertFalse("success should be false", success);
      // test put non-existent quadrant in
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly.getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant.FRONT_LEFT, 0.5, centerToPack);
         }
      });
      poly.setFootstep(RobotQuadrant.FRONT_LEFT, new FramePoint());
      // test put 4-sided in triangle method
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant.FRONT_LEFT, 0.309015, centerToPack);
         }
      });
      // test size 2 quadrants
      poly.removeFootstep(RobotQuadrant.FRONT_LEFT);
      poly.removeFootstep(RobotQuadrant.HIND_RIGHT);
      JUnitTools.assertExceptionThrown(UndefinedOperationException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            poly.getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant.HIND_LEFT, 0.5, centerToPack);
         }
      });
      
      QuadrupedSupportPolygon simple = createSimplePolygon();
      simple.getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant.HIND_LEFT, 0.5, centerToPack);
      expected = new Vector2d(0.5, 0.5);
      assertTrue("not correct expected: " + expected + " actual: " + centerToPack, centerToPack.epsilonEquals(expected, 1e-3));
      simple.getCenterOfCircleOfRadiusInCornerOfPolygon(RobotQuadrant.HIND_RIGHT, 0.25, centerToPack);
      expected = new Vector2d(0.75, 0.25);
      assertTrue("not correct expected: " + expected + " actual: " + centerToPack, centerToPack.epsilonEquals(expected, 1e-3));
   }

   private Random random = new Random(9123090L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackPointIntoMultipleStructuresAndCompare()
   {
      QuadrantDependentList<ReferenceFrame> frames = new QuadrantDependentList<>();
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      QuadrantDependentList<YoFramePoint> quadrantDependentList = new QuadrantDependentList<YoFramePoint>();
      
      for (RobotQuadrant robotQuadrant2 : RobotQuadrant.values)
      {
         TranslationReferenceFrame testFrame = new TranslationReferenceFrame("testFrame" + robotQuadrant2, ReferenceFrame.getWorldFrame());
         testFrame.updateTranslation(new Vector3d(randomScalar(), randomScalar(), randomScalar()));
         
         frames.set(robotQuadrant2, testFrame);
         
         quadrantDependentList.set(robotQuadrant2, new YoFramePoint("yo" + robotQuadrant2, ReferenceFrame.getWorldFrame(), registry));
      }
      
      TranslationReferenceFrame testFrame = new TranslationReferenceFrame("testFrame", ReferenceFrame.getWorldFrame());
      testFrame.updateTranslation(new Vector3d(randomScalar(), randomScalar(), randomScalar()));
      
      FramePoint framePoint = new FramePoint(testFrame, randomScalar(), randomScalar(), randomScalar());
      
      QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
      
      for (int i = 0; i < 1000; i++)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            TranslationReferenceFrame frame = (TranslationReferenceFrame) frames.get(robotQuadrant);
            
            frame.updateTranslation(new Vector3d(randomScalar(), randomScalar(), randomScalar()));
            
            framePoint.setToZero(frame);
            framePoint.changeFrame(ReferenceFrame.getWorldFrame());
            
            quadrupedSupportPolygon.setFootstep(robotQuadrant, framePoint);
            quadrantDependentList.get(robotQuadrant).set(framePoint);
            
            assertTrue("orig not equal poly", framePoint.epsilonEquals(quadrupedSupportPolygon.getFootstep(robotQuadrant), 1e-7));
            assertTrue("orig not equal list", framePoint.epsilonEquals(quadrantDependentList.get(robotQuadrant).getFrameTuple(), 1e-7));
            assertTrue("poly not equal list", quadrupedSupportPolygon.getFootstep(robotQuadrant).epsilonEquals(quadrupedSupportPolygon.getFootstep(robotQuadrant), 1e-7));
         }
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPackYoFrameConvexPolygon2d()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("boo", "yaw", ReferenceFrame.getWorldFrame(), 4, new YoVariableRegistry("bah"));
      poly.packYoFrameConvexPolygon2d(yoFrameConvexPolygon2d);
      
      for (int i = 0; i < 4; i++)
      {
         FramePoint polyPoint = poly.getFootstep(RobotQuadrant.getQuadrantNameFromOrdinal(i));
         FramePoint2d convexPoint = yoFrameConvexPolygon2d.getFrameVertex(i);
         assertTrue("not equal expected: " + polyPoint + " actual: " + convexPoint, polyPoint.epsilonEquals(convexPoint, 1e-7));
      }
      
      poly = create3LegPolygon();
      poly.packYoFrameConvexPolygon2d(yoFrameConvexPolygon2d);
      
      RobotQuadrant quadrant = poly.getFirstSupportingQuadrant();
      for (int i = 0; i < 3; i++)
      {
         FramePoint polyPoint = poly.getFootstep(quadrant);
         FramePoint2d convexPoint = yoFrameConvexPolygon2d.getFrameVertex(i);
         assertTrue("not equal expected: " + polyPoint + " actual: " + convexPoint, polyPoint.epsilonEquals(convexPoint, 1e-7));
         
         quadrant = poly.getNextClockwiseSupportingQuadrant(quadrant);
      }
   }
   
   private double randomScalar()
   {
      return 500.0 * random.nextDouble();
   }
   
   @Benchmark
   public void timeComputePolygon(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      
      FrameVector2d[] normalsToPack = new FrameVector2d[createSimplePolygon.size()];
      for (int j = 0; j < normalsToPack.length; j++)
      {
         normalsToPack[j] = new FrameVector2d();
      }
   }
   
   @Benchmark
   public void benchmarkCentroid(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint framePoint = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getCentroid(framePoint);
      }
   }
   
   @Benchmark
   public void benchmarkCentroidOpt(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint framePoint = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getCentroid(framePoint);
      }
   }
   
   @Benchmark
   public void benchmarkInCirclePoint(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint intersectionToPack = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getInCircle2d(intersectionToPack);
      }
   }

   private QuadrupedSupportPolygon createSimplePolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createDiamondPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.5, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.5, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(0.5, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 0.5, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createTrotPolygon(RobotSide leadingSide)
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      footPoints.remove(RobotQuadrant.getQuadrant(RobotEnd.FRONT, leadingSide.getOppositeSide()));
      footPoints.remove(RobotQuadrant.getQuadrant(RobotEnd.HIND, leadingSide));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createSidePolygon(RobotSide leadingSide)
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      footPoints.remove(RobotQuadrant.getQuadrant(RobotEnd.FRONT, leadingSide.getOppositeSide()));
      footPoints.remove(RobotQuadrant.getQuadrant(RobotEnd.HIND, leadingSide.getOppositeSide()));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createZeroedPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(0.0, 0.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createPolygonWithoutLeg(RobotQuadrant quadrantToSkip)
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant != quadrantToSkip)
         {
            framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
         }
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createEmptyPolygon()
   {
      return new QuadrupedSupportPolygon();
   }
   
   private QuadrupedSupportPolygon createOutOfOrderPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon create3LegPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : footPoints.quadrants())
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createPitchedUpPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(0.0, 1.0, 1.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(1.0, 1.0, 1.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createPitchedDownPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 2.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(0.0, 2.0, 2.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(-2.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(-2.0, 2.0, 0.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createRolledPolygon()
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(0.0, 1.0, 1.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(-1.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(-1.0, 1.0, 1.0));
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   private QuadrupedSupportPolygon createExtremeFootPolygon(RobotQuadrant quadrant, Point3d location)
   {
      QuadrantDependentList<Tuple3d> footPoints = new QuadrantDependentList<>();
      
      footPoints.set(RobotQuadrant.HIND_LEFT, new Point3d(0.0, 0.0, 2.0));
      footPoints.set(RobotQuadrant.HIND_RIGHT, new Point3d(0.0, 2.0, 2.0));
      footPoints.set(RobotQuadrant.FRONT_LEFT, new Point3d(-2.0, 0.0, 0.0));
      footPoints.set(RobotQuadrant.FRONT_RIGHT, new Point3d(-2.0, 2.0, 0.0));
      
      footPoints.set(quadrant, location);
      
      QuadrantDependentList<FramePoint> framePoints = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         framePoints.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame(), footPoints.get(robotQuadrant)));
      }
      
      return new QuadrupedSupportPolygon(framePoints);
   }
   
   public static void main(String[] args)
   {
      CaliperMain.main(QuadrupedSupportPolygonTest.class, args);
   }
}
