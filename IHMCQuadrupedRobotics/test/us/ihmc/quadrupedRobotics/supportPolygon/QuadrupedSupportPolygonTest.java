package us.ihmc.quadrupedRobotics.supportPolygon;

import static org.junit.Assert.*;
import static us.ihmc.tools.testing.TestPlanTarget.*;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import com.google.caliper.Benchmark;
import com.google.caliper.api.VmOptions;
import com.google.caliper.runner.CaliperMain;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.RunnableThatThrows;

@DeployableTestClass(targets = Fast)
@VmOptions("-XX:-TieredCompilation")
public class QuadrupedSupportPolygonTest
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPitch()
   {
      QuadrupedSupportPolygon pitchedUpPolygon = createPitchedUpPolygon();
      assertEquals("not 45 degrees", -Math.PI / 4, pitchedUpPolygon.getPitchInRadians(), 1e-7);
      
      QuadrupedSupportPolygon pitchedDownPolygon = createPitchedDownPolygon();
      assertEquals("not -45 degrees", Math.PI / 4, pitchedDownPolygon.getPitchInRadians(), 1e-7);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testDistanceInside()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      assertEquals("not 0.0 inside", 0.0, simplePolygon.distanceInside2d(new FramePoint(WORLD, 0.0,  0.0, 0.0)), 1e-7);
      assertEquals("not 0.25 inside", 0.25, simplePolygon.distanceInside2d(new FramePoint(WORLD, 0.25,  0.5, 0.0)), 1e-7);
      assertTrue("should be inside", simplePolygon.isInside(new FramePoint(WORLD, 0.25,  0.5, 0.0)));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCentroid()
   {
      QuadrupedSupportPolygon simplePolygon = createSimplePolygon();
      FramePoint centroid = new FramePoint();
      simplePolygon.getCentroid2d(centroid);
      FramePoint expected = new FramePoint(WORLD, 0.5, 0.5, 0.0);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
      simplePolygon.getCentroid2d(centroid);
      assertTrue("not equal expected " + expected + " actual " + centroid, expected.epsilonEquals(centroid, 1e-7));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetOrCreateFootstep()
   {
      QuadrupedSupportPolygon noFL = createPolygonWithoutLeg(RobotQuadrant.FRONT_LEFT);
      FramePoint footstep1 = noFL.getFootstepOrCreateIfNonSupporting(RobotQuadrant.FRONT_LEFT);
      assertNotNull("null", footstep1);
      FramePoint footstep2 = noFL.getFootstepOrCreateIfNonSupporting(RobotQuadrant.FRONT_LEFT);
      assertTrue("not same ref", footstep1 == footstep2);
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
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
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAndReplaceFootstep()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      FramePoint footstep = poly.getFootstep(RobotQuadrant.FRONT_LEFT);
      QuadrupedSupportPolygon replaceFootstep = new QuadrupedSupportPolygon();
      poly.getAndReplaceFootstep(replaceFootstep, RobotQuadrant.FRONT_LEFT, new FramePoint(ReferenceFrame.getWorldFrame(), 2.0, 1.0, 1.0));
      assertFalse("equal", footstep.epsilonEquals(replaceFootstep.getFootstep(RobotQuadrant.FRONT_LEFT), 0.1));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAndRemoveFootstep()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      QuadrupedSupportPolygon removeFootstep = new QuadrupedSupportPolygon();
      poly.getAndRemoveFootstep(RobotQuadrant.FRONT_LEFT, removeFootstep);
      removeFootstep.getFootstep(RobotQuadrant.FRONT_LEFT);
      assertNull("not null", removeFootstep.getFootstep(RobotQuadrant.FRONT_LEFT));
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAndSwapSameSideFootsteps()
   {
      QuadrupedSupportPolygon poly = createSimplePolygon();
      FramePoint footstepFL = poly.getFootstep(RobotQuadrant.FRONT_LEFT);
      FramePoint footstepHL = poly.getFootstep(RobotQuadrant.HIND_LEFT);
      QuadrupedSupportPolygon pack = new QuadrupedSupportPolygon();
      poly.getAndSwapSameSideFootsteps(RobotQuadrant.FRONT_LEFT, pack);
      assertTrue("not equal", footstepFL.epsilonEquals(pack.getFootstep(RobotQuadrant.HIND_LEFT), 1e-7));
      assertTrue("not equal", footstepHL.epsilonEquals(pack.getFootstep(RobotQuadrant.FRONT_LEFT), 1e-7));
   }

   private Random random = new Random(9123090L);

   @DeployableTestMethod(estimatedDuration = 0.1)
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
         createSimplePolygon.getCentroid2d(framePoint);
      }
   }
   
   @Benchmark
   public void benchmarkCentroidOpt(int reps)
   {
      QuadrupedSupportPolygon createSimplePolygon = createSimplePolygon();
      FramePoint framePoint = new FramePoint();
      
      for (int i = 0; i < reps; i++)
      {
         createSimplePolygon.getCentroid2d(framePoint);
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
   
   public static void main(String[] args)
   {
      CaliperMain.main(QuadrupedSupportPolygonTest.class, args);
   }
}
