package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class GeometricVirtualToePointCalculatorTest
{
   // Use these booleans for debugging. Otherwise, make sure they are set correctly for testing.
   private boolean ignoreProblems = false;
   private boolean repeatForever = false;
   private boolean debugViz = false;
   private boolean removeDebugVizEachTime = true;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@DeployableTestMethod(estimatedDuration = 0.7)
	@Test(timeout = 30000)
   public void testSimpleCaseWithRectangularFeetAllSquaredUp()
   {
      RobotSide upcomingSupportSide = null;
      
      double[][] leftFootPoints = new double[][] { { 0.2, 0.2 }, { -0.1, 0.2 }, { -0.1, 0.4 }, { 0.2, 0.4 } };
      double[][] rightFootPoints = new double[][] { { 0.2, -0.2 }, { -0.1, -0.2 }, { -0.1, -0.4 }, { 0.2, -0.4 } };

      FramePoint2d copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      double[][] expectedVirtualToePoints = new double[][] { { 0.0, 0.3 }, { 0.0, -0.3 } };

      testExpected(leftFootPoints, rightFootPoints, copDesired, expectedVirtualToePoints, upcomingSupportSide);
      testSensitivityToFootRotation(leftFootPoints, rightFootPoints, copDesired, upcomingSupportSide);

      copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.1, 0.0);
      expectedVirtualToePoints = new double[][] { { 0.1, 0.3 }, { 0.1, -0.3 } };
      testExpected(leftFootPoints, rightFootPoints, copDesired, expectedVirtualToePoints, upcomingSupportSide);
      testSensitivityToFootRotation(leftFootPoints, rightFootPoints, copDesired, upcomingSupportSide);

      copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.15, 0.15);
      expectedVirtualToePoints = new double[][] { { 0.15, 0.3 }, { 0.15, -0.3 } };
      testExpected(leftFootPoints, rightFootPoints, copDesired, expectedVirtualToePoints, upcomingSupportSide);
      testSensitivityToFootRotation(leftFootPoints, rightFootPoints, copDesired, upcomingSupportSide);

      FramePoint2d startCoPDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.15, 0.0);
      FramePoint2d endCoPDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.15, 0.15);
      
      RobotSide upcomingSupportFoot = RobotSide.LEFT;
      testSensitivityToCoPMotion(leftFootPoints, rightFootPoints, startCoPDesired, endCoPDesired, upcomingSupportFoot);

      copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 2.0, 0.49);
      expectedVirtualToePoints = new double[][] { { 0.5, 0.3 }, { 0.5, -0.3 } };
   }

   
   @Ignore 
   public void testFeetOverlapping()
   {
      //TODO: Fix so that works for overlapping feet. Doesn't right now because combining polygons only works for disjoint polygons.
      RobotSide upcomingSupportSide = null;
      
      double[][] leftFootPoints = new double[][] { { 0.2, 0.01 }, { -0.1, 0.01 }, { -0.1, 0.2 }, { 0.2, 0.2 } };
      double[][] rightFootPoints = new double[][] { { 0.2, -0.01 }, { -0.1, -0.01 }, { -0.1, -0.2 }, { 0.2, -0.2 } };

      FramePoint2d copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      
      testSensitivityToFootRotation(leftFootPoints, rightFootPoints, copDesired, upcomingSupportSide);
      
      Vector2d footTranslation = new Vector2d(-0.2, 0.0);
      testSensitivityToFootTranslation(leftFootPoints, rightFootPoints, copDesired, footTranslation, upcomingSupportSide);
   }

	@DeployableTestMethod(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testRotationWhenOnEdge()
   {
      RobotSide upcomingSupportSide = null;

      double[][] leftFootPoints = new double[][] { { 0.2, 0.2 }, { -0.05, 0.2 }, { -0.05, 0.35 }, { 0.2, 0.35 } };
      double[][] rightFootPoints = new double[][] { { 0.2, -0.2 }, { -0.05, -0.2 }, { -0.05, -0.35 }, { 0.2, -0.35 } };

      FramePoint2d copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.3, 0.0);
      testSensitivityToFootRotation(leftFootPoints, rightFootPoints, copDesired, upcomingSupportSide);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testTranslationIntoFoot()
   {
      RobotSide upcomingSupportSide = RobotSide.LEFT;

      double[][] leftFootPoints = new double[][] { { 0.2, 0.2 }, { -0.1, 0.2 }, { -0.1, 0.4 }, { 0.2, 0.4 } };
      double[][] rightFootPoints = new double[][] { { 0.2, -0.2 }, { -0.1, -0.2 }, { -0.1, -0.4 }, { 0.2, -0.4 } };

      FramePoint2d startCoPDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2d endCoPDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.15, 0.395);
      
      testSensitivityToCoPMotion(leftFootPoints, rightFootPoints, startCoPDesired, endCoPDesired, upcomingSupportSide);
   }

	@DeployableTestMethod(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void testTranslationOfFoot()
   {
      RobotSide upcomingSupportSide = RobotSide.LEFT;

      double[][] leftFootPoints = new double[][] { { 0.2, 0.2 }, { -0.1, 0.2 }, { -0.1, 0.4 }, { 0.2, 0.4 } };
      double[][] rightFootPoints = new double[][] { { 0.2, -0.2 }, { -0.1, -0.2 }, { -0.1, -0.4 }, { 0.2, -0.4 } };

      Vector2d footMotion = new Vector2d(0.3, 0.2);

      FramePoint2d copDesired = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.5, 0.0);
      testSensitivityToFootTranslation(leftFootPoints, rightFootPoints, copDesired, footMotion, upcomingSupportSide);
   }

   private void testExpected(double[][] leftFootPoints, double[][] rightFootPoints, FramePoint2d copDesired, double[][] expectedVirtualToePoints, RobotSide upcomingSupportSide)
   {
      if (ignoreProblems)
         return;
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges = createAndCombinePolygons(leftFootPoints, rightFootPoints);

      VirtualToePointCalculator virtualToePointCalculator = createVirtualToePointCalculator(debugViz, removeDebugVizEachTime);

      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, copDesired, supportPolygonAndEdges, upcomingSupportSide);

      assertVirtualToePointsAreEqual(expectedVirtualToePoints, virtualToePoints);
      assertVirtualToePointsAreInsideFeet(supportPolygonAndEdges, virtualToePoints);
      assertCoPAndVirtualToePointsAreInOrderAndColinear(copDesired, virtualToePoints);
   }
   
   private void testSensitivityToFootTranslation(double[][] leftFootPoints, double[][] rightFootPoints, FramePoint2d originalCopDesired,
         Vector2d footTranslation, RobotSide upcomingSupportSide)
   {
      double incrementDistance = 0.002;
      double incrementAlpha = incrementDistance / footTranslation.length();
      double startAlpha = -0.1;
      double endAlpha = 1.0;
      double sleepDuration = 0.01;
      double distanceOffEdge = 0.002;
      double vtpJumpPerTickMaxAllowed = 0.04;

      FrameConvexPolygon2d leftFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), leftFootPoints);
      FrameConvexPolygon2d translatedLeftFootPolygon = translateFootPolygon(leftFootPolygon, footTranslation, startAlpha);

      FrameConvexPolygon2d rightFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), rightFootPoints);
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges = createAndCombinePolygons(translatedLeftFootPolygon, rightFootPolygon);

      VirtualToePointCalculator virtualToePointCalculator = createVirtualToePointCalculator(debugViz, removeDebugVizEachTime);

      SideDependentList<FramePoint2d> originalVirtualToePoints = new SideDependentList<FramePoint2d>();

      FramePoint2d copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, supportPolygonAndEdges,
            distanceOffEdge);
      virtualToePointCalculator.packVirtualToePoints(originalVirtualToePoints, copDesired, supportPolygonAndEdges, upcomingSupportSide);

      do
      {
         SideDependentList<FramePoint2d> virtualToePoints = originalVirtualToePoints;

         for (double alpha = startAlpha; alpha < endAlpha; alpha = alpha + incrementAlpha)
         {
            if (debugViz)
               sleep(sleepDuration);

            translatedLeftFootPolygon = translateFootPolygon(leftFootPolygon, footTranslation, alpha);
            FrameConvexPolygon2dAndConnectingEdges newSupportPolygonAndEdges = createAndCombinePolygons(translatedLeftFootPolygon, rightFootPolygon);

            SideDependentList<FramePoint2d> newVirtualToePoints = new SideDependentList<FramePoint2d>();
            copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, newSupportPolygonAndEdges,
                  distanceOffEdge);

            virtualToePointCalculator.packVirtualToePoints(newVirtualToePoints, copDesired, newSupportPolygonAndEdges, upcomingSupportSide);

            boolean pointsAreClose = verifyVirtualToePointsAreClose(virtualToePoints, newVirtualToePoints, vtpJumpPerTickMaxAllowed);

            if (!ignoreProblems && !pointsAreClose)
            {
               VirtualToePointCalculator virtualToePointCalculatorToShowProblem = createVirtualToePointCalculator(true, false);

               SideDependentList<FramePoint2d> virtualToePointsToShowProblemOne = new SideDependentList<FramePoint2d>();
               SideDependentList<FramePoint2d> virtualToePointsToShowProblemTwo = new SideDependentList<FramePoint2d>();

               copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, supportPolygonAndEdges,
                     distanceOffEdge);
               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemOne, copDesired, supportPolygonAndEdges, upcomingSupportSide);

               copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, newSupportPolygonAndEdges,
                     distanceOffEdge);
               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemTwo, copDesired, newSupportPolygonAndEdges, upcomingSupportSide);

               System.out.println("Error in testing sensitivity to foot rotation!");
               System.err.println("alpha = " + alpha);
               System.err.println("supportPolygonAndEdges = " + supportPolygonAndEdges);
               System.err.println("virtualToePointsToShowProblemOne = " + virtualToePointsToShowProblemOne);
               System.err.println("newSupportPolygonAndEdges = " + newSupportPolygonAndEdges);
               System.err.println("virtualToePointsToShowProblemTwo = " + virtualToePointsToShowProblemTwo);

               sleepForever();
               fail();
            }

            virtualToePoints = newVirtualToePoints;
            supportPolygonAndEdges = newSupportPolygonAndEdges;
         }
      } while (repeatForever);

   }

   private void testSensitivityToFootRotation(double[][] leftFootPoints, double[][] rightFootPoints, FramePoint2d originalCopDesired, RobotSide upcomingSupportSide)
   {
      FrameConvexPolygon2d leftFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), leftFootPoints);

      double startAngle = -0.1;
      double endAngle = 2.0 * Math.PI + 0.1;
      double incrementAngle = 0.002;
      double sleepDuration = 0.001;
      double distanceOffEdge = 0.002;
      double vtpJumpPerTickMaxAllowed = 0.04;

      FrameConvexPolygon2d rotatedLeftFootPolygon = rotateFootPolygonAboutCentroid(leftFootPolygon, startAngle);

      FrameConvexPolygon2d rightFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), rightFootPoints);
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges = createAndCombinePolygons(rotatedLeftFootPolygon, rightFootPolygon);

      VirtualToePointCalculator virtualToePointCalculator = createVirtualToePointCalculator(debugViz, removeDebugVizEachTime);

      SideDependentList<FramePoint2d> originalVirtualToePoints = new SideDependentList<FramePoint2d>();

      FramePoint2d copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, supportPolygonAndEdges,
            distanceOffEdge);
      virtualToePointCalculator.packVirtualToePoints(originalVirtualToePoints, copDesired, supportPolygonAndEdges, upcomingSupportSide);

      do
      {
         SideDependentList<FramePoint2d> virtualToePoints = originalVirtualToePoints;

         for (double rotationAngle = startAngle; rotationAngle < endAngle; rotationAngle = rotationAngle + incrementAngle)
         {
            if (debugViz)
               sleep(sleepDuration);

            rotatedLeftFootPolygon = rotateFootPolygonAboutCentroid(leftFootPolygon, rotationAngle);
            FrameConvexPolygon2dAndConnectingEdges newSupportPolygonAndEdges = createAndCombinePolygons(rotatedLeftFootPolygon, rightFootPolygon);

            SideDependentList<FramePoint2d> newVirtualToePoints = new SideDependentList<FramePoint2d>();
            copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, newSupportPolygonAndEdges,
                  distanceOffEdge);

            virtualToePointCalculator.packVirtualToePoints(newVirtualToePoints, copDesired, newSupportPolygonAndEdges, upcomingSupportSide);

            boolean pointsAreClose = verifyVirtualToePointsAreClose(virtualToePoints, newVirtualToePoints, vtpJumpPerTickMaxAllowed);

            if (!ignoreProblems && !pointsAreClose)
            {
               VirtualToePointCalculator virtualToePointCalculatorToShowProblem = createVirtualToePointCalculator(true, false);

               SideDependentList<FramePoint2d> virtualToePointsToShowProblemOne = new SideDependentList<FramePoint2d>();
               SideDependentList<FramePoint2d> virtualToePointsToShowProblemTwo = new SideDependentList<FramePoint2d>();

               copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, supportPolygonAndEdges,
                     distanceOffEdge);
               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemOne, copDesired, supportPolygonAndEdges, upcomingSupportSide);

               copDesired = GeometricVirtualToePointCalculator.projectCoPIntoPolygonAndMoveItOffConnectingEdge(originalCopDesired, newSupportPolygonAndEdges,
                     distanceOffEdge);
               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemTwo, copDesired, newSupportPolygonAndEdges, upcomingSupportSide);

               System.out.println("Error in testing sensitivity to foot rotation!");
               System.err.println("rotationAngle = " + rotationAngle);
               System.err.println("supportPolygonAndEdges = " + supportPolygonAndEdges);
               System.err.println("virtualToePointsToShowProblemOne = " + virtualToePointsToShowProblemOne);
               System.err.println("newSupportPolygonAndEdges = " + newSupportPolygonAndEdges);
               System.err.println("virtualToePointsToShowProblemTwo = " + virtualToePointsToShowProblemTwo);

               sleepForever();
               fail();
            }

            virtualToePoints = newVirtualToePoints;
            supportPolygonAndEdges = newSupportPolygonAndEdges;
         }
      } while (repeatForever);

   }

   private void testSensitivityToCoPMotion(double[][] leftFootPoints, double[][] rightFootPoints, FramePoint2d copDesiredStart, FramePoint2d copDesiredEnd, RobotSide upcomingSupportSide)
   {
      double sleepDuration = 0.01;
      double incrementalDistance = 0.002;
      double vtpNearCopIfInsideUpcomingSupportFootTolerance = 0.04;
      
      FrameConvexPolygon2d leftFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), leftFootPoints);
      FrameConvexPolygon2d rightFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), rightFootPoints);
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges = createAndCombinePolygons(leftFootPolygon, rightFootPolygon);

      VirtualToePointCalculator virtualToePointCalculator = createVirtualToePointCalculator(debugViz, removeDebugVizEachTime);

      SideDependentList<FramePoint2d> originalVirtualToePoints = new SideDependentList<FramePoint2d>();

      double distanceOfCopPath = copDesiredStart.distance(copDesiredEnd);

      FramePoint2d copDesired = new FramePoint2d(copDesiredStart);
      virtualToePointCalculator.packVirtualToePoints(originalVirtualToePoints, copDesired, supportPolygonAndEdges, upcomingSupportSide);

      do
      {
         SideDependentList<FramePoint2d> virtualToePoints = originalVirtualToePoints;

         for (double distanceAlongPath = 0.0; distanceAlongPath < distanceOfCopPath; distanceAlongPath += incrementalDistance)
         {
            if (debugViz)
               sleep(sleepDuration);

            double alpha = distanceAlongPath / distanceOfCopPath;

//            FramePoint2d newCopDesired = FramePoint2d.morph(copDesiredStart, copDesiredEnd, alpha);
            FramePoint2d newCopDesired = new FramePoint2d();
            newCopDesired.interpolate(copDesiredStart, copDesiredEnd, alpha);


            SideDependentList<FramePoint2d> newVirtualToePoints = new SideDependentList<FramePoint2d>();

            virtualToePointCalculator.packVirtualToePoints(newVirtualToePoints, newCopDesired, supportPolygonAndEdges, upcomingSupportSide);

            double epsilon = 0.02;
            boolean pointsAreClose = verifyVirtualToePointsAreClose(virtualToePoints, newVirtualToePoints, epsilon);
            assertCoPAndVirtualToePointsAreInOrderAndColinear(newCopDesired, newVirtualToePoints);
            boolean okIfInsideFoot = verifyCoPIsNearVirtualToePointIfInsideFoot(leftFootPolygon, rightFootPolygon, newCopDesired, newVirtualToePoints, upcomingSupportSide, vtpNearCopIfInsideUpcomingSupportFootTolerance);      
            
            if (!ignoreProblems && (!pointsAreClose || !okIfInsideFoot))
            {
               VirtualToePointCalculator virtualToePointCalculatorToShowProblem = createVirtualToePointCalculator(true, false);

               SideDependentList<FramePoint2d> virtualToePointsToShowProblemOne = new SideDependentList<FramePoint2d>();
               SideDependentList<FramePoint2d> virtualToePointsToShowProblemTwo = new SideDependentList<FramePoint2d>();

               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemOne, copDesired, supportPolygonAndEdges, upcomingSupportSide);
               virtualToePointCalculatorToShowProblem.packVirtualToePoints(virtualToePointsToShowProblemTwo, newCopDesired, supportPolygonAndEdges, upcomingSupportSide);

               System.out.println("Error in testing sensitivity to CoP translation!");
               System.out.println("pointsAreClose = " + pointsAreClose + ", okIfInsideFoot = " + okIfInsideFoot);
               System.err.println("distanceAlongPath = " + distanceAlongPath);
               System.err.println("copDesired = " + copDesired);
               System.err.println("newCopDesired = " + newCopDesired);
               System.err.println("supportPolygonAndEdges = " + supportPolygonAndEdges);
               System.err.println("virtualToePointsToShowProblemOne = " + virtualToePointsToShowProblemOne);
               System.err.println("virtualToePointsToShowProblemTwo = " + virtualToePointsToShowProblemTwo);

               sleepForever();
               fail();
            }

            virtualToePoints = newVirtualToePoints;
            copDesired = newCopDesired;
         }
      } while (repeatForever);

   }

   private boolean verifyCoPIsNearVirtualToePointIfInsideFoot(FrameConvexPolygon2d leftFootPolygon, FrameConvexPolygon2d rightFootPolygon, FramePoint2d copDesired, SideDependentList<FramePoint2d> virtualToePoints,
         RobotSide upcomingSupportFoot, double vtpNearCopIfInsideUpcomingSupportFootTolerance)
   {
      if (upcomingSupportFoot == null) return true;
      
      FramePoint2d virtualToePoint = virtualToePoints.get(upcomingSupportFoot);
      
      FrameConvexPolygon2d upcomingSupportFootPolygon;
      if (upcomingSupportFoot == RobotSide.LEFT)
      {
         upcomingSupportFootPolygon = leftFootPolygon;
      }
      else
      {
         upcomingSupportFootPolygon = rightFootPolygon;
      }
      
      if (!upcomingSupportFootPolygon.isPointInside(copDesired)) return true;
      
      double distance = copDesired.distance(virtualToePoint);
      
      if (distance > vtpNearCopIfInsideUpcomingSupportFootTolerance) return false;
      return true;
   }

   private FrameConvexPolygon2d translateFootPolygon(FrameConvexPolygon2d footPolygon, Vector2d footTranslation, double alpha)
   {
      Vector3d translationVector = new Vector3d(footTranslation.getX(), footTranslation.getY(), 0.0);
      translationVector.scale(alpha);

      translate.setIdentity();
      translate.setTranslation(translationVector);
      
      FrameConvexPolygon2d translatedFootPolygon = new FrameConvexPolygon2d(footPolygon.getReferenceFrame());
      translatedFootPolygon.clear();
      translatedFootPolygon.addVertices(footPolygon);
      translatedFootPolygon.applyTransform(translate);
      translatedFootPolygon.update();
      return translatedFootPolygon;
   }

   private final RigidBodyTransform translate = new RigidBodyTransform();
   private final RigidBodyTransform rotate = new RigidBodyTransform();
   private final RigidBodyTransform undoTranslate = new RigidBodyTransform();
   private final RigidBodyTransform allThree = new RigidBodyTransform();

   private FrameConvexPolygon2d rotateFootPolygonAboutCentroid(FrameConvexPolygon2d footPolygon, double rotationAngle)
   {
      FramePoint2d centroid = footPolygon.getCentroidCopy();
      Vector3d translationVector = new Vector3d(centroid.getX(), centroid.getY(), 0.0);
      translate.setIdentity();
      translate.setTranslation(translationVector);

      rotate.setIdentity();
      rotate.rotZ(rotationAngle);

      Vector3d undoTranslationVector = new Vector3d(translationVector);
      undoTranslationVector.scale(-1.0);
      undoTranslate.setIdentity();
      undoTranslate.setTranslation(undoTranslationVector);

      allThree.set(translate);
      allThree.multiply(rotate);
      allThree.multiply(undoTranslate);

      FrameConvexPolygon2d rotatedLeftFootPolygon = new FrameConvexPolygon2d(footPolygon.getReferenceFrame());
      rotatedLeftFootPolygon.clear();
      rotatedLeftFootPolygon.addVertices(footPolygon);
      rotatedLeftFootPolygon.applyTransform(allThree);
      rotatedLeftFootPolygon.update();
      return rotatedLeftFootPolygon;
   }

   private void sleep(double sleepSeconds)
   {
      try
      {
         Thread.sleep((long) (1000 * sleepSeconds));
      } catch (InterruptedException e)
      {
      }

   }

   private boolean verifyVirtualToePointsAreClose(SideDependentList<FramePoint2d> virtualToePoints, SideDependentList<FramePoint2d> newVirtualToePoints,
         double epsilon)
   {
      boolean ret = true;

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d virtualToePoint = virtualToePoints.get(robotSide);
         FramePoint2d newVirtualToePoint = newVirtualToePoints.get(robotSide);

         ret = ret & verifyVirtualToePointsAreClose(virtualToePoint, newVirtualToePoint, epsilon);
      }

      return ret;

   }

   private boolean verifyVirtualToePointsAreClose(FramePoint2d virtualToePoint, FramePoint2d newVirtualToePoint, double epsilon)
   {
      double distance = virtualToePoint.distance(newVirtualToePoint);
      return distance <= epsilon;
   }

   private void sleepForever()
   {
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         } catch (InterruptedException e)
         {
         }
      }

   }

   private FrameConvexPolygon2dAndConnectingEdges createAndCombinePolygons(double[][] leftFootPoints, double[][] rightFootPoints)
   {
      FrameConvexPolygon2d leftFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), leftFootPoints);
      FrameConvexPolygon2d rightFootPolygon = new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), rightFootPoints);

      return createAndCombinePolygons(leftFootPolygon, rightFootPolygon);
   }

   private FrameConvexPolygon2dAndConnectingEdges createAndCombinePolygons(FrameConvexPolygon2d leftFootPolygon, FrameConvexPolygon2d rightFootPolygon)
   {
      FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges = ConvexPolygonTools.combineDisjointPolygons(leftFootPolygon, rightFootPolygon);

      if (supportPolygonAndEdges == null) throw new RuntimeException("supportPolygonAndEdges == null. Most likely that the feet are overlapping!");
      return supportPolygonAndEdges;
   }

   private VirtualToePointCalculator createVirtualToePointCalculator(boolean debugViz, boolean removeDebugVizEachTime)
   {
      YoVariableRegistry registry = new YoVariableRegistry("parent");
      YoGraphicsListRegistry yoGraphicsListRegistry = null;

      GeometricVirtualToePointCalculator geometricVirtualToePointCalculator = new GeometricVirtualToePointCalculator(registry,
            yoGraphicsListRegistry);
      geometricVirtualToePointCalculator.setAllFramesToComputeInToWorld();
      
      setParametersForGeometricVirtualToePointCalculator(geometricVirtualToePointCalculator);
      
      geometricVirtualToePointCalculator.setupForDebugViz(debugViz, removeDebugVizEachTime);
      return geometricVirtualToePointCalculator;
   }
   
   protected void setParametersForGeometricVirtualToePointCalculator(GeometricVirtualToePointCalculator geometricVirtualToePointCalculator)
   {
      geometricVirtualToePointCalculator.setDefaultParameters();
   }

   private void assertCoPAndVirtualToePointsAreInOrderAndColinear(FramePoint2d copDesired, SideDependentList<FramePoint2d> actualVirtualToePoints)
   {
      double epsilon = 1e-7;

      FramePoint2d leftVirtualToePoint = actualVirtualToePoints.get(RobotSide.LEFT);
      FramePoint2d rightVirtualToePoint = actualVirtualToePoints.get(RobotSide.RIGHT);

      boolean inOrderColinear = GeometryTools.arePointsInOrderAndColinear(leftVirtualToePoint.getPoint(), copDesired.getPoint(), rightVirtualToePoint.getPoint(),
            epsilon);

      assertTrue(inOrderColinear);
   }

   private void assertVirtualToePointsAreInsideFeet(FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges,
         SideDependentList<FramePoint2d> virtualToePoints)
   {
      SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>(supportPolygonAndEdges.getOriginalPolygon1(), supportPolygonAndEdges.getOriginalPolygon2());
      assertVirtualToePointsAreInsideFeet(footPolygons, virtualToePoints);
   }

   private void assertVirtualToePointsAreInsideFeet(SideDependentList<FrameConvexPolygon2d> footPolygons, SideDependentList<FramePoint2d> virtualToePoints)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d footPolygon = footPolygons.get(robotSide);
         FramePoint2d virtualToePoint = virtualToePoints.get(robotSide);

         assertTrue(footPolygon.isPointInside(virtualToePoint));
      }

   }

   private void assertVirtualToePointsAreEqual(double[][] expectedVirtualToePoints, SideDependentList<FramePoint2d> actualVirtualToePoints)
   {
      FramePoint2d leftVirtualToePoint = actualVirtualToePoints.get(RobotSide.LEFT);
      FramePoint2d rightVirtualToePoint = actualVirtualToePoints.get(RobotSide.RIGHT);

      double epsilon = 1e-7;

      assertEquals(expectedVirtualToePoints[0][0], leftVirtualToePoint.getX(), epsilon);
      assertEquals(expectedVirtualToePoints[0][1], leftVirtualToePoint.getY(), epsilon);
      assertEquals(expectedVirtualToePoints[1][0], rightVirtualToePoint.getX(), epsilon);
      assertEquals(expectedVirtualToePoints[1][1], rightVirtualToePoint.getY(), epsilon);
   }

}
