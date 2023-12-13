package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class HeuristicICPControllerEvaluator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry("ICPControllerEvaluator");
   private final ICPControllerInterface controller;
   private final ICPControllerTestVisualizer visualizer;

   public HeuristicICPControllerEvaluator(double kpParallel, double kpOrthogonal, int bufferSize)
   {
      HeuristicICPControllerTest.TestICPControllerParameters icpControllerParameters = HeuristicICPControllerTest.createTestICPControllerParameters(kpParallel,
                                                                                                                                                    kpOrthogonal);

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      controller = HeuristicICPControllerTest.createICPController(icpControllerParameters, controlDT, registry, yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      visualizer = new ICPControllerTestVisualizer(bufferSize, registry, yoGraphicsListRegistry);
   }

   public void visualizeOverGridMovingICP(ICPControllerTestCase testCase,
                                          double xLowerBound,
                                          double xUpperBound,
                                          double yLowerBound,
                                          double yUpperBound,
                                          double yIncrement,
                                          double xIncrement)
   {
      boolean incrementY = true;
      double y = yLowerBound;

      for (double x = xLowerBound; x < xUpperBound; x = x + xIncrement)
      {
         while (true)
         {
            testCase = new ICPControllerTestCase(testCase);

            FramePoint2D currentICP = new FramePoint2D(worldFrame, x, y);
            testCase.setCurrentICP(currentICP);

            solveAndVisualize(controller, visualizer, testCase);

            if (incrementY)
            {
               y = y + yIncrement;
               if (y >= yUpperBound)
               {
                  incrementY = !incrementY;
                  break;
               }
            }
            else
            {
               y = y - yIncrement;
               if (y <= yLowerBound)
               {
                  incrementY = !incrementY;
                  break;
               }
            }
         }
      }
   }

   private void solveAndVisualize(ICPControllerInterface controller, ICPControllerTestVisualizer visualizer, ArrayList<ICPControllerTestCase> testCases)
   {
      for (int i = 0; i < testCases.size(); i++)
      {
         ICPControllerTestCase testCase = testCases.get(i);
         solveAndVisualize(controller, visualizer, testCase);
      }
   }

   private void solveAndVisualize(ICPControllerInterface controller, ICPControllerTestVisualizer visualizer, ICPControllerTestCase testCase)
   {
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = testCase.getSupportPolygonInWorld();
      double omega = testCase.getOmega();

      FramePoint2DReadOnly desiredICP = testCase.getDesiredICP();
      FramePoint2DReadOnly perfectCoP = testCase.getPerfectCoP();
      FrameVector2DReadOnly perfectCMPOffset = testCase.getPerfectCMPOffset();
      FramePoint2DReadOnly currentICP = testCase.getCurrentICP();

      FramePoint2DReadOnly currentCoMPosition = testCase.getCurrentCoMPosition();
      FrameVector2DReadOnly desiredICPVelocity = testCase.getDesiredICPVelocity();

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCoP, perfectCMPOffset, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(controller.getDesiredCMP());
      FramePoint2D desiredCoP = new FramePoint2D(controller.getDesiredCoP());

      testCase.setDesiredCMP(desiredCMP);
      testCase.setDesiredCoP(desiredCoP);

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);
      testCase.setExpectedControlICPVelocity(expectedControlICPVelocity);

      FramePoint2DBasics expectedICPMeetupPoint = computeExpectedICPMeetupPoint(desiredICP, currentICP, desiredICPVelocity, expectedControlICPVelocity);

      visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
      visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity, expectedICPMeetupPoint);
   }

   private static FramePoint2DBasics computeExpectedICPMeetupPoint(FramePoint2DReadOnly desiredICP,
                                                                   FramePoint2DReadOnly currentICP,
                                                                   FrameVector2DReadOnly desiredICPVelocity,
                                                                   FrameVector2D expectedControlICPVelocity)
   {
      FramePoint2DBasics expectedICPMeetupPoint = new FramePoint2D(desiredICP.getReferenceFrame());

      FrameLine2D desiredICPLine = new FrameLine2D(desiredICP, desiredICPVelocity);
      FrameLine2D expectedICPLine = new FrameLine2D(currentICP, expectedControlICPVelocity);

      boolean intersectionExists = desiredICPLine.intersectionWith(expectedICPLine, expectedICPMeetupPoint);
      if (!intersectionExists)
      {
         expectedICPMeetupPoint.setToNaN();
         return expectedICPMeetupPoint;
      }

      FrameVector2D desiredToMeetup = new FrameVector2D(expectedICPMeetupPoint);
      desiredToMeetup.sub(desiredICP);

      if (desiredToMeetup.length() < 0.002)
         return expectedICPMeetupPoint;

      if (desiredICPVelocity.dot(desiredToMeetup) < 0.0)
      {
         expectedICPMeetupPoint.setToNaN();
      }

      return expectedICPMeetupPoint;
   }

   public static void computeDesiredICPVelocityFromPerfectCMP(double omega,
                                                              FramePoint2DReadOnly desiredICP,
                                                              FramePoint2DReadOnly perfectCMP,
                                                              FrameVector2D desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.set(desiredICP);
      desiredICPVelocityToPack.sub(perfectCMP);
      desiredICPVelocityToPack.scale(omega);
   }

   private static FrameConvexPolygon2D createSupportPolygonFromFootWidthAndLength(double footLength, double footWidth)
   {
      FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D(worldFrame);
      supportPolygonInWorld.addVertex(-footLength / 2.0, -footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, -footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, footWidth / 2.0);
      supportPolygonInWorld.addVertex(-footLength / 2.0, footWidth / 2.0);
      supportPolygonInWorld.update();
      return supportPolygonInWorld;
   }

   private static FrameConvexPolygon2D createSupportPolygonFromFootWidthLengthAndStanceWidth(double footLength, double footWidth, double stanceWidth)
   {
      FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D(worldFrame);
      supportPolygonInWorld.addVertex(-footLength / 2.0, stanceWidth / 2.0 - footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, stanceWidth / 2.0 - footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, stanceWidth / 2.0 + footWidth / 2.0);
      supportPolygonInWorld.addVertex(-footLength / 2.0, stanceWidth / 2.0 + footWidth / 2.0);

      supportPolygonInWorld.addVertex(-footLength / 2.0, -stanceWidth / 2.0 - footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, -stanceWidth / 2.0 - footWidth / 2.0);
      supportPolygonInWorld.addVertex(footLength / 2.0, -stanceWidth / 2.0 + footWidth / 2.0);
      supportPolygonInWorld.addVertex(-footLength / 2.0, -stanceWidth / 2.0 + footWidth / 2.0);
      supportPolygonInWorld.update();
      return supportPolygonInWorld;
   }

   private void cropBufferAndSleepForever()
   {
      visualizer.cropBuffer();
      ThreadTools.sleepForever();
   }

   private static ICPControllerTestCase createEvaluationScenarioOne()
   {
      double footLength = 0.25;
      double footWidth = 0.1;

      FrameConvexPolygon2D supportPolygonInWorld = createSupportPolygonFromFootWidthAndLength(footLength, footWidth);

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.15, 0.0);
      FramePoint2D perfectCoP = new FramePoint2D(worldFrame, -0.25, 0.0);
      FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, 0.0, 0.0);

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, -0.02, 0.0);

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      testCase.setOmega(omega);
      testCase.setSupportPolygonInWorld(supportPolygonInWorld);
      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentCoMPosition(currentCoMPosition);
      return testCase;
   }

   public void visualizeRandom() throws Exception
   {
      double omega = 3.0;

      ArrayList<ICPControllerTestCase> testCases = new ArrayList<>();

      int numberOfTests = 300;
      Random random = new Random(1776L);
      //      Random random = new Random();

      for (int i = 0; i < numberOfTests; i++)
      {
         ICPControllerTestCase testCase = new ICPControllerTestCase();
         testCase.setOmega(omega);

         FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame());
         supportPolygonInWorld.addVertex(-0.3, -0.2);
         supportPolygonInWorld.addVertex(0.3, 0.05);
         supportPolygonInWorld.addVertex(0.35, 0.15);
         supportPolygonInWorld.addVertex(-0.15, 0.1);
         supportPolygonInWorld.update();

         FramePoint2D desiredICP = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.2));
         FramePoint2D perfectCoP = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.06));
         FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, EuclidCoreRandomTools.nextVector2D(random));
         perfectCMPOffset.scale(0.04);

         FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
         perfectCMP.add(perfectCMPOffset);

         testCase.setPerfectCMPOffset(perfectCMPOffset);

         FramePoint2D currentICP = new FramePoint2D(desiredICP);
         FrameVector2D icpError = new FrameVector2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.1));
         currentICP.add(icpError);

         FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.2));

         FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

         testCase.setSupportPolygonInWorld(supportPolygonInWorld);
         testCase.setDesiredICP(desiredICP);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         testCase.setPerfectCoP(perfectCoP);
         testCase.setCurrentICP(currentICP);
         testCase.setCurrentCoMPosition(currentCoMPosition);

         testCases.add(testCase);
      }

      solveAndVisualize(controller, visualizer, testCases);

      for (ICPControllerTestCase testCase : testCases)
      {
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = testCase.getSupportPolygonInWorld();
         FramePoint2DReadOnly desiredCMP = testCase.getDesiredCMP();
         FramePoint2DReadOnly desiredCoP = testCase.getDesiredCoP();

         double copDistanceToPolygon = supportPolygonInWorld.signedDistance(desiredCoP);

         //         assertTrue(copDistanceToPolygon < 1e-7);

         FramePoint2DReadOnly currentICP = testCase.getCurrentICP();
         FramePoint2DReadOnly desiredICP = testCase.getDesiredICP();

         FrameVector2D icpError = new FrameVector2D(desiredICP);
         icpError.sub(currentICP);

         FrameVector2DReadOnly expectedControlICPVelocity = testCase.getExpectedControlICPVelocity();

         //TODO: Figure out what we can assert about the output of the ICP Controller...
         double dotProduct = expectedControlICPVelocity.dot(icpError);

         //         if (copDistanceToPolygon < -0.002)
         //         {
         //            System.out.println("icpError = " + icpError);
         //            System.out.println("expectedControlICPVelocity = " + expectedControlICPVelocity);
         //            System.out.println("dotProduct = " + dotProduct);
         //
         //            assertTrue(dotProduct > 0.0);
         //         }
      }
   }

   public void visualizeCasesOfInterest() throws Exception
   {
      double footLength = 0.25;
      double footWidth = 0.1;
      double stanceWidth = 0.35;

      FrameConvexPolygon2D supportPolygonInWorld = createSupportPolygonFromFootWidthLengthAndStanceWidth(footLength, footWidth, stanceWidth);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      double omega = 3.0;
      testCase.setOmega(omega);

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.04, 0.06);
      FramePoint2D perfectCoP = new FramePoint2D(worldFrame, 0.0, 0.06); //0.01, 0.01);
      FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, 0.0, 0.0);

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      FramePoint2D currentICP = new FramePoint2D(desiredICP);
      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.0, 0.0);
      currentICP.add(icpError);

      FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, -0.02, 0.0);

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      testCase.setSupportPolygonInWorld(supportPolygonInWorld);
      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentICP(currentICP);
      testCase.setCurrentCoMPosition(currentCoMPosition);

      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         desiredICP.add(0.01, 0.0);
         testCase.setDesiredICP(desiredICP);

         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);

         solveAndVisualize(controller, visualizer, testCase);
      }

      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.04, 0.06);
      testCase.setDesiredICP(desiredICP);

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(controller, visualizer, testCase);
      }

      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.1, 0.1);
      testCase.setDesiredICP(desiredICP);
      currentICP.set(0.1, 0.1);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(controller, visualizer, testCase);
      }
   }

   public static void main(String[] args) throws Exception
   {
      double kpParallel = 1.0;
      double kpOrthogonal = 2.0;
      int bufferSize = 62000;
      HeuristicICPControllerEvaluator evaluator = new HeuristicICPControllerEvaluator(kpParallel, kpOrthogonal, bufferSize);

      evaluator.visualizeCasesOfInterest();
      evaluator.visualizeRandom();

      ICPControllerTestCase testCase = createEvaluationScenarioOne();

      double xLowerBound = -0.2;
      double xUpperBound = 0.4;
      double yLowerBound = -0.2;
      double yUpperBound = 0.2;
      double yIncrement = 0.002;
      double xIncrement = 0.002;

      evaluator.visualizeOverGridMovingICP(testCase, xLowerBound, xUpperBound, yLowerBound, yUpperBound, yIncrement, xIncrement);
      evaluator.cropBufferAndSleepForever();
   }

}
