package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

import java.awt.*;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

public class SpeedUpExampleGraphic
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SimulationConstructionSet scs;

   private final DoubleYoVariable speedUpTime;
   private final DoubleYoVariable omega0;
   private final DoubleYoVariable segmentTime;
   private final DoubleYoVariable timeInSegment;

   private final DoubleYoVariable currentTime;

   private final YoFramePoint2d stanceCMP;
   private final YoFramePoint2d endICP;
   private final YoFramePoint2d desiredICP;
   private final YoFramePoint2d currentICP;
   private final YoFramePoint2d projectedICP;

   private final FramePoint2d desiredICP2d = new FramePoint2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();
   private final FramePoint2d referenceCMP = new FramePoint2d();

   private static final double forwardCMPOffset = 0.075;
   private static final double backwardCMPOffset = 0.02;
   private static final double stepLength = 0.30;
   private static final double stepWidth = 0.25;

   public SpeedUpExampleGraphic()
   {
      Robot robot = new DummyRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();

      currentTime = robot.getYoTime();
      speedUpTime = new DoubleYoVariable("speedUpTime", registry);
      omega0 = new DoubleYoVariable("omega0", registry);
      omega0.set(3.0);
      segmentTime = new DoubleYoVariable("segmentTime", registry);
      segmentTime.set(0.5);
      timeInSegment = new DoubleYoVariable("timeInSegment", registry);

      stanceCMP = new YoFramePoint2d("stanceCMP", worldFrame, registry);
      endICP = new YoFramePoint2d("endICP", worldFrame, registry);
      desiredICP = new YoFramePoint2d("desiredICP", worldFrame, registry);
      currentICP = new YoFramePoint2d("currentICP", worldFrame, registry);
      projectedICP = new YoFramePoint2d("projectedICP", worldFrame, registry);

      YoFrameConvexPolygon2d stanceFootPolygonViz = new YoFrameConvexPolygon2d("stanceFootPolygon", "", worldFrame, 4, registry);
      YoFrameConvexPolygon2d stepFootPolygonViz = new YoFrameConvexPolygon2d("stepFootPolygon", "", worldFrame, 4, registry);

      FrameConvexPolygon2d stanceFootPolygon = new FrameConvexPolygon2d();
      stanceFootPolygon.addVertex(new FramePoint2d(worldFrame, 0.11, 0.0425));
      stanceFootPolygon.addVertex(new FramePoint2d(worldFrame, 0.11, -0.0425));
      stanceFootPolygon.addVertex(new FramePoint2d(worldFrame, -0.11, -0.055));
      stanceFootPolygon.addVertex(new FramePoint2d(worldFrame, -0.11, 0.055));
      stanceFootPolygon.update();

      FrameConvexPolygon2d stepFootPolygon = new FrameConvexPolygon2d();
      stepFootPolygon.addVertex(new FramePoint2d(worldFrame, 0.11 + stepLength, 0.0425 + stepWidth));
      stepFootPolygon.addVertex(new FramePoint2d(worldFrame, 0.11 + stepLength, -0.0425 + stepWidth));
      stepFootPolygon.addVertex(new FramePoint2d(worldFrame, -0.11 + stepLength, -0.055 + stepWidth));
      stepFootPolygon.addVertex(new FramePoint2d(worldFrame, -0.11 + stepLength, 0.055 + stepWidth));
      stepFootPolygon.update();

      stanceFootPolygonViz.setFrameConvexPolygon2d(stanceFootPolygon);
      stepFootPolygonViz.setFrameConvexPolygon2d(stepFootPolygon);

      ArtifactList artifactList = new ArtifactList("");
      YoArtifactPolygon stanceFootPolygonArtifact = new YoArtifactPolygon("Stance Foot Polygon", stanceFootPolygonViz, defaultFeetColors.get(RobotSide.LEFT), false);
      YoArtifactPolygon stepFootPolygonArtifact = new YoArtifactPolygon("Step Foot Polygon", stepFootPolygonViz, defaultFeetColors.get(RobotSide.RIGHT), false);
      artifactList.add(stanceFootPolygonArtifact);
      artifactList.add(stepFootPolygonArtifact);
      yoGraphicsListRegistry.registerArtifactList(artifactList);



      FramePoint2d heelStrikeCMP = new FramePoint2d(worldFrame, stepLength - backwardCMPOffset, stepWidth);
      stanceCMP.set(forwardCMPOffset, 0);
      stanceCMP.getFrameTuple2d(referenceCMP);

      CapturePointTools.computeCapturePointPosition(omega0.getDoubleValue(), -0.05, heelStrikeCMP, referenceCMP, finalICP2d);
      endICP.set(finalICP2d);

      CapturePointTools.computeCapturePointPosition(omega0.getDoubleValue(), -segmentTime.getDoubleValue(), finalICP2d, referenceCMP, desiredICP2d);
      desiredICP.set(desiredICP2d);

      YoGraphicPosition stanceCMPGraphic = new YoGraphicPosition("stanceCMP", stanceCMP, 0.015, YoAppearance.Green(), GraphicType.BALL);
      YoGraphicPosition endICPGraphic = new YoGraphicPosition("endICP", endICP, 0.015, YoAppearance.DarkRed(), GraphicType.BALL);
      YoGraphicPosition desiredICPGraphic = new YoGraphicPosition("desiredICP", desiredICP, 0.02, YoAppearance.LightBlue(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition currentICPGraphic = new YoGraphicPosition("currentICP", currentICP, 0.025, YoAppearance.DarkBlue(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition projectedICPGraphic = new YoGraphicPosition("projectedICP", projectedICP, 0.015, YoAppearance.Yellow(), GraphicType.SOLID_BALL);

      YoArtifactLineSegment2d icpDynamicsArtifact = new YoArtifactLineSegment2d("icpDynamics", desiredICP, endICP, Color.BLUE);

      String name = "SpeedUpGraphic";
      yoGraphicsListRegistry.registerArtifact(name, icpDynamicsArtifact);
      yoGraphicsListRegistry.registerArtifact(name, stanceCMPGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, endICPGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, currentICPGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, desiredICPGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, projectedICPGraphic.createArtifact());
      yoGraphicsListRegistry.registerYoGraphic(name, stanceCMPGraphic);
      yoGraphicsListRegistry.registerYoGraphic(name, endICPGraphic);
      yoGraphicsListRegistry.registerYoGraphic(name, currentICPGraphic);
      yoGraphicsListRegistry.registerYoGraphic(name, desiredICPGraphic);
      yoGraphicsListRegistry.registerYoGraphic(name, projectedICPGraphic);

      scs = new SimulationConstructionSet(robot);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      Thread myThread = new Thread(scs);
      myThread.start();
   }

   private void estimateTimeRemainingForStateUnderDisturbance()
   {
      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(currentICP.getFrameTuple2d());
      speedUpTime.set(deltaTimeToBeAccounted);
   }

   private static final double segmentDuration = 20.0;
   private double initialTime = 0.0;
   private int segmentNumber = 0;
   private final Vector2dZUpFrame actionFrame = new Vector2dZUpFrame("actionFrame", worldFrame);
   private void updateCurrentICPLocation()
   {
      if (timeInSegment.getDoubleValue() > segmentDuration)
      {
         initialTime = currentTime.getDoubleValue();
         segmentNumber++;
         if (segmentNumber > 3)
            segmentNumber = 0;
      }

      timeInSegment.set(currentTime.getDoubleValue() - initialTime);

      FramePoint2d firstKnot = new FramePoint2d();

      FrameVector2d xAxis = new FrameVector2d(worldFrame);
      endICP.getFrameTuple2d(finalICP2d);
      desiredICP.getFrameTuple2d(desiredICP2d);
      xAxis.set(finalICP2d);
      xAxis.sub(desiredICP2d);
      actionFrame.setXAxis(xAxis);
      actionFrame.update();

      FramePoint2d secondKnot = new FramePoint2d(actionFrame);
      FramePoint2d thirdKnot = new FramePoint2d(actionFrame);
      FramePoint2d fourthKnot = new FramePoint2d(actionFrame);
      secondKnot.set(0.2, 0.1);
      thirdKnot.set(0.2, -0.05);
      fourthKnot.set(0.0, -0.15);
      secondKnot.changeFrame(worldFrame);
      thirdKnot.changeFrame(worldFrame);
      fourthKnot.changeFrame(worldFrame);

      desiredICP.getFrameTuple2d(firstKnot);

      if (segmentNumber == 0)
      {
         currentICP.set(firstKnot);
         secondKnot.scale(timeInSegment.getDoubleValue() / segmentDuration);
         currentICP.add(secondKnot);
      }
      else if (segmentNumber == 1)
      {
         currentICP.set(firstKnot);
         currentICP.add(secondKnot);

         thirdKnot.sub(secondKnot);
         thirdKnot.scale(timeInSegment.getDoubleValue() / segmentDuration);
         currentICP.add(thirdKnot);
      }
      else if (segmentNumber == 2)
      {
         currentICP.set(firstKnot);
         currentICP.add(thirdKnot);

         fourthKnot.sub(thirdKnot);
         fourthKnot.scale(timeInSegment.getDoubleValue() / segmentDuration);
         currentICP.add(fourthKnot);
      }
      else if (segmentNumber == 3)
      {
         currentICP.set(firstKnot);
         fourthKnot.scale(1 - timeInSegment.getDoubleValue() / segmentDuration);
         currentICP.add(fourthKnot);
      }
   }

   private final FrameLine2d desiredICPToFinalICPLine = new FrameLine2d();
   private final FrameLineSegment2d desiredICPToFinalICPLineSegment = new FrameLineSegment2d();
   private final FramePoint2d actualICP2d = new FramePoint2d();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2d actualCapturePointPosition)
   {
      stanceCMP.getFrameTuple2dIncludingFrame(referenceCMP);
      desiredICP.getFrameTuple2dIncludingFrame(desiredICP2d);
      endICP.getFrameTuple2dIncludingFrame(finalICP2d);

      if (desiredICP2d.distance(finalICP2d) < 1.0e-10)
         return Double.NaN;

      desiredICPToFinalICPLineSegment.set(desiredICP2d, finalICP2d);
      actualICP2d.setIncludingFrame(actualCapturePointPosition);
      double percentAlongLineSegmentICP = desiredICPToFinalICPLineSegment.percentageAlongLineSegment(actualICP2d);
      if (percentAlongLineSegmentICP < 0.0)
      {
         desiredICPToFinalICPLine.set(desiredICP2d, finalICP2d);
         desiredICPToFinalICPLine.orthogonalProjection(actualICP2d);
      }
      else
      {
         desiredICPToFinalICPLineSegment.orthogonalProjection(actualICP2d);
      }

      double actualDistanceDueToDisturbance = referenceCMP.distance(actualICP2d);
      double expectedDistanceAccordingToPlan = referenceCMP.distance(desiredICP2d);

      //computeTimeInCurrentState(time);
      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
      {
         projectedICP.set(desiredICP2d);
         return 0.0;
      }
      else
      {
         projectedICP.set(actualICP2d);
         return Math.log(distanceRatio) / omega0.getDoubleValue();
      }
   }

   public static void main(String[] args)
   {
      SpeedUpExampleGraphic speedUpExampleGraphic = new SpeedUpExampleGraphic();
   }

   private class DummyRobot extends Robot
   {
      public DummyRobot()
      {
         super("dummyRobot");
      }

      @Override
      public void update()
      {
         super.update();
         updateCurrentICPLocation();
         estimateTimeRemainingForStateUnderDisturbance();
      }
   }

   private static class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumns(x, y, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }
}
