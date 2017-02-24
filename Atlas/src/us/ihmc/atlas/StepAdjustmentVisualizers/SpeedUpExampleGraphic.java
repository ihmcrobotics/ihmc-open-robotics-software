package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class SpeedUpExampleGraphic
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SimulationConstructionSet scs;

   private final DoubleYoVariable speedUpTime;
   private final DoubleYoVariable omega0;
   private final DoubleYoVariable segmentTime;

   private final YoFramePoint2d stanceCMP;
   private final YoFramePoint2d endICP;
   private final YoFramePoint2d desiredICP;
   private final YoFramePoint2d currentICP;
   private final YoFramePoint2d projectedICP;

   private final FramePoint2d desiredICP2d = new FramePoint2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();
   private final FramePoint2d referenceCMP = new FramePoint2d();

   public SpeedUpExampleGraphic()
   {
      Robot robot = new DummyRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();

      speedUpTime = new DoubleYoVariable("speedUpTime", registry);
      omega0 = new DoubleYoVariable("omega0", registry);
      omega0.set(3.0);
      segmentTime = new DoubleYoVariable("segmentTime", registry);
      segmentTime.set(0.5);

      stanceCMP = new YoFramePoint2d("stanceCMP", worldFrame, registry);
      endICP = new YoFramePoint2d("endICP", worldFrame, registry);
      desiredICP = new YoFramePoint2d("desiredICP", worldFrame, registry);
      currentICP = new YoFramePoint2d("currentICP", worldFrame, registry);
      projectedICP = new YoFramePoint2d("projectedICP", worldFrame, registry);

      double forwardCMPOffset = 0.075;
      double backwardCMPOffset = 0.02;
      double stepLength = 0.30;
      double stepWidth = 0.25;

      FramePoint2d heelStrikeCMP = new FramePoint2d(worldFrame, stepLength - backwardCMPOffset, stepWidth);
      stanceCMP.set(forwardCMPOffset, 0);
      endICP.set(heelStrikeCMP);

      endICP.getFrameTuple2d(finalICP2d);
      stanceCMP.getFrameTuple2d(referenceCMP);

      CapturePointTools.computeCapturePointPosition(omega0.getDoubleValue(), -segmentTime.getDoubleValue(), finalICP2d, referenceCMP, desiredICP2d);
      desiredICP.set(desiredICP2d);

      YoGraphicPosition stanceCMPGraphic = new YoGraphicPosition("stanceCMP", stanceCMP, 0.005, YoAppearance.Green(), GraphicType.BALL);
      YoGraphicPosition endICPGraphic = new YoGraphicPosition("endICP", endICP, 0.005, YoAppearance.DarkRed(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition desiredICPGraphic = new YoGraphicPosition("desiredICP", desiredICP, 0.005, YoAppearance.LightBlue(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition currentICPGraphic = new YoGraphicPosition("currentICP", currentICP, 0.005, YoAppearance.DarkBlue(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition projectedICPGraphic = new YoGraphicPosition("projectedICP", projectedICP, 0.005, YoAppearance.Yellow(), GraphicType.SOLID_BALL);

      String name = "SpeedUpGraphic";
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

   // from ICP Planner
   public void estimateTimeRemainingForStateUnderDisturbance()
   {
      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(currentICP.getFrameTuple2d());
      speedUpTime.set(deltaTimeToBeAccounted);
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
         estimateTimeRemainingForStateUnderDisturbance();
      }
   }
}
