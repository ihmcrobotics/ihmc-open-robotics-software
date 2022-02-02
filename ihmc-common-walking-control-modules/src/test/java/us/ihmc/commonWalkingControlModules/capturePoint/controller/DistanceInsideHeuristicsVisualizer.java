package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

public class DistanceInsideHeuristicsVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final SimulationOverheadPlotter plotter;

   private final YoFramePoint2D yoCapturePoint;
   private final YoFramePoint2D unconstrainedCMP;
   private final YoFramePoint2D copOfLeastEffort;

   private final YoFrameConvexPolygon2D supportPolygonForViz;
   private final YoFrameConvexPolygon2D cmpAllowableRegionViz;
   private final YoFrameConvexPolygon2D copAllowableRegionViz;

   private final ConvexPolygonScaler scaler = new ConvexPolygonScaler();

   public DistanceInsideHeuristicsVisualizer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      supportPolygonForViz = new YoFrameConvexPolygon2D("supportPolygon", worldFrame, 20, registry);
      cmpAllowableRegionViz = new YoFrameConvexPolygon2D("cmpAllowableRegion", worldFrame, 20, registry);
      copAllowableRegionViz = new YoFrameConvexPolygon2D("copAllowableRegion", worldFrame, 20, registry);
      yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);
      unconstrainedCMP = new YoFramePoint2D("unconstrainedCMP", worldFrame, registry);
      copOfLeastEffort = new YoFramePoint2D("copOfLeastEffort", worldFrame, registry);

      if (yoGraphicsListRegistry == null)
         yoGraphicsListRegistry = new YoGraphicsListRegistry();

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Support Polygon", supportPolygonForViz, Color.pink, false);
      YoArtifactPolygon cmpPolygonArtifact = new YoArtifactPolygon("CMP Polygon", cmpAllowableRegionViz, Color.BLUE, false);
      YoArtifactPolygon copPolygonArtifact = new YoArtifactPolygon("CoP Polygon", copAllowableRegionViz, Color.RED, false);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", supportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", cmpPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", copPolygonArtifact);

      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, Blue(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition unconstrainedCMPViz = new YoGraphicPosition("Unconstrained CMP", unconstrainedCMP, 0.01, Purple(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoArtifactLineSegment2d lineOfAction = new YoArtifactLineSegment2d("Line Of Action", yoCapturePoint, unconstrainedCMP, Color.GREEN);
      YoGraphicPosition copOfLeastEffortViz = new YoGraphicPosition("CoP of Least Effort", copOfLeastEffort, 0.005, Purple(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", capturePointViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", lineOfAction);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", unconstrainedCMPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", copOfLeastEffortViz.createArtifact());

      scs = new SimulationConstructionSet(new Robot("Test"));

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      plotter = plotterFactory.createOverheadPlotter();

      scs.getRootRegistry().addChild(registry);
      scs.startOnAThread();

      plotter.update();
   }

   public void updateInputs(FrameConvexPolygon2DReadOnly supportPolygon, FramePoint2DReadOnly currentICP, FramePoint2DReadOnly unconstrainedCMP)
   {
      supportPolygonForViz.set(supportPolygon);
      yoCapturePoint.set(currentICP);
      this.unconstrainedCMP.set(unconstrainedCMP);

      plotter.update();
   }

   public void updateOutputs(DistanceInsideHeuristics heuristics)
   {
      scaler.scaleConvexPolygon(supportPolygonForViz, -heuristics.getCmpDistanceFromSupport(), cmpAllowableRegionViz);
      scaler.scaleConvexPolygon(supportPolygonForViz, heuristics.getCoPDistanceInsideSupport(), copAllowableRegionViz);

      copOfLeastEffort.set(heuristics.getCoPOfLeastEffort());
   }
}
