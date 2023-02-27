package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

public class TimeAdjustmentCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private double time = 0.0;
   private double dt = 0.1;

   private final SimulationConstructionSet scs;
   private final SimulationOverheadPlotter plotter;

   private final YoFramePoint2D desiredICP;
   private final YoFramePoint2D currentICP;
   private final YoFramePoint2D touchdownICP;
   private final YoFramePoint2D desiredCMP;

   private final YoFramePoint2D projectedICP;
   private final YoDouble timeAdjustment;

   public TimeAdjustmentCalculatorVisualizer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (registry == null)
         registry = new YoRegistry(getClass().getSimpleName());
      if (yoGraphicsListRegistry == null)
         yoGraphicsListRegistry = new YoGraphicsListRegistry();

      currentICP = new YoFramePoint2D("currentICP", worldFrame, registry);
      desiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      touchdownICP = new YoFramePoint2D("touchdownICP", worldFrame, registry);
      desiredCMP = new YoFramePoint2D("desiredCMP", worldFrame, registry);

      projectedICP = new YoFramePoint2D("projectedICP", worldFrame, registry);
      timeAdjustment = new YoDouble("timeAdjustment", registry);

      YoGraphicPosition currentICPViz = new YoGraphicPosition("Capture Point", currentICP, 0.01, Blue(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition desiredICPViz = new YoGraphicPosition("Desired Capture Point",
                                                              desiredICP,
                                                              0.01,
                                                              Yellow(),
                                                              YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition touchdownICPViz = new YoGraphicPosition("Touchdown Capture Point",
                                                                touchdownICP,
                                                                0.01,
                                                                Teal(),
                                                                YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition currentCMPViz = new YoGraphicPosition("CMP", desiredCMP, 0.01, Purple(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoGraphicPosition projectedICPViz = new YoGraphicPosition("Projected ICP",
                                                                projectedICP,
                                                                0.005,
                                                                YoAppearance.AliceBlue(),
                                                                YoGraphicPosition.GraphicType.SOLID_BALL);

      yoGraphicsListRegistry.registerArtifact("SpeedUpTest", currentICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("SpeedUpTest", desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("SpeedUpTest", touchdownICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("SpeedUpTest", currentCMPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("SpeedUpTest", projectedICPViz.createArtifact());

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

   public void updateInputs(FramePoint2DReadOnly currentICP,
                            FramePoint2DReadOnly desiredICP,
                            FramePoint2DReadOnly touchdownICP,
                            FramePoint2DReadOnly desiredCMP)
   {
      this.desiredICP.set(desiredICP);
      this.currentICP.set(currentICP);
      this.touchdownICP.set(touchdownICP);
      this.desiredCMP.set(desiredCMP);

      time += dt;
      scs.setTime(time);

      scs.tickAndUpdate();
      plotter.update();
   }

   public void updateOutputs(FramePoint2DReadOnly projectedICP, double timeAdjustment)
   {
      this.projectedICP.set(projectedICP);
      this.timeAdjustment.set(timeAdjustment);

      time += dt;
      scs.setTime(time);

      scs.tickAndUpdate();
      plotter.update();
   }
}
