package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Brown;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPControllerTestVisualizer
{
   private final SimulationConstructionSet scs;
   private final YoGraphicPosition desiredICPGraphic;
   private final YoGraphicPosition perfectCMPGraphic;
   private final SimulationOverheadPlotter plotter;

   private final YoFramePoint2D yoPerfectCMP;
   private final YoFramePoint2D yoPerfectCoP;
   private final YoFramePoint2D yoDesiredCoP;
   private final YoFramePoint2D yoDesiredCMP;
   //   private final YoFramePoint2D yoAchievedCMP;
   private final YoFramePoint3D yoCenterOfMass;
   private final YoFramePoint2D yoCapturePoint;
   private final YoFramePoint2D yoDesiredICP;
   private final YoFramePoint3D yoDesiredICP3D;
   //   private final YoFramePoint2D yoAdjustedDesiredCapturePoint;
   private final YoFrameVector2D yoDesiredICPVelocity;
   private final YoFrameVector3D yoDesiredICPVelocity3D;

   //   private final YoFramePoint2D yoFinalDesiredICP;
   //   private final YoFramePoint3D yoFinalDesiredCoM;

   private final YoFrameConvexPolygon2D yoSupportPolygonInWorld;

   public ICPControllerTestVisualizer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      yoPerfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);
      yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);

      yoDesiredCoP = new YoFramePoint2D("desiredCoP", worldFrame, registry);
      yoDesiredCMP = new YoFramePoint2D("desiredCMP", worldFrame, registry);
      //      yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
      yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
      yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);

      yoDesiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      yoDesiredICP3D = new YoFramePoint3D("desiredICP3D", worldFrame, registry);
      //      yoAdjustedDesiredCapturePoint = new YoFramePoint2D("adjustedDesiredICP", worldFrame, registry);
      yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
      yoDesiredICPVelocity3D = new YoFrameVector3D("desiredICPVelocity3D", worldFrame, registry);

      //      yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
      //      yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);

      yoSupportPolygonInWorld = new YoFrameConvexPolygon2D("supportPolygon", worldFrame, 20, registry);

      Robot nullRobot = new Robot("test");
      scs = new SimulationConstructionSet(nullRobot);

      //      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      desiredICPGraphic = new YoGraphicPosition("desiredICP", yoDesiredICP, 0.03, YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("yoGraphics", desiredICPGraphic);

      perfectCMPGraphic = new YoGraphicPosition("perfectCMP", yoPerfectCMP, 0.0107, YoAppearance.Orange());
      yoGraphicsListRegistry.registerYoGraphic("yoGraphics", perfectCMPGraphic);

      YoGraphicVector desiredICPVelocityViz = new YoGraphicVector("DesiredICPVelocity",
                                                                  yoDesiredICP3D,
                                                                  yoDesiredICPVelocity3D,
                                                                  0.01,
                                                                  YoAppearance.Yellow(),
                                                                  true);

      YoGraphicPosition desiredCoPViz = new YoGraphicPosition("Desired CoP", yoDesiredCoP, 0.01, Brown(), GraphicType.DIAMOND);
      YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", yoDesiredCMP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);
      //      YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredCoPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredCMPViz.createArtifact());
      //      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", achievedCMPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", capturePointViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredICPVelocityViz.createArtifact());

      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point",
                                                                       yoDesiredICP,
                                                                       0.011,
                                                                       Yellow(),
                                                                       GraphicType.BALL_WITH_ROTATED_CROSS);
      //      YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.012, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
      //      YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired CoM", yoFinalDesiredCoM, 0.013, Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", yoPerfectCMP, 0.002, BlueViolet());
      YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.0021, DarkViolet(), GraphicType.BALL_WITH_CROSS);

      //      YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);

      //      YoFramePose3D zeroFramePose = new YoFramePose3D("zeroFramePose", worldFrame, registry);
      //      supportPolygonGraphic = new YoGraphicPolygon("Support Polygon", yoSupportPolygonInWorld, zeroFramePose, 0, YoAppearance.Black());

      //      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", adjustedDesiredCapturePointViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredCapturePointViz.createArtifact());
      //      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", finalDesiredCapturePointViz.createArtifact());
      //      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", finalDesiredCoMViz.createArtifact());
      YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
      //      perfectCMPArtifact.setVisible(false);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", perfectCMPArtifact);
      YoArtifactPosition perfectCoPArtifact = perfectCoPViz.createArtifact();
      //      perfectCoPArtifact.setVisible(false);
      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", perfectCoPArtifact);

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Support Polygon", yoSupportPolygonInWorld, Color.pink, false);

      yoGraphicsListRegistry.registerArtifact("ICPControllerTest", supportPolygonArtifact);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      plotter = plotterFactory.createOverheadPlotter();

      scs.getRootRegistry().addChild(registry);
      scs.startOnAThread();

      plotter.update();
   }

   public void updateInputs(BipedSupportPolygons bipedSupportPolygons,
                            FramePoint2D desiredICP,
                            FrameVector2D desiredICPVelocity,
                            FramePoint2D perfectCMP,
                            FramePoint2D perfectCoP,
                            FramePoint2D currentICP,
                            FramePoint2D currentCoMPosition)
   {
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      this.yoDesiredICP.set(desiredICP);
      this.yoDesiredICP3D.set(desiredICP);
      this.yoDesiredICPVelocity.set(desiredICPVelocity);
      this.yoDesiredICPVelocity3D.set(desiredICPVelocity);
      this.yoPerfectCMP.set(perfectCMP);
      this.yoPerfectCoP.set(perfectCoP);
      this.yoCapturePoint.set(currentICP);
      this.yoCenterOfMass.set(currentCoMPosition);

      yoSupportPolygonInWorld.set(supportPolygonInWorld);

      //      scs.setTime(scs.getTime() + 1.0);
      //      scs.tickAndUpdate();

      desiredICPGraphic.update();
      perfectCMPGraphic.update();
      plotter.update();
   }

   public void updateOutputs(FramePoint2D desiredCoP, FramePoint2D desiredCMP)
   {
      this.yoDesiredCMP.set(desiredCMP);
      this.yoDesiredCoP.set(desiredCoP);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();

      plotter.update();
   }
}
