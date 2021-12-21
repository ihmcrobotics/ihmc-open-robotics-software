package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.BlueViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkViolet;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Purple;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testStandingWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(feedbackGain);
            gains.setKpOrthogonalToMotion(feedbackGain);

            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      
      
      boolean visualize = true;

      YoGraphicsListRegistry yoGraphicsListRegistry = null;

      if (visualize)
      {
         yoGraphicsListRegistry = new YoGraphicsListRegistry();
      }

      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters,
                                                                           bipedSupportPolygons, null, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

//      YoFramePoint2D desiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      YoFramePoint2D perfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);
      YoFramePoint2D yoPerfectCoP = new YoFramePoint2D("perfectCoP", worldFrame, registry);

      
      YoFramePoint2D yoDesiredCMP = new YoFramePoint2D("desiredCMP", worldFrame, registry);
      YoFramePoint2D yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
      YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
      YoFramePoint2D yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);

      YoFramePoint2D desiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      YoFramePoint2D yoAdjustedDesiredCapturePoint = new YoFramePoint2D("adjustedDesiredICP", worldFrame, registry);
      YoFrameVector2D yoDesiredICPVelocity = new YoFrameVector2D("desiredICPVelocity", worldFrame, registry);
      YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      YoFramePoint2D yoFinalDesiredICP = new YoFramePoint2D("finalDesiredICP", worldFrame, registry);
      YoFramePoint3D yoFinalDesiredCoM = new YoFramePoint3D("finalDesiredCoM", worldFrame, registry);
//      YoFrameVector3D yoFinalDesiredCoMVelocity = new YoFrameVector3D("finalDesiredCoMVelocity", worldFrame, registry);
//      YoFrameVector3D yoFinalDesiredCoMAcceleration = new YoFrameVector3D("finalDesiredCoMAcceleration", worldFrame, registry);

      
      
      
      SimulationConstructionSet scs = null;
      YoGraphicPosition desiredICPGraphic = null;
      YoGraphicPosition perfectCMPGraphic = null;
      SimulationOverheadPlotter plotter = null;
      
      if (visualize)
      {
         Robot nullRobot = new Robot("test");
         scs = new SimulationConstructionSet(nullRobot);

         ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());
         
//         desiredICPGraphic = new YoGraphicPosition("desiredICP", desiredICP, 0.03, YoAppearance.Yellow());
//         yoGraphicsListRegistry.registerYoGraphic("yoGraphics", desiredICPGraphic);
//         
//         perfectCMPGraphic = new YoGraphicPosition("perfectCMP", perfectCMP, 0.01, YoAppearance.Orange());
//         yoGraphicsListRegistry.registerYoGraphic("yoGraphics", perfectCMPGraphic);
         
         if (yoGraphicsListRegistry != null)
         {
            YoGraphicPosition desiredCMPViz = new YoGraphicPosition("Desired CMP", yoDesiredCMP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);
            YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
            YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
            YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredCMPViz.createArtifact());
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", achievedCMPViz.createArtifact());
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", centerOfMassViz.createArtifact());
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", capturePointViz.createArtifact());
        
         
         
            YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
            YoGraphicPosition finalDesiredCapturePointViz = new YoGraphicPosition("Final Desired Capture Point", yoFinalDesiredICP, 0.01, Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);
            YoGraphicPosition finalDesiredCoMViz = new YoGraphicPosition("Final Desired CoM", yoFinalDesiredCoM, 0.01, Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
            YoGraphicPosition perfectCMPViz = new YoGraphicPosition("Perfect CMP", perfectCMP, 0.002, BlueViolet());
            YoGraphicPosition perfectCoPViz = new YoGraphicPosition("Perfect CoP", yoPerfectCoP, 0.002, DarkViolet(), GraphicType.BALL_WITH_CROSS);

            YoGraphicPosition adjustedDesiredCapturePointViz = new YoGraphicPosition("Adjusted Desired Capture Point", yoAdjustedDesiredCapturePoint, 0.005, Yellow(), GraphicType.DIAMOND);
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", adjustedDesiredCapturePointViz.createArtifact());

            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", desiredCapturePointViz.createArtifact());
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", finalDesiredCapturePointViz.createArtifact());
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", finalDesiredCoMViz.createArtifact());
            YoArtifactPosition perfectCMPArtifact = perfectCMPViz.createArtifact();
            perfectCMPArtifact.setVisible(false);
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", perfectCMPArtifact);
            YoArtifactPosition perfectCoPArtifact = perfectCoPViz.createArtifact();
            perfectCoPArtifact.setVisible(false);
            yoGraphicsListRegistry.registerArtifact("ICPControllerTest", perfectCoPArtifact);
         
         
         }
         
         
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.setShowOnStart(true);
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         plotter = plotterFactory.createOverheadPlotter();
         
        
         scs.getRootRegistry().addChild(registry);
         scs.startOnAThread();
         
         plotter.update();
      }
      
      
      double omega = walkingControllerParameters.getOmega0();

      desiredICP.set(0.03, 0.06);
      perfectCMP.set(0.01, 0.04);
      
      if (visualize)
      {
         desiredICPGraphic.update();
         perfectCMPGraphic.update();
         plotter.update();
      }
      
      
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
      
      if (visualize)
      {
         for (int i=0; i<10; i++)
         {
         scs.setTime(scs.getTime() + 1.0);
         scs.tickAndUpdate();
         }
         ThreadTools.sleepForever();
      }
      
      
   }

   @Test
   public void testTransferWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(feedbackGain);
            gains.setKpOrthogonalToMotion(feedbackGain);

            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters,
                                                                           bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      FrameVector2D currentICPVelocity = new FrameVector2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoM = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoM, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
   }

   @Test
   public void testStandingConstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(feedbackGain);
            gains.setKpOrthogonalToMotion(feedbackGain);

            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      double footWidth = 0.1;
      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters,
                                                                           bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      double maxY = stanceWidth / 2.0;// + footWidth / 2.0;
      double maxX = footLength / 2.0;

      desiredCMPExpected.setX(Math.min(maxX, desiredCMPExpected.getX()));
      desiredCMPExpected.setY(Math.min(maxY, desiredCMPExpected.getY()));

      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(desiredCMPExpected, desiredCMP, epsilon);
   }

   @Test
   public void testStandingUnconstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(feedbackGain);
            gains.setKpOrthogonalToMotion(feedbackGain);

            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
         SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(10.0, 5.0, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters,
                                                   bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      FrameVector2D expectedCoPFeedbackDelta = new FrameVector2D();
      FrameVector2D expectedCMPFeedbackDelta = new FrameVector2D();
      expectedCoPFeedbackDelta.set(icpError);
      expectedCoPFeedbackDelta.scale(feedbackGain + 1.0);

      desiredCMPExpected.set(perfectCMP);
      desiredCMPExpected.add(expectedCMPFeedbackDelta);
      desiredCMPExpected.add(expectedCoPFeedbackDelta);

      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(icpError, controller.icpError, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(perfectCMP, controller.perfectCoP, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(perfectCMP, controller.perfectCMP, epsilon);
      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(expectedCMPFeedbackDelta, controller.feedbackCMPDelta, epsilon);
      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(expectedCoPFeedbackDelta, controller.feedbackCoPDelta, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(desiredCMPExpected, desiredCMP, epsilon);
   }

   @Test
   public void testStandingConstrainedWithAngularMomentum() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpOrthogonalToMotion(feedbackGain);
            gains.setKpParallelToMotion(feedbackGain);
            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 1000000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return true;
         }

         @Override
         public double getAngularMomentumMinimizationWeight()
         {
            return 1.0;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters,
                                                                           bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(desiredCMPExpected, desiredCMP, epsilon);
   }

   private SideDependentList<FootSpoof> setupContactableFeet(double footLength, double footWidth, double totalWidth)
   {
      SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
      SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, footWidth / 2.0));

         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.5 * (totalWidth - footWidth)));
         contactableFoot.setSoleFrame(startingPose);

         contactableFeet.put(robotSide, contactableFoot);
      }

      return contactableFeet;
   }

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoRegistry registry)
   {
      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
      
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         soleZUpFrames.put(robotSide, new ZUpFrame(contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      @Override
      public double getForwardFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getLateralFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getFootstepRateWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackForwardWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackLateralWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackRateWeight()
      {
         return 0.0001;
      }

      @Override
      public ICPControlGainsReadOnly getICPFeedbackGains()
      {
         ICPControlGains gains = new ICPControlGains();
         gains.setKpParallelToMotion(3.0);
         gains.setKpOrthogonalToMotion(2.5);

         return gains;
      }
      @Override
      public double getDynamicsObjectiveWeight()
      {
         return 500.0;
      }

      @Override
      public double getDynamicsObjectiveDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleStepRateWeightWithTime()
      {
         return false;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean useFeedbackRate()
      {
         return false;
      }

      @Override
      public boolean allowStepAdjustment()
      {
         return false;
      }

      @Override
      public boolean useAngularMomentum()
      {
         return false;
      }

      @Override
      public boolean useFootstepRate()
      {
         return false;
      }


      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.0001;
      }

      @Override
      public double getAdjustmentDeadband()
      {
         return 0.03;
      }

      @Override
      public double getSafeCoPDistanceToEdge()
      {
         return 0.0;
      }
   }

   private class TestWalkingControllerParameters extends WalkingControllerParameters
   {
      @Override
      public double getOmega0()
      {
         return 3.0;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         return false;
      }

      @Override
      public double getMinimumSwingTimeForDisturbanceRecovery()
      {
         return 0;
      }

      @Override
      public double getICPErrorThresholdToSpeedUpSwing()
      {
         return 0;
      }

      @Override
      public boolean allowAutomaticManipulationAbort()
      {
         return false;
      }

      @Override
      public ICPControlGains createICPControlGains()
      {
         return null;
      }

      @Override
      public PDGains getCoMHeightControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getSwingFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getHoldPositionFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getToeOffFootControlGains()
      {
         return null;
      }

      @Override
      public double getDefaultTransferTime()
      {
         return 0;
      }

      @Override
      public double getDefaultSwingTime()
      {
         return 0;
      }

      @Override
      public FootSwitchFactory getFootSwitchFactory()
      {
         return null;
      }

      @Override
      public String[] getJointsToIgnoreInController()
      {
         return new String[0];
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         return null;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportForwardX()
      {
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportInnerY()
      {
         return 0;
      }

      @Override
      public ToeOffParameters getToeOffParameters()
      {
         return null;
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return null;
      }

      @Override
      public double getMaximumLegLengthForSingularityAvoidance()
      {
         return 0;
      }

      @Override
      public double minimumHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double nominalHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double maximumHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public double defaultOffsetHeightAboveAnkle()
      {
         return 0;
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return new TestICPOptimizationParameters();
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return null;
      }
   }
}
