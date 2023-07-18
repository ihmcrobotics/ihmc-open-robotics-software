package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class MultiStepPaperVisualizers
{
   double footWidth = 0.1;
   double footLength = 0.2;
   double kinematicStepRange = 1.0;
   double forwardLimit = 1.0;
   double backwardLimit = 0.8;
   double innerLimit = 0.05;
   double outerLimit = 0.6;
   double inPlaceWidth = 0.3;
   double swingDuration = 0.6;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleAnkleZUpReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final SimpleAnkleZUpReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>(leftAnkleZUpFrame, rightAnkleZUpFrame);

   private final YoRegistry registry = new YoRegistry("CaptureRegionCalculatorTest");

   private final YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
   private final YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
   private final YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
   private final YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
   private final YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
   private final YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
   private final YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

   public MultiStepPaperVisualizers()
   {
      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(inPlaceWidth);
      yoSwingDuration.set(swingDuration);

   }

   public void visualizeWithTimeDelay()
   {

      leftAnkleZUpFrame.setOffset(-0.15, 0.1, 0.0);
      rightAnkleZUpFrame.setOffset(0.15, -0.1, 0.0);
      leftAnkleZUpFrame.update();
      rightAnkleZUpFrame.update();
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  kinematicStepRange,
                                                                                                  ankleZUpFrames,
                                                                                                  registry,
                                                                                                  null);

      FrameConvexPolygon2D leftFootPolygon = new FrameConvexPolygon2D(leftAnkleZUpFrame, createFootPolygon());
      FrameConvexPolygon2D rightFootPolygon = new FrameConvexPolygon2D(rightAnkleZUpFrame, createFootPolygon());
      leftFootPolygon.changeFrame(worldFrame);
      rightFootPolygon.changeFrame(worldFrame);
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(worldFrame);
      supportPolygon.addVertices(leftFootPolygon);
      supportPolygon.addVertices(rightFootPolygon);
      supportPolygon.update();

      FramePoint2D currentICP = new FramePoint2D(worldFrame, 0.27, 0.0);

      double swingDuration = 0.6;
      double transferDuration = 0.25;
      double omega = 3.0;

      YoFrameConvexPolygon2D delayedRegion = new YoFrameConvexPolygon2D("delayedCaptureRegion",worldFrame,  20, registry);
      YoFrameConvexPolygon2D swingRegion = new YoFrameConvexPolygon2D("captureRegion",worldFrame,  20, registry);

      captureRegionCalculator.calculateCaptureRegion(RobotSide.LEFT, transferDuration + swingDuration, currentICP, omega, rightFootPolygon);
      delayedRegion.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);

      captureRegionCalculator.calculateCaptureRegion(RobotSide.LEFT, swingDuration, currentICP, omega, rightFootPolygon);
      swingRegion.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);


      YoFrameConvexPolygon2D yoSupportPOlygon = new YoFrameConvexPolygon2D("supportPOlygon" ,worldFrame,  8, registry);
      YoFrameConvexPolygon2D yoLeftFoot = new YoFrameConvexPolygon2D("leftFoot", worldFrame, 4, registry);
      YoFrameConvexPolygon2D yoRightFoot = new YoFrameConvexPolygon2D("rightFoot", worldFrame, 4, registry);

      YoFramePoint2D yoICP = new YoFramePoint2D("icp", worldFrame, registry);
      yoICP.set(currentICP);

      yoSupportPOlygon.setMatchingFrame(supportPolygon, false);
      yoLeftFoot.setMatchingFrame(leftFootPolygon, false);
      yoRightFoot.setMatchingFrame(rightFootPolygon, false);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("leftFoot", yoLeftFoot, Color.GRAY, true));
      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("rightFoot", yoRightFoot, Color.GRAY, true));
      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("supportPOlygon", yoSupportPOlygon, Color.GRAY, false));
      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("delayedCpatureRegion", delayedRegion, Color.RED, false));
      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("captureRegion", swingRegion, Color.YELLOW, false));
      graphicsListRegistry.registerArtifact("regions", new YoGraphicPosition("capturePoint", yoICP, 0.025, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS).createArtifact());


      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));
      scs.startSimulationThread();
      scs.simulateNow(1);
   }

   public void visualizeDynamicPlan()
   {

      leftAnkleZUpFrame.setOffset(-0.15, 0.1, 0.0);
      rightAnkleZUpFrame.setOffset(0.15, -0.1, 0.0);
      leftAnkleZUpFrame.update();
      rightAnkleZUpFrame.update();
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  kinematicStepRange,
                                                                                                  ankleZUpFrames,
                                                                                                  registry,
                                                                                                  null);

      double gravity = 9.81;
      double height = 1.0;
      double stepLength = 1.0;
      double stanceWidth = 0.2;
      CoMTrajectoryPlanner trajectoryPlanner = new CoMTrajectoryPlanner(gravity, height, registry);

      FrameConvexPolygon2D leftFootPolygon = new FrameConvexPolygon2D(leftAnkleZUpFrame, createFootPolygon());
      FrameConvexPolygon2D rightFootPolygon = new FrameConvexPolygon2D(rightAnkleZUpFrame, createFootPolygon());
      leftFootPolygon.changeFrame(worldFrame);
      rightFootPolygon.changeFrame(worldFrame);


      double fastSwing = 0.55;
      double fastTransfer = 0.02;
      double omega = Math.sqrt(gravity / height);
      int steps = 3;
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      createSettableContactStatesViewer(stepLength, stanceWidth, createFootPolygon(), steps, registry, graphicsListRegistry);


      List<SettableContactStateProvider> slowContactStates = createSettableContactStates(stepLength, stanceWidth, 1.0, 0.5, steps);
      trajectoryPlanner.solveForTrajectory(slowContactStates);
      viewCaptureCornerPoints("slow", slowContactStates, trajectoryPlanner, registry, graphicsListRegistry, YoGraphicPosition.GraphicType.BALL);


      List<SettableContactStateProvider> fastContactStates = createSettableContactStates(stepLength, stanceWidth, fastSwing, fastTransfer, steps);
      trajectoryPlanner.solveForTrajectory(fastContactStates);
      viewCaptureCornerPoints("fast", fastContactStates, trajectoryPlanner, registry, graphicsListRegistry, YoGraphicPosition.GraphicType.SOLID_BALL);

      double alpha = 0.9;
      trajectoryPlanner.compute(1, alpha * fastSwing);

      FramePoint2D currentICP = new FramePoint2D(trajectoryPlanner.getDesiredDCMPosition());

      YoFrameConvexPolygon2D swingRegion = new YoFrameConvexPolygon2D("captureRegion", worldFrame,  20, registry);
      captureRegionCalculator.calculateCaptureRegion(RobotSide.LEFT, (1.0 - alpha) * fastSwing, currentICP, omega, rightFootPolygon);

      swingRegion.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);



      YoFramePoint2D yoICP = new YoFramePoint2D("icp", worldFrame, registry);
      yoICP.set(currentICP);

      graphicsListRegistry.registerArtifact("regions", new YoArtifactPolygon("captureRegion", swingRegion, Color.YELLOW, false));
      graphicsListRegistry.registerArtifact("regions", new YoGraphicPosition("capturePoint", yoICP, 0.025, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS).createArtifact());


      SimulationConstructionSet2 scs = new SimulationConstructionSet2();
      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));
      scs.startSimulationThread();
      scs.simulateNow(1);
   }

   public void visualizeDifferentCaptureRegionsForPaper()
   {

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;
      double horiztonalOffset = 0.75;

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  kinematicStepRange,
                                                                                                  ankleZUpFrames,
                                                                                                  registry,
                                                                                                  null);

      StepAdjustmentReachabilityConstraint reachabilityConstraint = new StepAdjustmentReachabilityConstraint(ankleZUpFrames,
                                                                                                             yoForwardLimit,
                                                                                                             yoBackwardLimit,
                                                                                                             yoInnerLimit,
                                                                                                             yoOuterLimit,
                                                                                                             yoNominalWidth,
                                                                                                             new StepAdjustmentParameters.CrossOverReachabilityParameters(),
                                                                                                             "name",
                                                                                                             false,
                                                                                                             registry,
                                                                                                             null);
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint, () -> false, registry);

      SimpleAnkleZUpReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("CrossOverleftAnkleZUp");
      SimpleAnkleZUpReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("CrossOverrightAnkleZUp");
      SideDependentList<ReferenceFrame> crossOverAnkleZUpFrames = new SideDependentList<>(leftAnkleZUpFrame, rightAnkleZUpFrame);

      YoRegistry crossOverRegistry = new YoRegistry("supaChild");
      registry.addChild(crossOverRegistry);
      StepAdjustmentReachabilityConstraint crossOverReachabilityConstraint = new StepAdjustmentReachabilityConstraint(crossOverAnkleZUpFrames,
                                                                                                                      yoForwardLimit,
                                                                                                                      yoBackwardLimit,
                                                                                                                      yoInnerLimit,
                                                                                                                      yoOuterLimit,
                                                                                                                      yoNominalWidth,
                                                                                                                      new CrossoverParametersForPaper(),
                                                                                                                      "crossOverName",
                                                                                                                      false,
                                                                                                                      crossOverRegistry,
                                                                                                                      null);
      MultiStepCaptureRegionCalculator crossOverMultiStepRegionCalculator = new MultiStepCaptureRegionCalculator(crossOverReachabilityConstraint,
                                                                                                                 () -> true,
                                                                                                                 crossOverRegistry);

      SimpleMultiStepCaptureRegionCalculator simpleMultiStepRegionCalculator = new SimpleMultiStepCaptureRegionCalculator(registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      RigidBodyTransform crossOverTranslation = new RigidBodyTransform();
      crossOverTranslation.getTranslation().addY(-horiztonalOffset);
      leftAnkleZUpFrame.setOffset(crossOverTranslation.getTranslation());
      rightAnkleZUpFrame.setOffset(crossOverTranslation.getTranslation());
      leftAnkleZUpFrame.update();
      rightAnkleZUpFrame.update();

      crossOverReachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D(worldFrame, crossOverTranslation));
      crossOverReachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D(worldFrame, crossOverTranslation));

      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, createFootPolygon());

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.3, -0.2);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);
      FrameConvexPolygon2D simpleCaptureRegion = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
      simpleCaptureRegion.changeFrameAndProjectToXYPlane(worldFrame);
      FrameConvexPolygon2D crossOverCaptureRegion = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
      crossOverCaptureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      RigidBodyTransform translation = new RigidBodyTransform();
      translation.getTranslation().addY(horiztonalOffset);
      simpleCaptureRegion.applyTransform(translation);

      crossOverCaptureRegion.applyTransform(crossOverTranslation);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      int numberOfRegions = 6;
      java.util.List<YoFrameConvexPolygon2D> trueCaptureRegions = createRegionPool("true", numberOfRegions, registry);
      java.util.List<YoFrameConvexPolygon2D> simpleCaptureRegions = createRegionPool("simple", numberOfRegions, registry);
      java.util.List<YoFrameConvexPolygon2D> crossOverCaptureRegions = createRegionPool("crossOver", numberOfRegions, registry);
      graphicsListRegistry.registerArtifactList(createRegionGraphics("true", trueCaptureRegions));
      graphicsListRegistry.registerArtifactList(createRegionGraphics("simple", simpleCaptureRegions));
      graphicsListRegistry.registerArtifactList(createRegionGraphics("crossOver", crossOverCaptureRegions));

      YoFrameConvexPolygon2D regularReachability = new YoFrameConvexPolygon2D("regularReachability", worldFrame, 20, registry);
      YoFrameConvexPolygon2D crossOverReachability = new YoFrameConvexPolygon2D("CrossOVerReachability", worldFrame, 20, registry);
      regularReachability.setMatchingFrame(reachabilityConstraint.getReachabilityPolygonInFootFrame(RobotSide.LEFT), false);
      crossOverReachability.setMatchingFrame(crossOverReachabilityConstraint.getTotalReachabilityHull(RobotSide.LEFT), false);
      graphicsListRegistry.registerArtifact("regular", new YoArtifactPolygon("regularReachability", regularReachability, Color.blue, false));
      graphicsListRegistry.registerArtifact("crossover", new YoArtifactPolygon("crossOverReachability", crossOverReachability, Color.blue, false));

      java.util.List<YoFrameConvexPolygon2D> trueMultiStepRegions = new ArrayList<>();
      for (int i = 1; i < trueCaptureRegions.size(); i++)
         trueMultiStepRegions.add(trueCaptureRegions.get(i));
      java.util.List<YoFrameConvexPolygon2D> simpleMultiStepRegions = new ArrayList<>();
      for (int i = 1; i < simpleCaptureRegions.size(); i++)
         simpleMultiStepRegions.add(simpleCaptureRegions.get(i));
      java.util.List<YoFrameConvexPolygon2D> crossOverMultiStepRegions = new ArrayList<>();
      for (int i = 1; i < crossOverCaptureRegions.size(); i++)
         crossOverMultiStepRegions.add(crossOverCaptureRegions.get(i));

      us.ihmc.simulationconstructionset.Robot robot = new us.ihmc.simulationconstructionset.Robot("test");
      robot.getRobotsYoRegistry().addChild(registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();

      trueCaptureRegions.get(0).setMatchingFrame(captureRegion, false);
      simpleCaptureRegions.get(0).setMatchingFrame(simpleCaptureRegion, false);
      crossOverCaptureRegions.get(0).setMatchingFrame(crossOverCaptureRegion, false);

      updateRegionsForPaper(yoSwingDuration.getDoubleValue(),
                            multiStepRegionCalculator,
                            simpleMultiStepRegionCalculator,
                            crossOverMultiStepRegionCalculator,
                            captureRegion,
                            simpleCaptureRegion,
                            crossOverCaptureRegion,
                            omega0,
                            yoForwardLimit.getDoubleValue(),
                            swingSide.getOppositeSide(),
                            trueMultiStepRegions,
                            simpleMultiStepRegions,
                            crossOverMultiStepRegions);

      YoVariableChangedListener updatedListener = v ->
      {
         reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
         reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

         updateRegionsForPaper(yoSwingDuration.getDoubleValue(),
                               multiStepRegionCalculator,
                               simpleMultiStepRegionCalculator,
                               crossOverMultiStepRegionCalculator,
                               captureRegion,
                               simpleCaptureRegion,
                               crossOverCaptureRegion,
                               omega0,
                               yoForwardLimit.getDoubleValue(),
                               swingSide.getOppositeSide(),
                               trueMultiStepRegions,
                               simpleMultiStepRegions,
                               crossOverMultiStepRegions);
         scs.tickAndUpdate();
      };

      yoUseCrossoverSteps.addListener(updatedListener);
      yoForwardLimit.addListener(updatedListener);
      yoBackwardLimit.addListener(updatedListener);
      yoInnerLimit.addListener(updatedListener);
      yoOuterLimit.addListener(updatedListener);
      yoNominalWidth.addListener(updatedListener);
      yoSwingDuration.addListener(updatedListener);

      scs.tickAndUpdate();

      ThreadTools.sleepForever();
   }

   private ConvexPolygon2D createFootPolygon()
   {
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      polygon2D.addVertex(-footLength / 2.0, -footWidth / 2.0);
      polygon2D.addVertex(-footLength / 2.0, footWidth / 2.0);
      polygon2D.addVertex(footLength / 2.0, -footWidth / 2.0);
      polygon2D.addVertex(footLength / 2.0, footWidth / 2.0);
      polygon2D.update();

      return polygon2D;
   }

   private static java.util.List<YoFrameConvexPolygon2D> createRegionPool(String prefix, int regionsTomake, YoRegistry registry)
   {
      java.util.List<YoFrameConvexPolygon2D> regions = new ArrayList<>();

      for (int i = 0; i < regionsTomake; i++)
      {
         regions.add(new YoFrameConvexPolygon2D(prefix + "StepRegion" + (i + 1), worldFrame, 100, registry));
      }

      return regions;
   }

   private static ArtifactList createRegionGraphics(String prefix, List<YoFrameConvexPolygon2D> regions)
   {
      return createRegionGraphics(prefix, regions.toArray(new YoFrameConvexPolygon2D[0]));
   }

   private static ArtifactList createRegionGraphics(String prefix, YoFrameConvexPolygon2D... regions)
   {
      ArtifactList artifactList = new ArtifactList(prefix + "regions");
      for (int i = 0; i < regions.length; i++)
      {
         int colorIdx = i % 3;
         Color color;
         if (colorIdx == 0)
            color = Color.green;
         else if (colorIdx == 1)
            color = Color.red;
         else
            color = Color.yellow;
         artifactList.add(new YoArtifactPolygon(prefix + "StepRegion" + (i + 1), regions[i], color, false));
      }

      return artifactList;
   }

   private static void createSettableContactStatesViewer(double stepLength, double stanceWidth, ConvexPolygon2DReadOnly footPolygon,
                                                         int steps,
                                                         YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoFramePoint2D cop0 = new YoFramePoint2D("cop0", worldFrame, registry);


      YoGraphicPosition cop0Viz = new YoGraphicPosition("cop0", cop0, 0.02, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
      graphicsListRegistry.registerArtifact("CoPs", cop0Viz.createArtifact());


      YoFrameConvexPolygon2D polygon0 = new YoFrameConvexPolygon2D("polygon0", worldFrame, 4, registry);
      YoFrameConvexPolygon2D polygon1 = new YoFrameConvexPolygon2D("polygon1", worldFrame, 4, registry);
      polygon0.set(footPolygon);
      polygon1.set(footPolygon);

      polygon0.translate(0.0, 0.5 * stanceWidth);
      polygon1.translate(0.0, -0.5 * stanceWidth);

      graphicsListRegistry.registerArtifact("polygons", new YoArtifactPolygon("polygon0", polygon0, Color.green, false));
      graphicsListRegistry.registerArtifact("polygons", new YoArtifactPolygon("polygon1", polygon1, Color.green, false));
      double width = -0.5 * stanceWidth;
      for (int i = 0; i < steps; i++)
      {
         YoFrameConvexPolygon2D polygon = new YoFrameConvexPolygon2D("polygon" + (i + 2), worldFrame, 4, registry);
         polygon.set(footPolygon);
         polygon.translate(i * stepLength, width);
         width = -width;

         YoFramePoint2D cop = new YoFramePoint2D("cop" + (i + 1), worldFrame, registry);
         cop.set(i * stepLength, -width);

         YoGraphicPosition cop1Viz = new YoGraphicPosition("cop" + (i + 1), cop, 0.02, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
         graphicsListRegistry.registerArtifact("CoPs", cop1Viz.createArtifact());

         graphicsListRegistry.registerArtifact("polygons", new YoArtifactPolygon("polygon" + (i + 2), polygon, Color.green, false));
      }

      YoFrameConvexPolygon2D lastPolygon = new YoFrameConvexPolygon2D("polygon" + (steps + 2), worldFrame, 4, registry);
      lastPolygon.set(footPolygon);
      lastPolygon.translate((steps - 1) * stepLength, width);
      graphicsListRegistry.registerArtifact("polygons", new YoArtifactPolygon("polygon" + (stepLength + 2), lastPolygon, Color.green, false));


      YoFramePoint2D cop = new YoFramePoint2D("cop" + (steps + 1), worldFrame, registry);
      cop.set((steps - 1) * stepLength, 0.0);

      YoGraphicPosition cop1Viz = new YoGraphicPosition("cop" + (steps + 1), cop, 0.02, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
      graphicsListRegistry.registerArtifact("CoPs", cop1Viz.createArtifact());
   }

   private static void viewCaptureCornerPoints(String prefix, List<SettableContactStateProvider> contactStates, CoMTrajectoryPlanner planner, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry,
                                               YoGraphicPosition.GraphicType graphicType)
   {
      int segments = planner.getCoMTrajectory().getCurrentNumberOfSegments();
      int index = 0;


      double size = 0.01;

      YoFramePoint2D icpStart = new YoFramePoint2D(prefix + "ICP" + index, worldFrame, registry);
      planner.compute(0, 0.0);
      double time = 0.0;
      icpStart.set(planner.getDesiredDCMPosition());
      YoGraphicPosition icpStartViz = new YoGraphicPosition(prefix + "ICP" + index, icpStart, size, YoAppearance.Blue(), graphicType);
      graphicsListRegistry.registerArtifact(prefix + "ICPs", icpStartViz.createArtifact());

      index++;

      for (int segment = 0; segment < segments; segment++)
      {
         YoFramePoint2D icp = new YoFramePoint2D(prefix + "ICP" + index, worldFrame, registry);
         time += contactStates.get(segment).getTimeInterval().getDuration();
         planner.compute(time);
         icp.set(planner.getDesiredDCMPosition());
         YoGraphicPosition icpViz = new YoGraphicPosition(prefix + "ICP" + index, icp, size, YoAppearance.Blue(), graphicType);
         graphicsListRegistry.registerArtifact(prefix + "ICPs", icpViz.createArtifact());

         index++;
      }
   }



   private static List<SettableContactStateProvider> createSettableContactStates(double stepLength, double stanceWidth, double swingDuration, double transferDuration, int steps)
   {
      SettableContactStateProvider transfer0 = new SettableContactStateProvider();
      transfer0.setStartECMPPosition(new FramePoint3D());
      transfer0.setEndECMPPosition(new FramePoint3D(worldFrame, 0.0, -0.5 * stanceWidth, 0.0));
      double startTime = 0.0;
      transfer0.getTimeInterval().setInterval(startTime, startTime += transferDuration);

      SettableContactStateProvider swing0 = new SettableContactStateProvider();
      swing0.setStartECMPPosition(transfer0.getECMPEndPosition());
      swing0.setEndECMPPosition(transfer0.getECMPEndPosition());
      swing0.getTimeInterval().setInterval(startTime, startTime += swingDuration);

      List<SettableContactStateProvider> contactStates = new ArrayList<>();
      contactStates.add(transfer0);
      contactStates.add(swing0);

      SettableContactStateProvider previousSwing = swing0;
      double width = 0.5 * stanceWidth;

      for (int step = 1; step < steps; step++)
      {
         SettableContactStateProvider transfer = new SettableContactStateProvider();
         transfer.setStartECMPPosition(previousSwing.getECMPEndPosition());
         transfer.setEndECMPPosition(new FramePoint3D(worldFrame, step * stepLength, width, 0.0));
         transfer.getTimeInterval().setInterval(startTime, startTime += transferDuration);

         SettableContactStateProvider swing = new SettableContactStateProvider();
         swing.setStartECMPPosition(transfer.getECMPEndPosition());
         swing.setEndECMPPosition(transfer.getECMPEndPosition());
         swing.getTimeInterval().setInterval(startTime, startTime += swingDuration);

         contactStates.add(transfer);
         contactStates.add(swing);

         previousSwing = swing;
         width = -width;
      }

      SettableContactStateProvider finalTransfer = new SettableContactStateProvider();
      finalTransfer.setStartECMPPosition(previousSwing.getECMPEndPosition());
      finalTransfer.setEndECMPPosition(new FramePoint3D(worldFrame, (steps - 1) * stepLength, 0.0, 0.0));
      finalTransfer.getTimeInterval().setInterval(startTime, startTime + transferDuration);

      contactStates.add(finalTransfer);

      return contactStates;
   }

   private void updateRegionsForPaper(double swingDuration,
                                      MultiStepCaptureRegionCalculator calculator,
                                      SimpleMultiStepCaptureRegionCalculator simpleCalculator,
                                      MultiStepCaptureRegionCalculator crossOverCalculator,
                                      FrameConvexPolygon2DReadOnly captureRegion,
                                      FrameConvexPolygon2DReadOnly simpleCaptureRegion,
                                      FrameConvexPolygon2DReadOnly crossOverCaptureRegion,
                                      double omega,
                                      double stepReach,
                                      RobotSide stanceSide,
                                      java.util.List<YoFrameConvexPolygon2D> truePolygons,
                                      java.util.List<YoFrameConvexPolygon2D> simplePolygons,
                                      List<YoFrameConvexPolygon2D> crossOverPolygons)
   {
      int i = 1;
      for (YoFrameConvexPolygon2D polygon : truePolygons)
      {
         i++;
         calculator.compute(stanceSide, captureRegion, swingDuration, omega, i);
         polygon.setMatchingFrame(calculator.getCaptureRegion(), false);
      }

      i = 1;
      for (YoFrameConvexPolygon2D polygon : simplePolygons)
      {
         i++;
         simpleCalculator.compute(simpleCaptureRegion, swingDuration, omega, stepReach, i);
         polygon.setMatchingFrame(simpleCalculator.getCaptureRegion(), false);
      }
      i = 1;
      for (YoFrameConvexPolygon2D polygon : crossOverPolygons)
      {
         i++;
         crossOverCalculator.compute(stanceSide, crossOverCaptureRegion, swingDuration, omega, i);
         polygon.setMatchingFrame(crossOverCalculator.getCaptureRegion(), false);
      }
   }

   private static class SimpleAnkleZUpReferenceFrame extends ReferenceFrame
   {
      private final Vector3D offset = new Vector3D();

      public SimpleAnkleZUpReferenceFrame(String name)
      {
         super(name, ReferenceFrame.getWorldFrame());
      }

      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setIdentity();
         transformToParent.getTranslation().set(offset);
      }

      public void setOffset(Tuple3DReadOnly offset)
      {
         setOffset(offset.getX(), offset.getY(), offset.getZ());
      }

      public void setOffset(double x, double y, double z)
      {
         this.offset.set(x, y, z);
      }
   }

   private class CrossoverParametersForPaper extends StepAdjustmentParameters.CrossOverReachabilityParameters
   {
      @Override
      public double getForwardCrossOverDistance()
      {
         return 0.2;
      }

      public double getBackwardCrossOverDistance()
      {
         return 0.1;
      }
   }

   public static void main(String[] args)
   {
      MultiStepPaperVisualizers test = new MultiStepPaperVisualizers();
      test.visualizeDynamicPlan();
   }
}
