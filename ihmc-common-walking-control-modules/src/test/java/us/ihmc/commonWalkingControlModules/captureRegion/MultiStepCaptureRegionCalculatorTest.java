package us.ihmc.commonWalkingControlModules.captureRegion;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentReachabilityConstraint;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class MultiStepCaptureRegionCalculatorTest
{
   private static final boolean PLOT_RESULTS = true;
   private static final boolean WAIT_FOR_BUTTON_PUSH = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleAnkleZUpReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final SimpleAnkleZUpReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>(leftAnkleZUpFrame, rightAnkleZUpFrame);

   private final YoRegistry registry = new YoRegistry("CaptureRegionCalculatorTest");

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @SuppressWarnings("unused")
   @Test
   public void testPointsInsideSimpleSquareRegion()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;
      boolean useCrossOverSteps = false;

      YoBoolean yoUseCrossOverSteps = new YoBoolean("yoUseCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossOverSteps.set(useCrossOverSteps);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossOverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      for (RobotSide swingSide : RobotSide.values())
      {

         ReferenceFrame stanceFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
         FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(stanceFrame);
         captureRegion.addVertex(0.2, swingSide.negateIfRightSide(0.2));
         captureRegion.addVertex(0.4, swingSide.negateIfRightSide(0.2));
         captureRegion.addVertex(0.4, swingSide.negateIfRightSide(0.4));
         captureRegion.addVertex(0.2, swingSide.negateIfRightSide(0.4));
         captureRegion.update();

         captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

         //         testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

         if (PLOT_RESULTS)
         {
            YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
            YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
            YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 40, registry);
            YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 50, registry);
            YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 60, registry);
            YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 80, registry);
            YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

            YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
            YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
            YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
            YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
            YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
            YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

            graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

            SimulationConstructionSet2 scs = new SimulationConstructionSet2();
            scs.addRegistry(registry);


            MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                               () -> scs.simulateNow(1),
                                                                                               registry,
                                                                                               graphicsListRegistry);

            scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));

            //         multiStepRegionCalculator.attachVisualizer(visualizer);

            scs.startSimulationThread();

            yoOneStepRegion.setMatchingFrame(captureRegion, false);

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);

            YoVariableChangedListener updatedListener = v ->
            {
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

               updateRegions(yoSwingDuration.getDoubleValue(),
                             multiStepRegionCalculator,
                             captureRegion,
                             omega0,
                             swingSide.getOppositeSide(),
                             yoTwoStepRegion,
                             yoThreeStepRegion,
                             yoFourStepRegion,
                             yoFiveStepRegion,
                             yoSixStepRegion);
               scs.simulateNow(1);
            };

            yoUseCrossOverSteps.addListener(updatedListener);
            yoForwardLimit.addListener(updatedListener);
            yoBackwardLimit.addListener(updatedListener);
            yoInnerLimit.addListener(updatedListener);
            yoOuterLimit.addListener(updatedListener);
            yoNominalWidth.addListener(updatedListener);
            yoSwingDuration.addListener(updatedListener);

            scs.simulateNow(1);

            ThreadTools.sleepForever();
         }
      }
   }

   @Test
   public void testPointsInsideSimpleLine()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;
      boolean useCrossOverSteps = false;

      YoBoolean yoUseCrossOverSteps = new YoBoolean("yoUseCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossOverSteps.set(useCrossOverSteps);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossOverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      for (RobotSide swingSide : RobotSide.values())
      {

         ReferenceFrame stanceFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
         FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(stanceFrame);
         captureRegion.addVertex(0.2, swingSide.negateIfRightSide(0.2));
         captureRegion.addVertex(0.4, swingSide.negateIfRightSide(0.4));
//         captureRegion.addVertex(0.4, swingSide.negateIfRightSide(0.4));
//         captureRegion.addVertex(0.2, swingSide.negateIfRightSide(0.4));
         captureRegion.update();

         captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

         //         testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

         if (PLOT_RESULTS)
         {
            YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
            YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
            YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 40, registry);
            YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 50, registry);
            YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 60, registry);
            YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 80, registry);
            YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

            YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
            YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
            YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
            YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
            YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
            YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

            graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

            SimulationConstructionSet2 scs = new SimulationConstructionSet2();
            scs.addRegistry(registry);


            MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                               () -> scs.simulateNow(1),
                                                                                               registry,
                                                                                               graphicsListRegistry);

            scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));

            //         multiStepRegionCalculator.attachVisualizer(visualizer);

            scs.startSimulationThread();

            yoOneStepRegion.setMatchingFrame(captureRegion, false);

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);

            YoVariableChangedListener updatedListener = v ->
            {
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

               updateRegions(yoSwingDuration.getDoubleValue(),
                             multiStepRegionCalculator,
                             captureRegion,
                             omega0,
                             swingSide.getOppositeSide(),
                             yoTwoStepRegion,
                             yoThreeStepRegion,
                             yoFourStepRegion,
                             yoFiveStepRegion,
                             yoSixStepRegion);
               scs.simulateNow(1);
            };

            yoUseCrossOverSteps.addListener(updatedListener);
            yoForwardLimit.addListener(updatedListener);
            yoBackwardLimit.addListener(updatedListener);
            yoInnerLimit.addListener(updatedListener);
            yoOuterLimit.addListener(updatedListener);
            yoNominalWidth.addListener(updatedListener);
            yoSwingDuration.addListener(updatedListener);

            scs.simulateNow(1);

            ThreadTools.sleepForever();
         }
      }
   }

   @Test
   public void testPointsInsideFromDataAsALine()
   {
      double forwardLimit = 1.0;
      double backwardLimit = 1.0;
      double innerLimit = 0.075;
      double outerLimit = 1.0;

      double width = 0.3;
      double swingDuration = 0.8;
      boolean useCrossOverSteps = true;

      YoBoolean yoUseCrossOverSteps = new YoBoolean("yoUseCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossOverSteps.set(useCrossOverSteps);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      double omega0 = 3.2;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossOverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      for (RobotSide swingSide : RobotSide.values())
      {

         FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(worldFrame);
         captureRegion.addVertex(1.991670922080258, 0.2786498697757683);
         captureRegion.addVertex(2.3916474477638046, 0.5996120819012615);
         captureRegion.update();

         //         testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

         if (PLOT_RESULTS)
         {
            YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
            YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
            YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 40, registry);
            YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 50, registry);
            YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 60, registry);
            YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 80, registry);
            YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

            YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
            YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
            YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
            YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
            YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
            YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

            graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

            SimulationConstructionSet2 scs = new SimulationConstructionSet2();
            scs.addRegistry(registry);


            MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                               () -> scs.simulateNow(1),
                                                                                               registry,
                                                                                               graphicsListRegistry);

            scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));

            //         multiStepRegionCalculator.attachVisualizer(visualizer);

            scs.startSimulationThread();

            yoOneStepRegion.setMatchingFrame(captureRegion, false);

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);

            YoVariableChangedListener updatedListener = v ->
            {
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

               updateRegions(yoSwingDuration.getDoubleValue(),
                             multiStepRegionCalculator,
                             captureRegion,
                             omega0,
                             swingSide.getOppositeSide(),
                             yoTwoStepRegion,
                             yoThreeStepRegion,
                             yoFourStepRegion,
                             yoFiveStepRegion,
                             yoSixStepRegion);
               scs.simulateNow(1);
            };

            yoUseCrossOverSteps.addListener(updatedListener);
            yoForwardLimit.addListener(updatedListener);
            yoBackwardLimit.addListener(updatedListener);
            yoInnerLimit.addListener(updatedListener);
            yoOuterLimit.addListener(updatedListener);
            yoNominalWidth.addListener(updatedListener);
            yoSwingDuration.addListener(updatedListener);

            scs.simulateNow(1);

            ThreadTools.sleepForever();
         }
      }
   }

   @Test
   public void testPointsInsideFromDataAsALine2()
   {
      double forwardLimit = 1.0;
      double backwardLimit = 1.0;
      double innerLimit = 0.075;
      double outerLimit = 0.7;

      double width = 0.25;
      double swingDuration = 0.7;
      boolean useCrossOverSteps = true;

      YoBoolean yoUseCrossOverSteps = new YoBoolean("yoUseCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossOverSteps.set(useCrossOverSteps);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      double omega0 = 3.2;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossOverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      for (RobotSide swingSide : RobotSide.values())
      {

         FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(worldFrame);
         captureRegion.addVertex(1.979075376941339, 0.4348325958734666);
         captureRegion.addVertex(2.2106156464723465, 0.7705540580420254);
         captureRegion.update();

         //         testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

         if (PLOT_RESULTS)
         {
            YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
            YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
            YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 40, registry);
            YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 50, registry);
            YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 60, registry);
            YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 80, registry);
            YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

            YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
            YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
            YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
            YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
            YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
            YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

            graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
            graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

            SimulationConstructionSet2 scs = new SimulationConstructionSet2();
            scs.addRegistry(registry);


            MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                               () -> scs.simulateNow(1),
                                                                                               registry,
                                                                                               graphicsListRegistry);

            scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));

            //         multiStepRegionCalculator.attachVisualizer(visualizer);

            scs.startSimulationThread();

            yoOneStepRegion.setMatchingFrame(captureRegion, false);

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);

            YoVariableChangedListener updatedListener = v ->
            {
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
               reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

               updateRegions(yoSwingDuration.getDoubleValue(),
                             multiStepRegionCalculator,
                             captureRegion,
                             omega0,
                             swingSide.getOppositeSide(),
                             yoTwoStepRegion,
                             yoThreeStepRegion,
                             yoFourStepRegion,
                             yoFiveStepRegion,
                             yoSixStepRegion);
               scs.simulateNow(1);
            };

            yoUseCrossOverSteps.addListener(updatedListener);
            yoForwardLimit.addListener(updatedListener);
            yoBackwardLimit.addListener(updatedListener);
            yoInnerLimit.addListener(updatedListener);
            yoOuterLimit.addListener(updatedListener);
            yoNominalWidth.addListener(updatedListener);
            yoSwingDuration.addListener(updatedListener);

            scs.simulateNow(1);

            ThreadTools.sleepForever();
         }
      }
   }

   @Test
   public void testPointsInsideCaptureRegion()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;

      YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossoverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.3, -0.2);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

      if (PLOT_RESULTS)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
         YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

         YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
         YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
         YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
         YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
         YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
         YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

         graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

         Robot robot = new Robot("test");
         robot.getRobotsYoRegistry().addChild(registry);

         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                            () -> scs.tickAndUpdate(),
                                                                                            registry,
                                                                                            graphicsListRegistry);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);

         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();

         //         multiStepRegionCalculator.attachVisualizer(visualizer);

         scs.startOnAThread();

         yoOneStepRegion.setMatchingFrame(captureRegion, false);

         updateRegions(yoSwingDuration.getDoubleValue(),
                       multiStepRegionCalculator,
                       captureRegion,
                       omega0,
                       swingSide.getOppositeSide(),
                       yoTwoStepRegion,
                       yoThreeStepRegion,
                       yoFourStepRegion,
                       yoFiveStepRegion,
                       yoSixStepRegion);

         YoVariableChangedListener updatedListener = v ->
         {
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);
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
   }

   @Test
   public void testCaptureRegionIsALine()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;

      YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.1;
      double omega0 = 3.0;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossoverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.0, 0.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.1, -0.05);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(captureRegionCalculator.getCaptureRegion());
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      //      testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

      if (PLOT_RESULTS)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
         YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 100, registry);
         YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 100, registry);

         YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
         YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
         YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
         YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
         YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
         YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

         graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

         Robot robot = new Robot("test");
         robot.getRobotsYoRegistry().addChild(registry);

         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                            () -> scs.tickAndUpdate(),
                                                                                            registry,
                                                                                            graphicsListRegistry);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);

         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();

         //         multiStepRegionCalculator.attachVisualizer(visualizer);

         scs.startOnAThread();

         yoOneStepRegion.setMatchingFrame(captureRegion, false);

         updateRegions(yoSwingDuration.getDoubleValue(),
                       multiStepRegionCalculator,
                       captureRegion,
                       omega0,
                       swingSide.getOppositeSide(),
                       yoTwoStepRegion,
                       yoThreeStepRegion,
                       yoFourStepRegion,
                       yoFiveStepRegion,
                       yoSixStepRegion);

         YoVariableChangedListener updatedListener = v ->
         {
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);
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
   }

   @Test
   public void testCustomRegionSimple()
   {
      double forwardLimit = 0.8;
      double backwardLimit = 0.8;
      double innerLimit = 0.075;
      double outerLimit = 0.6;
      double width = 0.25;
      double swingDuration = 0.6;

      YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      RobotSide swingSide = RobotSide.LEFT;
      double omega0 = 3.0;

      rightAnkleZUpFrame.offset.set(-7.8, -0.689, 0.0);
      rightAnkleZUpFrame.update();

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossoverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(rightAnkleZUpFrame);
      captureRegion.addVertex(-0.4, -0.16);
      captureRegion.addVertex(-1.2, -0.46);
      captureRegion.addVertex(-1.0, -0.61);
      captureRegion.update();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      //      testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

      if (PLOT_RESULTS)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
         YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 20, registry);

         YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
         YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
         YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
         YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
         YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
         YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

         graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

         Robot robot = new Robot("test");
         robot.getRobotsYoRegistry().addChild(registry);

         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                            () -> scs.tickAndUpdate(),
                                                                                            registry,
                                                                                            graphicsListRegistry);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);

         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();

         //         multiStepRegionCalculator.attachVisualizer(visualizer);

         scs.startOnAThread();

         yoOneStepRegion.setMatchingFrame(captureRegion, false);

         updateRegions(yoSwingDuration.getDoubleValue(),
                       multiStepRegionCalculator,
                       captureRegion,
                       omega0,
                       swingSide.getOppositeSide(),
                       yoTwoStepRegion,
                       yoThreeStepRegion,
                       yoFourStepRegion,
                       yoFiveStepRegion,
                       yoSixStepRegion);

         YoVariableChangedListener updatedListener = v ->
         {
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);
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
   }

   @Disabled
   @Test
   public void testCaptureRegionIsAPoint()
   {
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;

      YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

      RobotSide swingSide = RobotSide.RIGHT;
      double omega0 = 3.0;

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        yoUseCrossoverSteps,
                                                                                                        registry);

      new DefaultParameterReader().readParametersInRegistry(registry);

      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
      reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

      FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D(ankleZUpFrames.get(swingSide.getOppositeSide()));
      captureRegion.addVertex(0.3, 0.3);
      captureRegion.update();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      testTheRegions(multiStepRegionCalculator, captureRegion, swingDuration, omega0, kinematicStepRange, swingSide.getOppositeSide());

      if (PLOT_RESULTS)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         YoFrameConvexPolygon2D yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);
         YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoThreeStepRegion = new YoFrameConvexPolygon2D("threeStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoFourStepRegion = new YoFrameConvexPolygon2D("fourStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoFiveStepRegion = new YoFrameConvexPolygon2D("fiveStepRegion", worldFrame, 20, registry);
         YoFrameConvexPolygon2D yoSixStepRegion = new YoFrameConvexPolygon2D("sixStepRegion", worldFrame, 20, registry);

         YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegion", yoOneStepRegion, Color.green, false);
         YoArtifactPolygon twoStepRegionGraphic = new YoArtifactPolygon("twoStepRegion", yoTwoStepRegion, Color.blue, false);
         YoArtifactPolygon threeStepRegionGraphic = new YoArtifactPolygon("threeStepRegion", yoThreeStepRegion, Color.red, false);
         YoArtifactPolygon fourStepRegionGraphic = new YoArtifactPolygon("fourStepRegion", yoFourStepRegion, Color.green, false);
         YoArtifactPolygon fiveStepRegionGraphic = new YoArtifactPolygon("fiveStepRegion", yoFiveStepRegion, Color.blue, false);
         YoArtifactPolygon sixStepRegionGraphic = new YoArtifactPolygon("sixStepRegion", yoSixStepRegion, Color.red, false);

         graphicsListRegistry.registerArtifact("test", oneStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", twoStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", threeStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fourStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", fiveStepRegionGraphic);
         graphicsListRegistry.registerArtifact("test", sixStepRegionGraphic);

         Robot robot = new Robot("test");
         robot.getRobotsYoRegistry().addChild(registry);

         SimulationConstructionSet scs = new SimulationConstructionSet(robot);

         MultiStepCaptureRegionVisualizer visualizer = new MultiStepCaptureRegionVisualizer(multiStepRegionCalculator,
                                                                                            () -> scs.tickAndUpdate(),
                                                                                            registry,
                                                                                            graphicsListRegistry);

         scs.addYoGraphicsListRegistry(graphicsListRegistry);

         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
         plotterFactory.createOverheadPlotter();

         //         multiStepRegionCalculator.attachVisualizer(visualizer);

         scs.startOnAThread();

         yoOneStepRegion.setMatchingFrame(captureRegion, false);

         updateRegions(yoSwingDuration.getDoubleValue(),
                       multiStepRegionCalculator,
                       captureRegion,
                       omega0,
                       swingSide.getOppositeSide(),
                       yoTwoStepRegion,
                       yoThreeStepRegion,
                       yoFourStepRegion,
                       yoFiveStepRegion,
                       yoSixStepRegion);

         YoVariableChangedListener updatedListener = v ->
         {
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.LEFT, new FramePose3D());
            reachabilityConstraint.initializeReachabilityConstraint(RobotSide.RIGHT, new FramePose3D());

            updateRegions(yoSwingDuration.getDoubleValue(),
                          multiStepRegionCalculator,
                          captureRegion,
                          omega0,
                          swingSide.getOppositeSide(),
                          yoTwoStepRegion,
                          yoThreeStepRegion,
                          yoFourStepRegion,
                          yoFiveStepRegion,
                          yoSixStepRegion);
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
   }

   public void visualizeForPaper()
   {
      double footWidth = 0.1;
      double footLength = 0.2;
      double kinematicStepRange = 1.0;
      double forwardLimit = 1.0;
      double backwardLimit = 0.8;
      double innerLimit = 0.05;
      double outerLimit = 0.6;
      double width = 0.3;
      double swingDuration = 0.6;

      YoBoolean yoUseCrossoverSteps = new YoBoolean("useCrossOverSteps", registry);
      YoDouble yoForwardLimit = new YoDouble("forwardLimit", registry);
      YoDouble yoBackwardLimit = new YoDouble("backwardLimit", registry);
      YoDouble yoInnerLimit = new YoDouble("innerLimit", registry);
      YoDouble yoOuterLimit = new YoDouble("outerLimit", registry);
      YoDouble yoNominalWidth = new YoDouble("nominalWidth", registry);
      YoDouble yoSwingDuration = new YoDouble("swingDuration", registry);

      yoUseCrossoverSteps.set(false);
      yoForwardLimit.set(forwardLimit);
      yoBackwardLimit.set(backwardLimit);
      yoInnerLimit.set(innerLimit);
      yoOuterLimit.set(outerLimit);
      yoNominalWidth.set(width);
      yoSwingDuration.set(swingDuration);

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
      MultiStepCaptureRegionCalculator multiStepRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraint,
                                                                                                        () -> false,
                                                                                                        registry);


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

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

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

      if (PLOT_RESULTS)
      {
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

         int numberOfRegions = 6;
         List<YoFrameConvexPolygon2D> trueCaptureRegions = createRegionPool("true", numberOfRegions, registry);
         List<YoFrameConvexPolygon2D> simpleCaptureRegions = createRegionPool("simple", numberOfRegions, registry);
         List<YoFrameConvexPolygon2D> crossOverCaptureRegions = createRegionPool("crossOver", numberOfRegions, registry);
         graphicsListRegistry.registerArtifactList(createRegionGraphics("true", trueCaptureRegions));
         graphicsListRegistry.registerArtifactList(createRegionGraphics("simple", simpleCaptureRegions));
         graphicsListRegistry.registerArtifactList(createRegionGraphics("crossOver", crossOverCaptureRegions));

         YoFrameConvexPolygon2D regularReachability = new YoFrameConvexPolygon2D("regularReachability", worldFrame, 20, registry);
         YoFrameConvexPolygon2D crossOverReachability = new YoFrameConvexPolygon2D("CrossOVerReachability", worldFrame, 20, registry);
         regularReachability.setMatchingFrame(reachabilityConstraint.getReachabilityPolygonInFootFrame(RobotSide.LEFT), false);
         crossOverReachability.setMatchingFrame(crossOverReachabilityConstraint.getTotalReachabilityHull(RobotSide.LEFT), false);
         graphicsListRegistry.registerArtifact("regular", new YoArtifactPolygon("regularReachability", regularReachability, Color.blue, false));
         graphicsListRegistry.registerArtifact("crossover", new YoArtifactPolygon("crossOverReachability", crossOverReachability, Color.blue, false));

         List<YoFrameConvexPolygon2D> trueMultiStepRegions = new ArrayList<>();
         for (int i = 1; i < trueCaptureRegions.size(); i++)
            trueMultiStepRegions.add(trueCaptureRegions.get(i));
         List<YoFrameConvexPolygon2D> simpleMultiStepRegions = new ArrayList<>();
         for (int i = 1; i < simpleCaptureRegions.size(); i++)
            simpleMultiStepRegions.add(simpleCaptureRegions.get(i));
         List<YoFrameConvexPolygon2D> crossOverMultiStepRegions = new ArrayList<>();
         for (int i = 1; i < crossOverCaptureRegions.size(); i++)
            crossOverMultiStepRegions.add(crossOverCaptureRegions.get(i));

         Robot robot = new Robot("test");
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
   }

   private static List<YoFrameConvexPolygon2D> createRegionPool(String prefix, int regionsTomake, YoRegistry registry)
   {
      List<YoFrameConvexPolygon2D> regions = new ArrayList<>();

      for (int i = 0; i < regionsTomake; i++)
      {
         regions.add(new YoFrameConvexPolygon2D(prefix + "StepRegion" + (i + 1), worldFrame, 100, registry));
      }

      return regions;
   }

   private static ArtifactList createRegionGraphics(String prefix, List<YoFrameConvexPolygon2D> regions)
   {
      ArtifactList artifactList = new ArtifactList(prefix + "regions");
      for (int i = 0; i < regions.size(); i++)
      {
         int colorIdx = i % 3;
         Color color;
         if (colorIdx == 0)
            color = Color.green;
         else if (colorIdx == 1)
            color = Color.red;
         else
            color = Color.yellow;
         artifactList.add(new YoArtifactPolygon(prefix + "StepRegion" + (i + 1), regions.get(i), color, false));
      }

      return artifactList;
   }

   private void testTheRegions(MultiStepCaptureRegionCalculator multiStepRegionCalculator,
                               FrameConvexPolygon2DReadOnly captureRegion,
                               double swingDuration,
                               double omega0,
                               double kinematicStepRange,
                               RobotSide stanceSide)
   {
      multiStepRegionCalculator.compute(stanceSide, captureRegion, swingDuration, omega0, 1);
      FrameConvexPolygon2DReadOnly oneStepRegion = new FrameConvexPolygon2D(multiStepRegionCalculator.getCaptureRegion());

      EuclidCoreTestTools.assertEquals(captureRegion, oneStepRegion, 1e-5);

      // check that the polygons increase in size as you increase the steps
      for (int i = 2; i < 6; i++)
      {
         multiStepRegionCalculator.compute(stanceSide, captureRegion, swingDuration, omega0, i);
         FrameConvexPolygon2DReadOnly biggerRegion = new FrameConvexPolygon2D(multiStepRegionCalculator.getCaptureRegion());

         for (int vertexId = 0; vertexId < captureRegion.getNumberOfVertices(); vertexId++)
         {
            double distanceInside = biggerRegion.signedDistance(captureRegion.getVertex(vertexId));
            assertTrue("Distance outside is " + distanceInside, distanceInside < 5e-3);
         }
      }

      // check that it's inside the region that doesn't consider reachability limits
      double exponential = Math.exp(-omega0 * swingDuration);
      double scale = exponential;
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      FrameConvexPolygon2D nStepRegionBound = new FrameConvexPolygon2D(oneStepRegion);
      for (int i = 2; i < 6; i++)
      {
         multiStepRegionCalculator.compute(stanceSide, captureRegion, swingDuration, omega0, i);
         FrameConvexPolygon2DReadOnly interiorRegion = new FrameConvexPolygon2D(multiStepRegionCalculator.getCaptureRegion());

         FrameConvexPolygon2D scaledPolygon = new FrameConvexPolygon2D();
         scaler.scaleConvexPolygon(nStepRegionBound, -scale * kinematicStepRange, scaledPolygon);

         for (int vertexId = 0; vertexId < captureRegion.getNumberOfVertices(); vertexId++)
         {
            double distanceInside = scaledPolygon.signedDistance(interiorRegion.getVertex(vertexId));
            assertTrue("Distance outside is " + distanceInside, distanceInside < 5e-3);
         }

         scale *= exponential;

         nStepRegionBound.set(scaledPolygon);
      }
   }

   private void updateRegions(double swingDuration,
                              MultiStepCaptureRegionCalculator calculator,
                              FrameConvexPolygon2DReadOnly captureRegion,
                              double omega,
                              RobotSide stanceSide,
                              YoFrameConvexPolygon2D... polygons)
   {
      int i = 1;
      for (YoFrameConvexPolygon2D polygon : polygons)
      {
         i++;
         calculator.compute(stanceSide, captureRegion, swingDuration, omega, i);
         polygon.setMatchingFrame(calculator.getCaptureRegion(), false);
      }
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
                                      List<YoFrameConvexPolygon2D> truePolygons,
                                      List<YoFrameConvexPolygon2D> simplePolygons,
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
         this.offset.set(offset);
      }
   }

   private void waitForButtonOrPause(FrameGeometryTestFrame testFrame)
   {
      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      else
         pauseOneSecond();
   }

   private void pauseOneSecond()
   {
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException ex)
      {
      }
   }

   private static void setupVisualizer()
   {
      Robot robot = new Robot("CaptureRegionViz");
      double footLength = 0.255;
      double footBack = 0.09;
      double footForward = footLength - footBack;
      double midFootAnkleXOffset = footForward - footLength / 2.0;
      double footWidth = 0.095;
      double kinematicStepRange = 0.6;
      final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      final SideDependentList<FrameConvexPolygon2D> footPolygons = new SideDependentList<>();
      final SideDependentList<YoFrameConvexPolygon2D> yoFootPolygons = new SideDependentList<>();
      YoRegistry registry = robot.getRobotsYoRegistry();
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      final SideDependentList<YoArtifactPolygon> footArtifacts = new SideDependentList<>();
      for (final RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame ankleZUpFrame = new ReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + "AnkleZUpFrame", worldFrame)
         {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               transformToParent.getTranslation().set(new Vector3D(0.0, robotSide.negateIfRightSide(0.15), 0.0));
            }
         };
         ankleZUpFrame.update();
         ankleZUpFrames.put(robotSide, ankleZUpFrame);

         FrameConvexPolygon2D footConvexPolygon2d = new FrameConvexPolygon2D(ankleZUpFrame);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footForward, -footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, footForward, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footBack, footWidth / 2.0);
         footConvexPolygon2d.addVertex(ankleZUpFrame, -footBack, -footWidth / 2.0);
         footConvexPolygon2d.update();
         footPolygons.put(robotSide, footConvexPolygon2d);

         YoFrameConvexPolygon2D yoFootPolygon = new YoFrameConvexPolygon2D(robotSide.getCamelCaseNameForStartOfExpression() + "Foot",
                                                                           "",
                                                                           worldFrame,
                                                                           4,
                                                                           registry);
         footConvexPolygon2d.changeFrame(worldFrame);
         yoFootPolygon.set(footConvexPolygon2d);
         footConvexPolygon2d.changeFrame(ankleZUpFrame);
         yoFootPolygons.put(robotSide, yoFootPolygon);
         Color footColor;
         if (robotSide == RobotSide.LEFT)
            footColor = Color.pink;
         else
            footColor = Color.green;
         YoArtifactPolygon footArtifact = new YoArtifactPolygon(robotSide.getCamelCaseNameForStartOfExpression(), yoFootPolygon, footColor, false);
         yoGraphicsListRegistry.registerArtifact("Feet", footArtifact);
         footArtifacts.put(robotSide, footArtifact);
      }
      final OneStepCaptureRegionCalculator oneStepCaptureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                               kinematicStepRange,
                                                                                                               ankleZUpFrames,
                                                                                                               registry,
                                                                                                               null);

      final YoFrameConvexPolygon2D yoCaptureRegion = new YoFrameConvexPolygon2D("captureRegion", "", worldFrame, 50, registry);
      YoArtifactPolygon captureRegionArtifact = new YoArtifactPolygon("CaptureRegion", yoCaptureRegion, Color.BLACK, false);
      yoGraphicsListRegistry.registerArtifact("Capture", captureRegionArtifact);
      final YoEnum<RobotSide> yoSupportSide = new YoEnum<>("supportSide", registry, RobotSide.class);
      final YoDouble swingTimeRemaining = new YoDouble("swingTimeRemaining", registry);
      final YoFramePoint2D yoICP = new YoFramePoint2D("ICP", worldFrame, registry);
      yoGraphicsListRegistry.registerArtifact("ICP", new YoGraphicPosition("ICP", yoICP, 0.02, YoAppearance.Blue(), GraphicType.CROSS).createArtifact());
      final double omega0 = 3.4;

      final SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      YoVariableChangedListener variableChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            FramePoint2D icp = new FramePoint2D(yoICP);
            RobotSide supportSide = yoSupportSide.getEnumValue();
            yoFootPolygons.get(supportSide.getOppositeSide()).clear();
            footPolygons.get(supportSide).changeFrame(worldFrame);
            yoFootPolygons.get(supportSide).set(footPolygons.get(supportSide));
            footPolygons.get(supportSide).changeFrame(ankleZUpFrames.get(supportSide));
            oneStepCaptureRegionCalculator.calculateCaptureRegion(supportSide.getOppositeSide(),
                                                                  swingTimeRemaining.getDoubleValue(),
                                                                  icp,
                                                                  omega0,
                                                                  footPolygons.get(supportSide));

            FrameConvexPolygon2D frameConvexPolygon2d = new FrameConvexPolygon2D();
            frameConvexPolygon2d.setIncludingFrame(oneStepCaptureRegionCalculator.getCaptureRegion());
            frameConvexPolygon2d.changeFrame(worldFrame);
            yoCaptureRegion.set(frameConvexPolygon2d);

            simulationOverheadPlotter.update();
         }
      };
      swingTimeRemaining.addListener(variableChangedListener);
      yoICP.attachVariableChangedListener(variableChangedListener);
      yoSupportSide.addListener(variableChangedListener);

      swingTimeRemaining.set(0.3);
      yoICP.set(0.1, 0.2);
      variableChangedListener.changed(null);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      //      simulationOverheadPlotter.setDrawHistory(false);

      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, plotterName, true);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend", false);

      yoGraphicsListRegistry.update();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, false);
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
      Thread myThread = new Thread(scs);
      myThread.start();
   }

   @Test
   public void testIsSharedVertex()
   {
      Point2D squareTopLeft = new Point2D(1.0, 1.0);
      Point2D squareTopRight = new Point2D(1.0, -1.0);
      Point2D squareBottomRight = new Point2D(-1.0, -1.0);
      Point2D squareBottomLeft = new Point2D(-1.0, 1.0);

      Point2D diamondTop = new Point2D(1.0, 0.0);
      Point2D diamondRight = new Point2D(0.0, -1.0);
      Point2D diamondBottom = new Point2D(-1.0, 0.0);
      Point2D diamondLeft = new Point2D(0.0, 1.0);

      // check the right side of the diamond is shared with the top left of the square
      assertTrue(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareBottomLeft,
                                                                                      squareTopLeft,
                                                                                      squareTopRight,
                                                                                      diamondTop,
                                                                                      diamondRight,
                                                                                      diamondBottom));
      // check the bottom side of the diamond is shared with the top left of the square
      assertTrue(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareBottomLeft,
                                                                                      squareTopLeft,
                                                                                      squareTopRight,
                                                                                      diamondRight,
                                                                                      diamondBottom,
                                                                                      diamondLeft));
      // check the left side of the diamond is not shared with the top left of the square without intersection
      assertFalse(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareBottomLeft,
                                                                                       squareTopLeft,
                                                                                       squareTopRight,
                                                                                       diamondBottom,
                                                                                       diamondLeft,
                                                                                       diamondTop));
      // check the top side of the diamond is not shared with the top left of the square without intersection
      assertFalse(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareBottomLeft,
                                                                                       squareTopLeft,
                                                                                       squareTopRight,
                                                                                       diamondLeft,
                                                                                       diamondTop,
                                                                                       diamondRight));

      // check the bottom side of the diamond is shared with the top right of the square
      assertTrue(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareTopLeft,
                                                                                      squareTopRight,
                                                                                      squareBottomRight,
                                                                                      diamondRight,
                                                                                      diamondBottom,
                                                                                      diamondLeft));
      // check the left side of the diamond is shared with the top right of the square
      assertTrue(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareTopLeft,
                                                                                      squareTopRight,
                                                                                      squareBottomRight,
                                                                                      diamondBottom,
                                                                                      diamondLeft,
                                                                                      diamondTop));
      // check the top side of the diamond is not shared with the top right of the square without intersection
      assertFalse(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareTopLeft,
                                                                                       squareTopRight,
                                                                                       squareBottomRight,
                                                                                       diamondLeft,
                                                                                       diamondTop,
                                                                                       diamondRight));
      // check the right side of the diamond is not shared with the top left of the square without intersection
      assertFalse(MultiStepCaptureRegionCalculator.isPointASharedNonIntersectingVertex(squareTopLeft,
                                                                                       squareTopRight,
                                                                                       squareBottomRight,
                                                                                       diamondTop,
                                                                                       diamondRight,
                                                                                       diamondLeft));
   }

   @Test
   public void testIsRayPointingInside()
   {
      Point2D squareTopLeft = new Point2D(1.0, 1.0);
      Point2D squareTopRight = new Point2D(1.0, -1.0);
      Point2D squareBottomRight = new Point2D(-1.0, -1.0);
      Point2D squareBottomLeft = new Point2D(-1.0, 1.0);


      // check the right side of the diamond is shared with the top left of the square
      assertTrue(MultiStepCaptureRegionCalculator.isRayPointingToTheInside(squareBottomLeft,
                                                                           squareTopLeft,
                                                                           squareTopRight,
                                                                           squareTopLeft,
                                                                           squareBottomRight));
      // check the top side of the diamond is not shared with the top left of the square without intersection
      assertFalse(MultiStepCaptureRegionCalculator.isRayPointingToTheInside(squareBottomLeft,
                                                                                       squareTopLeft,
                                                                                       squareTopRight,
                                                                                       squareTopLeft,
                                                                                       new Point2D(2.0, 2.0)));

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
      MultiStepCaptureRegionCalculatorTest test = new MultiStepCaptureRegionCalculatorTest();
      test.visualizeForPaper();
   }
}
