package us.ihmc.commonWalkingControlModules.capturePoint;

import org.apache.commons.math3.util.Precision;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifact;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.sql.Ref;

import static us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer.defaultFeetColors;

public class TimeAdjustmentCalculatorVisualizer
{
   private static final double moveDuration = 10.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2D leftFootPolygon;
   private final YoFrameConvexPolygon2D rightFootPolygon;
   private final YoFrameConvexPolygon2D supportPolygon;

   private final YoFramePoint2D desiredICPAtTouchdown;
   private final YoFramePoint2D desiredICP;
   private final YoFramePoint2D currentICP;
   private final YoFramePoint2D perfectCMP;
   private final YoFramePoint2D projectedICP;

   private final YoDouble currentTimeRemaining;
   private final YoDouble adjustedTimeRemaining;
   private final YoDouble adjustmentTime;
   private final YoDouble omega;

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final YoArtifactText timeRemainingText;
   private final YoArtifactText adjustedTimeRemainingText;
   private final YoDouble time;

   public TimeAdjustmentCalculatorVisualizer()
   {
      footPolygon.addVertex(0.1, 0.05);
      footPolygon.addVertex(0.1, -0.05);
      footPolygon.addVertex(-0.1, -0.05);
      footPolygon.addVertex(-0.1, 0.05);
      footPolygon.update();

      Robot robot = new Robot("dummy");
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      leftFootPolygon = new YoFrameConvexPolygon2D("leftFootPolygon", worldFrame, 4, registry);
      rightFootPolygon = new YoFrameConvexPolygon2D("rightFootPolygon", worldFrame, 4, registry);
      supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", worldFrame, 8, registry);

      desiredICPAtTouchdown = new YoFramePoint2D("desiredICPAtTouchdown", worldFrame, registry);
      desiredICP = new YoFramePoint2D("desiredICP", worldFrame, registry);
      currentICP = new YoFramePoint2D("currentICP", worldFrame, registry);
      perfectCMP = new YoFramePoint2D("perfectCMP", worldFrame, registry);
      projectedICP = new YoFramePoint2D("projectedICP", worldFrame, registry);
      currentTimeRemaining = new YoDouble("currentTimeRemaining", registry);
      adjustedTimeRemaining = new YoDouble("adjustedTimeRemaining", registry);
      adjustmentTime = new YoDouble("adjustmentTime", registry);
      omega = new YoDouble("omega", registry);

      currentTimeRemaining.set(0.3);
      omega.set(3.0);

      YoArtifactPolygon leftFootPolygonArtifact = new YoArtifactPolygon("Left Foot Polygon", leftFootPolygon, defaultFeetColors.get(RobotSide.LEFT), false);
      YoArtifactPolygon rightFootPolygonArtifact = new YoArtifactPolygon("Right Foot Polygon", rightFootPolygon, defaultFeetColors.get(RobotSide.RIGHT), false);
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("Combined Polygon", supportPolygon, Color.pink, false);

      timeRemainingText = new YoArtifactText("Time Remaining", "Time Remaining: ", currentTimeRemaining, Color.black);
      adjustedTimeRemainingText = new YoArtifactText("Adjusted Time Remaining", "Adjusted Time Remaining: ", adjustedTimeRemaining, Color.black);
      adjustedTimeRemainingText.setTextPosition(-0.4, -0.27);
      timeRemainingText.setTextPosition(-0.4, -0.3);

      double size = 0.01;
      YoGraphicPosition desiredICPAtTouchdownArtifact = new YoGraphicPosition("Desired ICP At Touchdown", desiredICPAtTouchdown, size, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition desiredICPArtifact = new YoGraphicPosition("Desired ICP", desiredICP, size, YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition currentICPArtifact = new YoGraphicPosition("Current ICP", currentICP, size, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition projectedICPArtifact = new YoGraphicPosition("Project ICP", projectedICP, size, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition perfectCMPArtifact = new YoGraphicPosition("Perfect CMP", perfectCMP, size, YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      graphicsListRegistry.registerArtifact("Time Adjustment", desiredICPAtTouchdownArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("Time Adjustment", desiredICPArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("Time Adjustment", currentICPArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("Time Adjustment", projectedICPArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("Time Adjustment", perfectCMPArtifact.createArtifact());
      graphicsListRegistry.registerArtifact("Time Adjustment", leftFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("Time Adjustment", rightFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("Time Adjustment", supportPolygonArtifact);
      graphicsListRegistry.registerArtifact("Time Adjustment", adjustedTimeRemainingText);
      graphicsListRegistry.registerArtifact("Time Adjustment", timeRemainingText);

      VisualizerController visualizerController = new VisualizerController();
      robot.setController(visualizerController);

      visualizerController.getYoRegistry().addChild(registry);

      double dt = 0.01;
      time = robot.getYoTime();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(dt, 1);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      scs.simulate(moveDuration);
//      scs.stop();
   }

   private class VisualizerController implements RobotController
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      private final TimeAdjustmentCalculator timeAdjustmentCalculator = new TimeAdjustmentCalculator();


      @Override
      public void doControl()
      {
         FrameConvexPolygon2D left = new FrameConvexPolygon2D(worldFrame, footPolygon);
         FrameConvexPolygon2D right = new FrameConvexPolygon2D(worldFrame, footPolygon);
         left.translate(0.3, 0.1);
         right.translate(0.0, -0.1);

         leftFootPolygon.set(left);
         rightFootPolygon.set(right);

         FrameConvexPolygon2D combinedPolygon = new FrameConvexPolygon2D();
         combinedPolygon.addVertices(leftFootPolygon);
         combinedPolygon.addVertices(rightFootPolygon);
         combinedPolygon.update();

         supportPolygon.set(combinedPolygon);

         desiredICPAtTouchdown.set(0.35, 0.08);
         perfectCMP.set(rightFootPolygon.getCentroid());

         double exp = Math.exp(-currentTimeRemaining.getDoubleValue() * omega.getValue());
         desiredICP.interpolate(perfectCMP, desiredICPAtTouchdown, exp);


         moveCurrentICP();

         adjustmentTime.set(timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredICPAtTouchdown, perfectCMP, currentICP, omega.getDoubleValue()));
         projectedICP.set(timeAdjustmentCalculator.getProjectedICPEstimate());

         adjustedTimeRemaining.set(currentTimeRemaining.getDoubleValue() + adjustmentTime.getValue());
      }

      private void moveCurrentICP()
      {
         double localTime = time.getDoubleValue() % moveDuration;
         FrameVector2D motionDirection = new FrameVector2D();
         motionDirection.sub(desiredICPAtTouchdown, perfectCMP);
         FrameVector2D add = new FrameVector2D(motionDirection);
         add.scale(1.0);
         FrameVector2D translate = new FrameVector2D();
         translate.set(add.getY(), -add.getX());
         translate.scale(0.3, translate.length());

         FramePoint2D endOfMove1 = new FramePoint2D(add);

         FramePoint2D endOfMove2 = new FramePoint2D(endOfMove1);
         endOfMove2.add(translate);

         FramePoint2D endOfMove3 = new FramePoint2D(endOfMove2);
         endOfMove3.sub(add);

         FramePoint2D endOfMove4 = new FramePoint2D();
         motionDirection.sub(desiredICPAtTouchdown, perfectCMP);
         double durationForEachMove = moveDuration / 4.0;
         if (localTime < durationForEachMove)
         {
            double alpha = localTime / durationForEachMove;
            currentICP.interpolate(new FramePoint2D(), endOfMove1, alpha);
         }
         else if (localTime < 2.0 * durationForEachMove)
         {
            localTime -= durationForEachMove;
            double alpha = localTime / durationForEachMove;
            currentICP.interpolate(endOfMove1, endOfMove2, alpha);
         }
         else if (localTime < 3.0 * durationForEachMove)
         {
            localTime -= 2.0 * durationForEachMove;
            double alpha = localTime / durationForEachMove;
            currentICP.interpolate(endOfMove2, endOfMove3, alpha);
         }
         else
         {
            localTime -= 3.0 * durationForEachMove;
            double alpha = localTime / durationForEachMove;
            currentICP.interpolate(endOfMove3, endOfMove4, alpha);
         }
      }


      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }

   private static class YoArtifactText extends YoArtifact
   {
      private final String prefix;
      private final YoVariable variable;
      private final FramePoint2D textPosition = new FramePoint2D();

      public YoArtifactText(String name, String prefix, YoVariable variableToDraw, Color color)
      {
         super(name, new double[] {0.0}, color, variableToDraw);

         this.prefix = prefix;
         this.variable = variableToDraw;
      }

      public void setTextPosition(Point2DReadOnly textPosition)
      {
         setTextPosition(textPosition.getX(), textPosition.getY());
      }

      public void setTextPosition(double x, double y)
      {
         this.textPosition.set(x, y);
      }

      @Override
      public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
      {
         drawInternal(graphics, entry[0]);
      }

      @Override
      public YoArtifact duplicate(YoRegistry newRegistry)
      {
         return null;
      }

      @Override
      public void draw(Graphics2DAdapter graphics)
      {
         drawInternal(graphics, variable.getValueAsDouble());
      }

      private void drawInternal(Graphics2DAdapter graphics, double value)
      {
         graphics.drawString(prefix + Precision.round(value, 2), textPosition);
      }

      @Override
      public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
      {
      }
   }

   public static void main(String[] args)
   {
      new TimeAdjustmentCalculatorVisualizer();
   }
}
