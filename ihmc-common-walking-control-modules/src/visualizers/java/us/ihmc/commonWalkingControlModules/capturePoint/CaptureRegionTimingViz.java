package us.ihmc.commonWalkingControlModules.capturePoint;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifact;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.util.ArrayList;

public class CaptureRegionTimingViz
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);

   private final YoRegistry registry = new YoRegistry("CaptureRegionCalculatorTest");

   public CaptureRegionTimingViz()
   {
      // do not change parameters
      // expected results are pre-calculated
      double footWidth = 0.1;
      double footLength = 0.2;
      double kineamaticStepRange = 1.25;

      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth,
                                                                                                  kineamaticStepRange,
                                                                                                  ankleZUpFrames,
                                                                                                  registry,
                                                                                                  null);

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      listOfPoints.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      FrameConvexPolygon2D supportFootPolygon = new FrameConvexPolygon2D(worldFrame, Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      FramePoint2D icp = new FramePoint2D(worldFrame, 0.17, 0.1);


      YoFramePoint2D yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);

      YoFrameConvexPolygon2D yoFoot = new YoFrameConvexPolygon2D("foot", worldFrame, 4, registry);
      YoFrameConvexPolygon2D yoCaptureRegion1 = new YoFrameConvexPolygon2D("captureRegion1", worldFrame, 20, registry);
      YoFrameConvexPolygon2D yoCaptureRegion2 = new YoFrameConvexPolygon2D("captureRegion2", worldFrame, 20, registry);
      YoFrameConvexPolygon2D yoCaptureRegion3 = new YoFrameConvexPolygon2D("captureRegion3", worldFrame, 20, registry);
      YoFrameConvexPolygon2D yoCaptureRegion4 = new YoFrameConvexPolygon2D("captureRegion4", worldFrame, 20, registry);

      YoGraphicPosition yoCapturePointViz = new YoGraphicPosition("capture point", yoCapturePoint, 0.02, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);

      YoArtifactPolygon footArtifact = new YoArtifactPolygon("fOOT aRTIFACT", yoFoot, Color.BLUE, false);
      YoArtifactPolygon polygonArtifact1 = new YoArtifactPolygon("Capture Region 1", yoCaptureRegion1, Color.GREEN, false);
      YoArtifactPolygon polygonArtifact2 = new YoArtifactPolygon("Capture Region 2", yoCaptureRegion2, Color.GREEN, false);
      YoArtifactPolygon polygonArtifact3 = new YoArtifactPolygon("Capture Region 3", yoCaptureRegion3, Color.GREEN, false);
      YoArtifactPolygon polygonArtifact4 = new YoArtifactPolygon("Capture Region 4", yoCaptureRegion4, Color.GREEN, false);

      YoArtifactText text1 = new YoArtifactText("text 1", "0.2s", Color.BLACK);
      YoArtifactText text2 = new YoArtifactText("text 2", "0.5s", Color.RED);
      YoArtifactText text3 = new YoArtifactText("text 3", "0.7s", Color.RED);
      YoArtifactText text4 = new YoArtifactText("text 4", "1.0s", Color.RED);
      text1.setTextPosition(0.22, 0.15);
      text2.setTextPosition(0.42, 0.3);
      text3.setTextPosition(0.7, 0.48);
      text4.setTextPosition(1.0, 0.6);

      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), text1);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), text2);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), text3);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), text4);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), yoCapturePointViz.createArtifact());
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), footArtifact);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact1);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact2);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact3);
      graphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact4);


      yoCapturePoint.set(icp);
      yoFoot.setMatchingFrame(supportFootPolygon, false);

      captureRegionCalculator.calculateCaptureRegion(swingSide, 0.2, icp, omega0, supportFootPolygon);
      yoCaptureRegion1.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);
      captureRegionCalculator.calculateCaptureRegion(swingSide, 0.5, icp, omega0, supportFootPolygon);
      yoCaptureRegion2.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);
      captureRegionCalculator.calculateCaptureRegion(swingSide, 0.7, icp, omega0, supportFootPolygon);
      yoCaptureRegion3.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);
      captureRegionCalculator.calculateCaptureRegion(swingSide, 1.0, icp, omega0, supportFootPolygon);
      yoCaptureRegion4.setMatchingFrame(captureRegionCalculator.getCaptureRegion(), false);

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      scs.addYoGraphicsListRegistry(graphicsListRegistry);

      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      scs.tickAndUpdate();

      ThreadTools.sleepForever();
   }

   private class SimpleAnkleZUpReferenceFrame extends ReferenceFrame
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
   }

   private static class YoArtifactText extends YoArtifact
   {
      private final String prefix;
      private final FramePoint2D textPosition = new FramePoint2D();

      public YoArtifactText(String name, String prefix, Color color)
      {
         super(name, new double[] {0.0}, color);

         this.prefix = prefix;
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
         drawInternal(graphics);
      }

      @Override
      public YoArtifact duplicate(YoRegistry newRegistry)
      {
         return null;
      }

      @Override
      public void draw(Graphics2DAdapter graphics)
      {
         drawInternal(graphics);
      }

      private void drawInternal(Graphics2DAdapter graphics)
      {
         graphics.setColor(color);
         graphics.drawString(prefix, textPosition);
      }

      @Override
      public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
      {
      }
   }

   public static void main(String[] args)
   {
      new CaptureRegionTimingViz();
   }
}
