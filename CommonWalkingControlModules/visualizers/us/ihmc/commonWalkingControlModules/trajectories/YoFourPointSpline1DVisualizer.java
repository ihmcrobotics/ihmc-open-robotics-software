package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.thread.ThreadTools;

public class YoFourPointSpline1DVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public YoFourPointSpline1DVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("Visualizer");

      YoFourPointCubicSpline1D fourPointSpline1D = new YoFourPointCubicSpline1D("z", registry);

      double x0 = 0.0;
      double x1 = x0 + 0.5;
      double x2 = x1 + 0.1;
      double x3 = x2 + 0.9;

      double y0 = 0.5;
      double y1 = y0 - 0.25;
      double y2 = y0 - 0.25;
      double y3 = y0;

      Point2D[] points = new Point2D[]{new Point2D(x0, y0), new Point2D(x1, y1), new Point2D(x2, y2), new Point2D(x3, y3)};

      fourPointSpline1D.initialize(new Point2D(x0, y0), new Point2D(x1, y1), new Point2D(x2, y2), new Point2D(x3, y3));

      AppearanceDefinition[] colors = {YoAppearance.Red(), YoAppearance.Purple(), YoAppearance.Blue(), YoAppearance.Yellow()};

      String listName = "visualizer";
      double pointVizScale = 0.01;

      for (int i = 0; i < 4; i++)
      {
         YoFramePoint yoPoint = new YoFramePoint("point" + i, worldFrame, registry);
         yoPoint.set(points[i].getX(), 0.0, points[i].getY());

         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("pointViz" + i, yoPoint, pointVizScale, colors[i]);
         yoGraphicsListRegistry.registerYoGraphic(listName, yoGraphicPosition);
      }

      double numberOfPoints = 100;
      double queryScale = 0.005;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double alpha = i / (numberOfPoints - 1.0);
         double queryPoint = (1.0 - alpha) * x0 + alpha * x3;
         fourPointSpline1D.compute(queryPoint);

         YoFramePoint resultPosition = new YoFramePoint("splineZ" + i, worldFrame, registry);
         resultPosition.set(queryPoint, 0.0, fourPointSpline1D.getY());

         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition("splineViz" + i, resultPosition, queryScale, YoAppearance.Black());
         yoGraphicsListRegistry.registerYoGraphic(listName, yoGraphicPosition);
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new YoFourPointSpline1DVisualizer();
   }
}
