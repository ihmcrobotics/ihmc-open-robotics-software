package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;


import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.plotting.Artifact;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.plotting.SimulationOverheadPlotter;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPosition;
import us.ihmc.yoUtilities.math.frames.YoFrameLineSegment2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class PointAndLinePlotter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private SimulationOverheadPlotter simulationOverheadPlotter;

   private int numberOfRegisteredPoints;
   private int numberOfLines;

   private ArrayList<YoGraphicPosition> dynamicGraphicPositions = new ArrayList<YoGraphicPosition>();
   private ArrayList<YoArtifactPosition> dynamicGraphicPositionsArtifactList = new ArrayList<YoArtifactPosition>();

   private ArrayList<Artifact> lineSegmentArtifacts = new ArrayList<Artifact>();

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public PointAndLinePlotter(YoVariableRegistry parentRegistry)
   {
      numberOfRegisteredPoints = 0;
      numberOfLines = 0;

      parentRegistry.addChild(registry);
   }

public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
{
   return yoGraphicsListRegistry;
}

   public void createAndShowOverheadPlotterInSCS(SimulationConstructionSet scs)
   {
      simulationOverheadPlotter = new SimulationOverheadPlotter();
      simulationOverheadPlotter.setDrawHistory(false);

      scs.attachPlaybackListener(simulationOverheadPlotter);
      JPanel simulationOverheadPlotterJPanel = simulationOverheadPlotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(simulationOverheadPlotterJPanel, plotterName);
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();

      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      scs.addExtraJpanel(scrollPane, "Plotter Legend");

      StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
      if (standardSimulationGUI != null) standardSimulationGUI.selectPanel(plotterName);
   }


   public void plotYoFramePoints(String name, ArrayList<YoFramePoint> pointList, AppearanceDefinition appearance, double size)
   {
      for (YoFramePoint yoFramePoint : pointList)
      {
         plotYoFramePoint(name, yoFramePoint, appearance, size);
      }
   }


   public void plotPoints(String name, ArrayList<Point3d> pointList, AppearanceDefinition appearance, double size)
   {
      for (Point3d point3d : pointList)
      {
         plotPoint3d(name, point3d, appearance, size);
      }
   }

   public void plotPoint3ds(String name, ArrayList<Point3d> pointList, AppearanceDefinition appearance, double size)
   {
      for (Point3d point3d : pointList)
      {
         plotPoint3d(name, point3d, appearance, size);
      }
   }

   public void plotPoint3ds(String name, Point3d[] pointList, AppearanceDefinition appearance, double size)
   {
      for (Point3d point3d : pointList)
      {
         plotPoint3d(name, point3d, appearance, size);
      }
   }

   public YoFramePoint plotPoint3d(String name, Point3d point3d, AppearanceDefinition appearance, double size)
   {
      YoFramePoint yoFramePoint = new YoFramePoint(name + numberOfRegisteredPoints, worldFrame, registry);
      yoFramePoint.set(point3d);
      plotYoFramePoint(name, yoFramePoint, appearance, size);
      
      return yoFramePoint;
   }

   public void plotYoFramePoint(String name, YoFramePoint point, AppearanceDefinition appearance, double size)
   {
      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(name + numberOfRegisteredPoints, point, size, appearance);
      dynamicGraphicPositions.add(dynamicGraphicPosition);
      yoGraphicsListRegistry.registerYoGraphic("GraphicsObjects", dynamicGraphicPositions.get(numberOfRegisteredPoints));
      YoArtifactPosition artifact = dynamicGraphicPositions.get(numberOfRegisteredPoints).createArtifact();
      dynamicGraphicPositionsArtifactList.add(artifact);
      yoGraphicsListRegistry.registerArtifact(name + numberOfRegisteredPoints, dynamicGraphicPositionsArtifactList.get(numberOfRegisteredPoints));

      numberOfRegisteredPoints += 1;
   }

   public void plotLineSegment(String name, YoFrameLineSegment2d lineSegment, Color color)
   {
      Artifact lineSegmentArtifact = new YoArtifactLineSegment2d("line" + numberOfLines, lineSegment, color);
      lineSegmentArtifacts.add(lineSegmentArtifact);
      yoGraphicsListRegistry.registerArtifact("line" + numberOfLines, lineSegmentArtifact);

      numberOfLines += 1;
   }

   public void plotLineSegments(String name, ArrayList<YoFrameLineSegment2d> lineSegments, Color color)
   {
      for (YoFrameLineSegment2d lineSegment : lineSegments)
      {
         plotLineSegment(name, lineSegment, color);
      }
   }

   public void addPointsAndLinesToSCS(SimulationConstructionSet scs)
   {
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
   }

   public void addGraphicObjectsAndArtifactsToSCS(SimulationConstructionSet scs)
   {
      yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
   }


   
   public static void setLineSegmentBasedOnStartAndEndPoints(YoFrameLineSegment2d lineSegmentToPack, Point2d startPoint, Point2d endPoint)
   {
      FramePoint2d startFramePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), startPoint.getX(), startPoint.getY());
      FramePoint2d endFramePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), endPoint.getX(), endPoint.getY());

      if (startFramePoint.distanceSquared(endFramePoint) < 1e-6)
         startFramePoint.setX(startFramePoint.getX() + 1e-6);


      FrameLineSegment2d lineSegment = new FrameLineSegment2d(startFramePoint, endFramePoint);
      lineSegmentToPack.setFrameLineSegment2d(lineSegment);
   }

   public static void setLineSegmentBasedOnStartAndEndFramePoints(YoFrameLineSegment2d lineSegmentToPack, FramePoint2d startPoint, FramePoint2d endPoint)
   {
      if (startPoint.distanceSquared(endPoint) < 1e-6)
         startPoint.setX(startPoint.getX() + 1e-6);

      FrameLineSegment2d lineSegment = new FrameLineSegment2d(startPoint, endPoint);
      lineSegmentToPack.setFrameLineSegment2d(lineSegment);
   }
   


   public static void setLineSegmentBasedOnStartAndEndFramePoints(YoFrameLineSegment2d lineSegmentToPack, Point2d startPoint, Point2d endPoint)
   {
      if (startPoint.distanceSquared(endPoint) < 1e-6)
         startPoint.setX(startPoint.getX() + 1e-6);

      FrameLineSegment2d lineSegment = new FrameLineSegment2d(ReferenceFrame.getWorldFrame(), startPoint, endPoint);
      lineSegmentToPack.setFrameLineSegment2d(lineSegment);
   }

   public static void setEndPointGivenStartAndAdditionalVector(YoFramePoint endPointToSet, YoFramePoint startPoint, YoFrameVector additionalVector, double scaling)
   {
      endPointToSet.set(additionalVector);
      endPointToSet.scale(scaling);
      endPointToSet.add(startPoint);
   }
   
   public static void setEndPointGivenStartAndAdditionalVector(YoFramePoint endPointToSet, FramePoint startPoint, FrameVector additionalVector, double scaling)
   {
      endPointToSet.set(additionalVector);
      endPointToSet.scale(scaling);
      endPointToSet.add(startPoint);
   }


   public static void setEndPointGivenStartAndAdditionalVector(YoFramePoint endPointToSet, Point3d startPoint, Vector3d additionalVector, double scaling)
   {
      endPointToSet.set(additionalVector);
      endPointToSet.scale(scaling);
      endPointToSet.add(startPoint);
   }

  
}
