package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

import java.awt.Color;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.thread.ThreadTools;

public class QuadrupedSupportPolygonVisualizer implements RobotController
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("footChooserViz");
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   private final SimulationConstructionSet scs;

   private final Robot robot;
   private final FloatingJoint rootJoint;

   private final QuadrantDependentList<YoFramePoint> vertices = new QuadrantDependentList<YoFramePoint>();
   private QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final YoFrameConvexPolygon2d currentSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoArtifactPolygon currentQuadSupportPolygonArtifact = new YoArtifactPolygon("supportPolygonArtifact", currentSupportPolygon, Color.blue, false);
   
   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredCenterOfPressure = new YoFramePoint("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint snappedDesiredCenterOfPressure = new YoFramePoint("snappedDesiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMassViz", centerOfMass, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition desiredCenterOfPressureViz = new YoGraphicPosition("desiredCenterOfPressureViz", desiredCenterOfPressure, 0.01, YoAppearance.DarkSlateBlue(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition snappedDesiredCenterOfPressureViz = new YoGraphicPosition("snappedDesiredCenterOfPressureViz", snappedDesiredCenterOfPressure, 0.01, YoAppearance.Orange(), GraphicType.BALL_WITH_ROTATED_CROSS);
   
   private final QuadrupedSupportPolygon tempCommonSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempPolygon = new QuadrupedSupportPolygon();

   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint weightedCentroid = new YoFramePoint("weightedCentroid", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint centroidFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint weightedCentroidFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final YoGraphicPosition centroidGraphic = new YoGraphicPosition("centroidGraphic", centroid, 0.03, YoAppearance.Chartreuse());
   private final YoGraphicPosition weightedCentroidGraphic = new YoGraphicPosition("weightedCentroidGraphic", weightedCentroid, 0.01, YoAppearance.Chocolate());

   private final YoFramePoint circleCenter = new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.03, YoAppearance.Green());

   private final DoubleYoVariable inscribedCircleRadius = new DoubleYoVariable("inscribedCircleRadius", registry);
   private final YoArtifactOval inscribedCircle = new YoArtifactOval("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);
   
   private final YoFramePoint miniCircleCenter = new YoFramePoint("miniCircleCenter", ReferenceFrame.getWorldFrame(), registry);
   
//   private final BooleanYoVariable miniCircleRadiusSuccess = new BooleanYoVariable("miniCircleRadiusSuccess", registry);
   private final DoubleYoVariable miniCircleRadius = new DoubleYoVariable("miniCircleRadius", registry);
   private final YoArtifactOval miniCircle = new YoArtifactOval("miniCircle", miniCircleCenter, miniCircleRadius, Color.YELLOW);
   
   private DoubleYoVariable bodyHeadingAngle = new DoubleYoVariable("bodyHeadingAngle", registry);

   public QuadrupedSupportPolygonVisualizer()
   {
      miniCircleRadius.set(0.2);
      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3d(), robot);
      
      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.0, 0.0);
      FramePoint origin = new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, -1.0, 2.0);
      
      vertices.set(RobotQuadrant.FRONT_LEFT, new YoFramePoint(RobotQuadrant.FRONT_LEFT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.FRONT_RIGHT, new YoFramePoint(RobotQuadrant.FRONT_RIGHT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.HIND_RIGHT, new YoFramePoint(RobotQuadrant.HIND_RIGHT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.HIND_LEFT, new YoFramePoint(RobotQuadrant.HIND_LEFT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      
      vertices.get(RobotQuadrant.FRONT_LEFT).set(0.0, 1.0, 0.0);
      vertices.get(RobotQuadrant.FRONT_RIGHT).set(1.0, 1.0, 0.0);
      vertices.get(RobotQuadrant.HIND_RIGHT).set(1.0, 0.0, 0.0);
      vertices.get(RobotQuadrant.HIND_LEFT).set(0.0, 0.0, 0.0);
      
      vertices.get(RobotQuadrant.FRONT_LEFT).add(origin);
      vertices.get(RobotQuadrant.FRONT_RIGHT).add(origin);
      vertices.get(RobotQuadrant.HIND_RIGHT).add(origin);
      vertices.get(RobotQuadrant.HIND_LEFT).add(origin);
      
      QuadrupedSupportPolygon firstQuadrupedPolygon = new QuadrupedSupportPolygon();
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, new FramePoint(WORLD, 0.0, 2.0, 0.0));
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_RIGHT, new FramePoint(WORLD, 1.0, 2.0, 0.0));
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_LEFT, new FramePoint(WORLD, 0.0, 0.0, 0.0));
      
      QuadrupedSupportPolygon secondQuadrupedPolygon = new QuadrupedSupportPolygon();
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, new FramePoint(WORLD, 0.0, 2.0, 0.0));
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_RIGHT, new FramePoint(WORLD, 1.0, 1.0, 0.0));
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_LEFT, new FramePoint(WORLD, 0.0, 0.0, 0.0));
      
//      firstQuadrupedPolygon.getCommonSupportPolygon(secondQuadrupedPolygon, commonPolygonToPack , RobotQuadrant.FRONT_RIGHT);
      firstQuadrupedPolygon.getShrunkenCommonTriangle2d(secondQuadrupedPolygon, tempCommonSupportPolygon, tempPolygon, RobotQuadrant.FRONT_RIGHT, 0.1, 0.1, 0.1);
      
      vertices.get(RobotQuadrant.FRONT_LEFT).set(tempCommonSupportPolygon.getFootstep(RobotQuadrant.FRONT_LEFT));
      vertices.get(RobotQuadrant.FRONT_RIGHT).set(tempCommonSupportPolygon.getFootstep(RobotQuadrant.FRONT_RIGHT));
      vertices.remove(RobotQuadrant.HIND_RIGHT);
      vertices.get(RobotQuadrant.HIND_LEFT).set(tempCommonSupportPolygon.getFootstep(RobotQuadrant.HIND_LEFT));
      
//      for(RobotQuadrant quadrant : RobotQuadrant.values)
//      {
//         if(!quadrant.equals(RobotQuadrant.HIND_RIGHT))
//         {
//            YoFramePoint vertex = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry);
//            vertex.set(point);
//            vertices.put(quadrant, vertex);
//            point.yawAboutPoint(origin, point, 2 * Math.PI / 3);
//         }
//      }
      
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      scs = new SimulationConstructionSet();
      scs.setRobot(robot);
      
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistry.registerYoGraphic("centroid", centroidGraphic);
      yoGraphicsListRegistry.registerArtifact("copSnapping", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("copSnapping", desiredCenterOfPressureViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("copSnapping", snappedDesiredCenterOfPressureViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centroid", centroidGraphic.createArtifact());
      
      yoGraphicsListRegistry.registerYoGraphic("weightedCentroid", weightedCentroidGraphic);
      yoGraphicsListRegistry.registerArtifact("weightedCentroid", weightedCentroidGraphic.createArtifact());
      
      yoGraphicsListRegistry.registerArtifact("inscribedCircle", inscribedCircle);
      yoGraphicsListRegistry.registerArtifact("inscribedCircleCenter", circleCenterGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact("miniCircle", miniCircle);
      
      yoGraphicsListRegistry.registerArtifact("currentQuadSupportPolygonArtifact", currentQuadSupportPolygonArtifact);

      boolean showOverheadView = true;
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setVariableNameToTrack("centroidGraphic");
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulateDT, recordFrequency);
      scs.startOnAThread();
      scs.simulate();
   }

   private AppearanceDefinition getYoAppearance(RobotQuadrant robotQuadrant)
   {
      switch (robotQuadrant)
      {
      case FRONT_LEFT:
         return YoAppearance.White();
      case FRONT_RIGHT:
         return YoAppearance.Yellow();
      case HIND_LEFT:
         return YoAppearance.Blue();
      case HIND_RIGHT:
         return YoAppearance.Black();
      default:
         throw new RuntimeException("bad quad");
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "viz";
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
   
   FrameVector initialVelocity = new FrameVector();
   FrameVector intialAcceleration = new FrameVector();
   FrameVector finalDesiredVelocity = new FrameVector();

   @Override
   public void doControl()
   {
      for (RobotQuadrant quadrant : vertices.quadrants())
      {
         YoFramePoint vertex = vertices.get(quadrant);
         supportPolygon.setFootstep(quadrant, vertex.getFrameTuple());
      }
      drawSupportPolygon(supportPolygon, currentSupportPolygon);
      
      supportPolygon.getCentroid(centroidFramePoint);
      centroid.set(centroidFramePoint);
      

      supportPolygon.getCentroidEqualWeightingEnds(weightedCentroidFramePoint);
      weightedCentroid.set(weightedCentroidFramePoint);

//      centerOfMass.set(centroid);
//      desiredCenterOfPressure.set(0.0, 2.0, 0.0);
//      supportPolygon.snapPointToEdgeTowardsInnerPointIfOutside(snappedDesiredCenterOfPressure, desiredCenterOfPressure, centerOfMass);
      
      FramePoint centerOfInscribedCircle = new FramePoint();
      double radius = supportPolygon.getInCircle2d(centerOfInscribedCircle);
      circleCenter.setX(centerOfInscribedCircle.getX());
      circleCenter.setY(centerOfInscribedCircle.getY());
      inscribedCircleRadius.set(radius);
      
      FramePoint2d centerOfMiniCircle = new FramePoint2d();
      if (supportPolygon.size() >= 3)
      {
         supportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant.HIND_LEFT, miniCircleRadius.getDoubleValue(), centerOfMiniCircle);

         miniCircleCenter.setX(centerOfMiniCircle.getX());
         miniCircleCenter.setY(centerOfMiniCircle.getY());
      }
      ThreadTools.sleep(10);
   }
   
   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      for(RobotQuadrant quadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         FramePoint footstep = supportPolygon.getFootstep(quadrant);
         polygon.addVertex(footstep.getX(), footstep.getY());
      }
      polygon.update();
      yoFramePolygon.setConvexPolygon2d(polygon);
   }

   
   public static void main(String[] args)
   {
      new QuadrupedSupportPolygonVisualizer();
   }
}
