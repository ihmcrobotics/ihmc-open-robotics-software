package us.ihmc.quadrupedBasics.supportPolygon;

import java.awt.Color;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSupportPolygonVisualizer implements RobotController
{
   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;
   
   private final YoRegistry registry = new YoRegistry("footChooserViz");
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   private final SimulationConstructionSet scs;

   private final Robot robot;
   private final FloatingJoint rootJoint;

   private final QuadrantDependentList<YoFramePoint3D> vertices = new QuadrantDependentList<YoFramePoint3D>();
   private QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final YoFrameConvexPolygon2D currentSupportPolygon = new YoFrameConvexPolygon2D("supportPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoArtifactPolygon currentQuadSupportPolygonArtifact = new YoArtifactPolygon("supportPolygonArtifact", currentSupportPolygon, Color.blue, false);
   
   private final YoFramePoint3D centerOfMass = new YoFramePoint3D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCenterOfPressure = new YoFramePoint3D("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D snappedDesiredCenterOfPressure = new YoFramePoint3D("snappedDesiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMassViz", centerOfMass, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition desiredCenterOfPressureViz = new YoGraphicPosition("desiredCenterOfPressureViz", desiredCenterOfPressure, 0.01, YoAppearance.DarkSlateBlue(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition snappedDesiredCenterOfPressureViz = new YoGraphicPosition("snappedDesiredCenterOfPressureViz", snappedDesiredCenterOfPressure, 0.01, YoAppearance.Orange(), GraphicType.BALL_WITH_ROTATED_CROSS);
   
   private final QuadrupedSupportPolygon tempCommonSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempPolygon = new QuadrupedSupportPolygon();

   private final YoFramePoint3D centroid = new YoFramePoint3D("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D weightedCentroid = new YoFramePoint3D("weightedCentroid", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint3D centroidFramePoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FramePoint3D weightedCentroidFramePoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final YoGraphicPosition centroidGraphic = new YoGraphicPosition("centroidGraphic", centroid, 0.03, YoAppearance.Chartreuse());
   private final YoGraphicPosition weightedCentroidGraphic = new YoGraphicPosition("weightedCentroidGraphic", weightedCentroid, 0.01, YoAppearance.Chocolate());

   private final YoFramePoint3D circleCenter = new YoFramePoint3D("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.03, YoAppearance.Green());

   private final YoDouble inscribedCircleRadius = new YoDouble("inscribedCircleRadius", registry);
   private final YoArtifactOval inscribedCircle = new YoArtifactOval("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);
   
   private final YoFramePoint3D miniCircleCenter = new YoFramePoint3D("miniCircleCenter", ReferenceFrame.getWorldFrame(), registry);
   
//   private final YoBoolean miniCircleRadiusSuccess = new YoBoolean("miniCircleRadiusSuccess", registry);
   private final YoDouble miniCircleRadius = new YoDouble("miniCircleRadius", registry);
   private final YoArtifactOval miniCircle = new YoArtifactOval("miniCircle", miniCircleCenter, miniCircleRadius, Color.YELLOW);
   
   private YoDouble bodyHeadingAngle = new YoDouble("bodyHeadingAngle", registry);

   public QuadrupedSupportPolygonVisualizer()
   {
      miniCircleRadius.set(0.2);
      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3D(), robot);
      
      FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5, 0.0, 0.0);
      FramePoint3D origin = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5, -1.0, 2.0);
      
      vertices.set(RobotQuadrant.FRONT_LEFT, new YoFramePoint3D(RobotQuadrant.FRONT_LEFT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.FRONT_RIGHT, new YoFramePoint3D(RobotQuadrant.FRONT_RIGHT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.HIND_RIGHT, new YoFramePoint3D(RobotQuadrant.HIND_RIGHT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      vertices.set(RobotQuadrant.HIND_LEFT, new YoFramePoint3D(RobotQuadrant.HIND_LEFT.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry));
      
      vertices.get(RobotQuadrant.FRONT_LEFT).set(0.0, 1.0, 0.0);
      vertices.get(RobotQuadrant.FRONT_RIGHT).set(1.0, 1.0, 0.0);
      vertices.get(RobotQuadrant.HIND_RIGHT).set(1.0, 0.0, 0.0);
      vertices.get(RobotQuadrant.HIND_LEFT).set(0.0, 0.0, 0.0);
      
      vertices.get(RobotQuadrant.FRONT_LEFT).add(origin);
      vertices.get(RobotQuadrant.FRONT_RIGHT).add(origin);
      vertices.get(RobotQuadrant.HIND_RIGHT).add(origin);
      vertices.get(RobotQuadrant.HIND_LEFT).add(origin);
      
      QuadrupedSupportPolygon firstQuadrupedPolygon = new QuadrupedSupportPolygon();
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, new FramePoint3D(WORLD, 0.0, 2.0, 0.0));
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_RIGHT, new FramePoint3D(WORLD, 1.0, 2.0, 0.0));
      firstQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_LEFT, new FramePoint3D(WORLD, 0.0, 0.0, 0.0));
      
      QuadrupedSupportPolygon secondQuadrupedPolygon = new QuadrupedSupportPolygon();
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.FRONT_LEFT, new FramePoint3D(WORLD, 0.0, 2.0, 0.0));
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_RIGHT, new FramePoint3D(WORLD, 1.0, 1.0, 0.0));
      secondQuadrupedPolygon.setFootstep(RobotQuadrant.HIND_LEFT, new FramePoint3D(WORLD, 0.0, 0.0, 0.0));
      
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
      
      robot.getRobotsYoRegistry();
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
   public YoRegistry getYoRegistry()
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
   
   FrameVector3D initialVelocity = new FrameVector3D();
   FrameVector3D intialAcceleration = new FrameVector3D();
   FrameVector3D finalDesiredVelocity = new FrameVector3D();

   @Override
   public void doControl()
   {
      for (RobotQuadrant quadrant : vertices.quadrants())
      {
         YoFramePoint3D vertex = vertices.get(quadrant);
         supportPolygon.setFootstep(quadrant, vertex);
      }
      drawSupportPolygon(supportPolygon, currentSupportPolygon);
      
      supportPolygon.getCentroid(centroidFramePoint);
      centroid.set(centroidFramePoint);
      

      supportPolygon.getCentroidEqualWeightingEnds(weightedCentroidFramePoint);
      weightedCentroid.set(weightedCentroidFramePoint);

//      centerOfMass.set(centroid);
//      desiredCenterOfPressure.set(0.0, 2.0, 0.0);
//      supportPolygon.snapPointToEdgeTowardsInnerPointIfOutside(snappedDesiredCenterOfPressure, desiredCenterOfPressure, centerOfMass);
      
      FramePoint3D centerOfInscribedCircle = new FramePoint3D();
      double radius = supportPolygon.getInCircle2d(centerOfInscribedCircle);
      circleCenter.setX(centerOfInscribedCircle.getX());
      circleCenter.setY(centerOfInscribedCircle.getY());
      inscribedCircleRadius.set(radius);
      
      FramePoint2D centerOfMiniCircle = new FramePoint2D();
      if (supportPolygon.getNumberOfVertices() >= 3)
      {
         supportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(RobotQuadrant.HIND_LEFT, miniCircleRadius.getDoubleValue(), centerOfMiniCircle);

         miniCircleCenter.setX(centerOfMiniCircle.getX());
         miniCircleCenter.setY(centerOfMiniCircle.getY());
      }
      ThreadTools.sleep(10);
   }
   
   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2D yoFramePolygon)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      for(RobotQuadrant quadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         FramePoint3D footstep = supportPolygon.getFootstep(quadrant);
         polygon.addVertex(footstep.getX(), footstep.getY());
      }
      polygon.update();
      yoFramePolygon.set(polygon);
   }

   
   public static void main(String[] args)
   {
      new QuadrupedSupportPolygonVisualizer();
   }
}
