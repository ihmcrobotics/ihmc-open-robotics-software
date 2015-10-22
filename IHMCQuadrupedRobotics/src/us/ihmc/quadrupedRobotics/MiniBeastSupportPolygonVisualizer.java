package us.ihmc.quadrupedRobotics;

import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactCircle;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.tools.thread.ThreadTools;

public class MiniBeastSupportPolygonVisualizer implements RobotController
{
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("footChooserViz");
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   
   private final SimulationConstructionSet scs;

   private final Robot robot;
   private final FloatingJoint rootJoint;

   private final QuadrantDependentList<YoFramePoint> vertices = new QuadrantDependentList<YoFramePoint>();
   private MiniBeastSupportPolygon supportPolygon = new MiniBeastSupportPolygon(); 
   private final YoFrameConvexPolygon2d currentSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoArtifactPolygon currentQuadSupportPolygonArtifact = new YoArtifactPolygon("supportPolygonArtifact", currentSupportPolygon, Color.blue, false);

   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint centroidFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final YoGraphicPosition centroidGraphic = new YoGraphicPosition("centroidGraphic", centroid, 0.03, YoAppearance.Chartreuse());

   private final YoFramePoint circleCenter = new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.03, YoAppearance.Green());

   private final DoubleYoVariable inscribedCircleRadius = new DoubleYoVariable("inscribedCircleRadius", registry);
   private final YoArtifactCircle inscribedCircle = new YoArtifactCircle("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);
   
   private DoubleYoVariable bodyHeadingAngle = new DoubleYoVariable("bodyHeadingAngle", registry);

   public MiniBeastSupportPolygonVisualizer()
   {
      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3d(), robot);
      
      FramePoint point = new FramePoint(ReferenceFrame.getWorldFrame(), 0.5, 0.0, 0.0);
      FramePoint origin = new FramePoint();
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(!quadrant.equals(RobotQuadrant.HIND_RIGHT))
         {
            YoFramePoint vertex = new YoFramePoint(quadrant.getCamelCaseNameForStartOfExpression(), ReferenceFrame.getWorldFrame(), registry);
            vertex.set(point);
            vertices.put(quadrant, vertex);
            point = point.yawAboutPoint(origin, 2 * Math.PI / 3);
         }
      }
      
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      scs = new SimulationConstructionSet();
      scs.setRobot(robot);
      
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      yoGraphicsListRegistry.registerYoGraphic("centroid", centroidGraphic);
      yoGraphicsListRegistry.registerArtifact("centroid", centroidGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact("inscribedCircle", inscribedCircle);
      yoGraphicsListRegistry.registerArtifact("inscribedCircleCenter", circleCenterGraphic.createArtifact());
      
      yoGraphicsListRegistry.registerArtifact("currentQuadSupportPolygonArtifact", currentQuadSupportPolygonArtifact);

      boolean showOverheadView = true;
      VisualizerUtils.createOverheadPlotter(scs, showOverheadView , yoGraphicsListRegistry);
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
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if(!quadrant.equals(RobotQuadrant.HIND_RIGHT))
         {
            YoFramePoint vertex = vertices.get(quadrant);
            supportPolygon.setFootstep(quadrant, vertex.getFrameTuple());
         }
      }
      drawSupportPolygon(supportPolygon, currentSupportPolygon);
      
      supportPolygon.getCentroid(centroidFramePoint);
      centroid.set(centroidFramePoint);
      
      Point2d centerOfInscribedCircle = new Point2d();
      double radius = supportPolygon.getInCircle(centerOfInscribedCircle);
      circleCenter.setX(centerOfInscribedCircle.getX());
      circleCenter.setY(centerOfInscribedCircle.getY());
      inscribedCircleRadius.set(radius);
      ThreadTools.sleep(10);
   }
   
   private void drawSupportPolygon(MiniBeastSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
   {
      ConvexPolygon2d polygon = new ConvexPolygon2d();
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = supportPolygon.getFootstep(quadrant);
         if(footstep != null)
         {
            polygon.addVertex(footstep.getX(), footstep.getY());
         }
      }
      polygon.update();
      yoFramePolygon.setConvexPolygon2d(polygon);
   }

   
   public static void main(String[] args)
   {
      new MiniBeastSupportPolygonVisualizer();
   }
}
