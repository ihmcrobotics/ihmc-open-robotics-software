package us.ihmc.quadrupedRobotics.footstepChooser;

import java.awt.Color;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.referenceFrames.MockQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicLineSegment;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.tools.thread.ThreadTools;

public class MidFootZUpSwingTargetGeneratorVisualizer implements RobotController
{
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry("footChooserViz");
   private final SimulationConstructionSet scs;
   private final Robot robot;
   private final FloatingJoint rootJoint;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private EnumYoVariable<RobotQuadrant> swingLeg = new EnumYoVariable<RobotQuadrant>("swingLeg", registry, RobotQuadrant.class, true);

   private final YoFrameVector desiredVelocity = new YoFrameVector("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredYawRate = new DoubleYoVariable("desiredYawRate", registry);

   /** Foot Swing **/
   private final ParabolicCartesianTrajectoryGenerator cartesianTrajectoryGenerator;
   private final YoVariableDoubleProvider swingTimeDoubleProvider = new YoVariableDoubleProvider("swingTime", registry);

   private final QuadrantDependentList<YoFramePoint> yoFootPositions = new QuadrantDependentList< YoFramePoint>();
   private final QuadrantDependentList<YoGraphicPosition> footPositionGraphics = new QuadrantDependentList<YoGraphicPosition>();

   private final YoFramePoint swingTarget = new YoFramePoint("swingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition targetViz = new YoGraphicPosition("swingTarget", swingTarget, 0.01, YoAppearance.Red());
   
   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centroidViz = new YoGraphicPosition("centroidViz", centroid, 0.01, YoAppearance.BurlyWood());
   private final YoGraphicLineSegment nominalYawGraphic;
   private final YoFrameLineSegment2d nominalYawLineSegment = new YoFrameLineSegment2d("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);
   
   private final DoubleYoVariable nominalYaw = new DoubleYoVariable("nominalYaw", registry);
   YoFramePoint nominalYawEndpoint = new YoFramePoint("nominalYawEndpoint", ReferenceFrame.getWorldFrame(), registry);

   private final YoGraphicReferenceFrame leftMidZUpFrameViz;
   private final YoGraphicReferenceFrame rightMidZUpFrameViz;

   private final MidFootZUpSwingTargetGenerator footChoser;
   private final QuadrupedReferenceFrames referenceFrames;

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentTriplePolygon = new YoFrameConvexPolygon2d("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
   
   public MidFootZUpSwingTargetGeneratorVisualizer(QuadrupedReferenceFrames referenceFrames)
   {
      desiredVelocity.setX(0.24);

      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3d(), robot);
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      scs = new SimulationConstructionSet();
      scs.setRobot(robot);
      this.referenceFrames = referenceFrames;
      swingLeg.set(RobotQuadrant.FRONT_RIGHT);

      leftMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.LEFT), registry, 0.2);
      rightMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.RIGHT), registry, 0.2);

      
      nominalYawGraphic = new YoGraphicLineSegment("nominalYaw", centroid, nominalYawEndpoint, 0.2, YoAppearance.Yellow(), true, 0.02);
      
      double groundClearance = 0.1;
      boolean showOverheadView = true;
      swingTimeDoubleProvider.set(0.4);
      cartesianTrajectoryGenerator = new ParabolicCartesianTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), swingTimeDoubleProvider,
            groundClearance, registry);

      DefaultSwingTargetGeneratorParameters desfaultFootStepParameters = new DefaultSwingTargetGeneratorParameters();
      footChoser = new MidFootZUpSwingTargetGenerator(desfaultFootStepParameters, referenceFrames, registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         YoFramePoint footPosition = new YoFramePoint(prefix + "footPosition", ReferenceFrame.getWorldFrame(), registry);
         yoFootPositions.put(robotQuadrant, footPosition);

         AppearanceDefinition yoAppearance = this.getYoAppearance(robotQuadrant);
         YoGraphicPosition footPositionGraphic = new YoGraphicPosition(prefix + "footPositionViz", footPosition, 0.02, yoAppearance,
               GraphicType.BALL_WITH_CROSS);
         footPositionGraphics.put(robotQuadrant, footPositionGraphic);

         yoGraphicsListRegistry.registerYoGraphic(prefix + "feet", footPositionGraphic);
         yoGraphicsListRegistry.registerArtifact(prefix + "feetArtifact", footPositionGraphic.createArtifact());
      }

      yoFootPositions.get(RobotQuadrant.FRONT_LEFT).set(new Vector3d(0.12, 0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_LEFT).set(new Vector3d(-0.12, 0.14, 0.0));

      yoFootPositions.get(RobotQuadrant.FRONT_RIGHT).set(new Vector3d(0.12, -0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_RIGHT).set(new Vector3d(-0.12, -0.14, 0.0));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPositionGraphics.get(robotQuadrant).update();
      }

      createGraphicsAndArtifacts(yoGraphicsListRegistry);
      VisualizerUtils.createOverheadPlotter(scs, showOverheadView, yoGraphicsListRegistry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulateDT, recordFrequency);
      scs.startOnAThread();
      scs.simulate();
   }
   
   private void createGraphicsAndArtifacts(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistry.registerYoGraphic("Frames", leftMidZUpFrameViz);
      yoGraphicsListRegistry.registerYoGraphic("Frames", rightMidZUpFrameViz);
      
      yoGraphicsListRegistry.registerYoGraphic("target", targetViz);
      yoGraphicsListRegistry.registerArtifact("target", targetViz.createArtifact());
      
      yoGraphicsListRegistry.registerArtifact("centroidViz", centroidViz.createArtifact());
      yoGraphicsListRegistry.registerYoGraphic("nominalYaw", nominalYawGraphic);

      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("quadSupportPolygonArtifact", supportPolygon, Color.blue, false);
      YoArtifactPolygon currentTriplePolygonArtifact = new YoArtifactPolygon("currentTriplePolygonArtifact", currentTriplePolygon, Color.GREEN, false);
      
      yoGraphicsListRegistry.registerArtifact("nominalYawArtifact", nominalYawArtifact);

      yoGraphicsListRegistry.registerArtifact("supportPolygon", supportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("currentTriplePolygon", currentTriplePolygonArtifact);

      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
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
         quadrupedSupportPolygon.setFootstep(quadrant, yoFootPositions.get(quadrant).getFrameTuple());
      }
//      referenceFrames.update(yoFootPositions);
      if (!cartesianTrajectoryGenerator.isDone())
      {
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
         FramePoint swingLegPosition = new FramePoint();
         cartesianTrajectoryGenerator.computeNextTick(swingLegPosition, simulateDT);

         yoFootPositions.get(robotQuadrant).set(swingLegPosition);
      }
      else
      {
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
         robotQuadrant = robotQuadrant.getNextRegularGaitSwingQuadrant();
         swingLeg.set(robotQuadrant);

         FramePoint initialPosition = new FramePoint(yoFootPositions.get(robotQuadrant).getFramePointCopy());
         FramePoint desiredFootPosition = new FramePoint();
         //         desiredFootPosition.set(initialPosition);
         //         desiredFootPosition.add(0.1,0.0,0.0);
         
         footChoser.getSwingTarget(quadrupedSupportPolygon, robotQuadrant, desiredVelocity.getFrameTuple(), desiredFootPosition, desiredYawRate.getDoubleValue());
         cartesianTrajectoryGenerator.initialize(initialPosition, initialVelocity, intialAcceleration, desiredFootPosition, finalDesiredVelocity);
         swingTarget.set(desiredFootPosition);
      }
      
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint yoFootPosition = yoFootPositions.get(robotQuadrant);
         FramePoint footPosition = yoFootPosition.getFramePointCopy();
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         fourFootSupportPolygon.setFootstep(robotQuadrant, footPosition);
      }
      FramePoint centroidFramePoint = fourFootSupportPolygon.getCentroidFramePoint();
      centroid.set(centroidFramePoint);
      nominalYaw.set(fourFootSupportPolygon.getNominalYaw());
      
      FramePoint endPoint = new FramePoint(centroidFramePoint);
      endPoint.add(0.4,0.0,0.0);
      endPoint.set(endPoint.yawAboutPoint(centroidFramePoint, nominalYaw.getDoubleValue()));
      nominalYawEndpoint.set(endPoint);
      
      FramePoint2d endpointTwoD = new FramePoint2d();
      endPoint.getFramePoint2d(endpointTwoD);
      nominalYawLineSegment.set(centroid.getFramePoint2dCopy(), endpointTwoD);
      nominalYawGraphic.update();
      
      rootJoint.setPosition(centroidFramePoint.getPoint());
      drawSupportPolygon(fourFootSupportPolygon, supportPolygon);
      leftMidZUpFrameViz.update();
      rightMidZUpFrameViz.update();
      ThreadTools.sleep(10);
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
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
}
