package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import java.awt.Color;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser.NextSwingLegChooser;
import us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser.QuadrupedGaitSwingLegChooser;
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
import us.ihmc.robotics.math.trajectories.ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
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
   private final ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator cartesianTrajectoryGenerator;
   private final DoubleYoVariable swingTimeTrajectoryTimeStart = new DoubleYoVariable("swingTimeTrajectoryTimeStart", registry);
   private final DoubleYoVariable swingTimeTrajectoryTimeCurrent = new DoubleYoVariable("swingTimeTrajectoryTimeCurrent", registry);
   private final DoubleYoVariable desiredSwingTime = new DoubleYoVariable("desiredSwingTime", registry);
   private final DoubleYoVariable swingHeight = new DoubleYoVariable("swingHeight", registry);

   private final QuadrantDependentList<YoFramePoint> yoFootPositions = new QuadrantDependentList< YoFramePoint>();
   private final QuadrantDependentList<YoGraphicPosition> footPositionGraphics = new QuadrantDependentList<YoGraphicPosition>();

   private final YoFramePoint swingTarget = new YoFramePoint("swingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector finalDesiredVelocity = new YoFrameVector("finalDesiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition targetViz = new YoGraphicPosition("swingTarget", swingTarget, 0.01, YoAppearance.Red());

   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint temporaryCentroid = new FramePoint();
   private final YoGraphicPosition centroidViz = new YoGraphicPosition("centroidViz", centroid, 0.01, YoAppearance.BurlyWood());
   private final YoGraphicLineSegment nominalYawGraphic;
   private final YoFrameLineSegment2d nominalYawLineSegment = new YoFrameLineSegment2d("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);

   private final DoubleYoVariable nominalYaw = new DoubleYoVariable("nominalYaw", registry);
   YoFramePoint nominalYawEndpoint = new YoFramePoint("nominalYawEndpoint", ReferenceFrame.getWorldFrame(), registry);

   private final YoGraphicReferenceFrame leftMidZUpFrameViz;
   private final YoGraphicReferenceFrame rightMidZUpFrameViz;

   private final NextSwingLegChooser nextStepFootChooser;
   private final MidFootZUpSwingTargetGenerator swingTargetGenerator;
   private final QuadrupedReferenceFrames referenceFrames;

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentTriplePolygon = new YoFrameConvexPolygon2d("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
   private final DoubleYoVariable robotTimestamp;

   public MidFootZUpSwingTargetGeneratorVisualizer(QuadrupedReferenceFrames referenceFrames)
   {
      desiredVelocity.setX(0.24);
      swingHeight.set(0.2);

      robot = new Robot("viz");
      rootJoint = new FloatingJoint("floating", new Vector3D(), robot);
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      scs = new SimulationConstructionSet();
      scs.setRobot(robot);
      robotTimestamp = robot.getYoTime();
      this.referenceFrames = referenceFrames;
      swingLeg.set(RobotQuadrant.FRONT_RIGHT);

      leftMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.LEFT), registry, 0.2);
      rightMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.RIGHT), registry, 0.2);

      nominalYawGraphic = new YoGraphicLineSegment("nominalYaw", centroid, nominalYawEndpoint, 0.2, YoAppearance.Yellow(), true);
      nominalYawGraphic.setLineRadiusWhenOneMeterLong(0.02);

      boolean showOverheadView = true;
      desiredSwingTime.set(0.4);
      cartesianTrajectoryGenerator = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), registry);
      DefaultSwingTargetGeneratorParameters desfaultFootStepParameters = new DefaultSwingTargetGeneratorParameters();
//      nextStepFootChooser = new LongestFeasibleStepChooser(desfaultFootStepParameters, referenceFrames, registry, yoGraphicsListRegistry);
      nextStepFootChooser = new QuadrupedGaitSwingLegChooser(referenceFrames, registry, yoGraphicsListRegistry);
      swingTargetGenerator = new MidFootZUpSwingTargetGenerator(desfaultFootStepParameters, referenceFrames, registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         YoFramePoint footPosition = new YoFramePoint(prefix + "footPosition", ReferenceFrame.getWorldFrame(), registry);
         yoFootPositions.set(robotQuadrant, footPosition);

         AppearanceDefinition yoAppearance = this.getYoAppearance(robotQuadrant);
         YoGraphicPosition footPositionGraphic = new YoGraphicPosition(prefix + "footPositionViz", footPosition, 0.02, yoAppearance,
               GraphicType.BALL_WITH_CROSS);
         footPositionGraphics.set(robotQuadrant, footPositionGraphic);

         yoGraphicsListRegistry.registerYoGraphic(prefix + "feet", footPositionGraphic);
         yoGraphicsListRegistry.registerArtifact(prefix + "feetArtifact", footPositionGraphic.createArtifact());
      }

      yoFootPositions.get(RobotQuadrant.FRONT_LEFT).set(new Vector3D(0.12, 0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_LEFT).set(new Vector3D(-0.12, 0.14, 0.0));

      yoFootPositions.get(RobotQuadrant.FRONT_RIGHT).set(new Vector3D(0.12, -0.14, 0.0));
      yoFootPositions.get(RobotQuadrant.HIND_RIGHT).set(new Vector3D(-0.12, -0.14, 0.0));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPositionGraphics.get(robotQuadrant).update();
      }

      createGraphicsAndArtifacts(yoGraphicsListRegistry);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(showOverheadView);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
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

   FrameVector intialAcceleration = new FrameVector();

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
         swingTimeTrajectoryTimeCurrent.set(robotTimestamp.getDoubleValue() - swingTimeTrajectoryTimeStart.getDoubleValue());
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
         FramePoint swingLegPosition = new FramePoint();
         cartesianTrajectoryGenerator.compute(simulateDT);
         cartesianTrajectoryGenerator.getPosition(swingLegPosition);

         yoFootPositions.get(robotQuadrant).set(swingLegPosition);
      }
      else
      {
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
//         robotQuadrant = robotQuadrant.getNextRegularGaitSwingQuadrant();
         robotQuadrant = nextStepFootChooser.chooseNextSwingLeg(quadrupedSupportPolygon, robotQuadrant, desiredVelocity.getFrameTuple(), desiredYawRate.getDoubleValue());
         swingLeg.set(robotQuadrant);

         FramePoint initialPosition = new FramePoint(yoFootPositions.get(robotQuadrant).getFramePointCopy());
         FramePoint desiredFootPosition = new FramePoint();

         swingTimeTrajectoryTimeStart.set(robotTimestamp.getDoubleValue());
         swingTargetGenerator.getSwingTarget(quadrupedSupportPolygon, robotQuadrant, desiredVelocity.getFrameTuple(), desiredFootPosition, desiredYawRate.getDoubleValue());
         cartesianTrajectoryGenerator.setTrajectoryParameters(desiredSwingTime.getDoubleValue(), initialPosition, swingHeight.getDoubleValue(), desiredFootPosition, finalDesiredVelocity.getFrameTuple());
         cartesianTrajectoryGenerator.initialize();
         swingTarget.set(desiredFootPosition);
      }

      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint yoFootPosition = yoFootPositions.get(robotQuadrant);
         FramePoint footPosition = yoFootPosition.getFramePointCopy();
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         fourFootSupportPolygon.setFootstep(robotQuadrant, footPosition);
      }

      fourFootSupportPolygon.getCentroid(temporaryCentroid);
      centroid.set(temporaryCentroid);
      nominalYaw.set(fourFootSupportPolygon.getNominalYaw());

      FramePoint endPoint = new FramePoint(temporaryCentroid);
      endPoint.add(0.4,0.0,0.0);
      endPoint.yawAboutPoint(temporaryCentroid, endPoint, nominalYaw.getDoubleValue());
      nominalYawEndpoint.set(endPoint);

      FramePoint2d endpointTwoD = new FramePoint2d();
      endPoint.getFramePoint2d(endpointTwoD);
      nominalYawLineSegment.set(centroid.getFramePoint2dCopy(), endpointTwoD);
      nominalYawGraphic.update();

      rootJoint.setPosition(temporaryCentroid.getPoint());
      drawSupportPolygon(fourFootSupportPolygon, supportPolygon);
      leftMidZUpFrameViz.update();
      rightMidZUpFrameViz.update();
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
}
