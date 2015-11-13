package us.ihmc.quadrupedRobotics.footstepChooser;

import java.awt.Color;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedControllerParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.swingLegChooser.LongestFeasibleStepChooser;
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
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.gui.tools.VisualizerUtils;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicLineSegment;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
 
public abstract class QuadrupedMidFootZUpSwingTargetGeneratorTest implements RobotController
{
   private static final boolean DEBUG = false;

   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;
   
   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;

   private final YoVariableRegistry registry = new YoVariableRegistry("MidFootZUpSwingTargetGeneratorTest");
   private SimulationConstructionSet scs;
   private Robot robot;
   private FloatingJoint rootJoint;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private EnumYoVariable<RobotQuadrant> swingLeg = new EnumYoVariable<RobotQuadrant>("swingLeg", registry, RobotQuadrant.class, true);

   private final YoFrameVector desiredVelocity = new YoFrameVector("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredYawRate = new DoubleYoVariable("desiredYawRate", registry);

   /** Foot Swing **/
   private ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator cartesianTrajectoryGenerator;
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
   private final YoGraphicPosition centroidViz = new YoGraphicPosition("centroidViz", centroid, 0.01, YoAppearance.BurlyWood());
   private YoGraphicLineSegment nominalYawGraphic;
   private final YoFrameLineSegment2d nominalYawLineSegment = new YoFrameLineSegment2d("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);
   
   private final DoubleYoVariable nominalYaw = new DoubleYoVariable("nominalYaw", registry);
   YoFramePoint nominalYawEndpoint = new YoFramePoint("nominalYawEndpoint", ReferenceFrame.getWorldFrame(), registry);

   private YoGraphicReferenceFrame leftMidZUpFrameViz;
   private YoGraphicReferenceFrame rightMidZUpFrameViz;

   private LongestFeasibleStepChooser nextStepFootChooser;
   private MidFootZUpSwingTargetGenerator swingTargetGenerator;

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentTriplePolygon = new YoFrameConvexPolygon2d("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final QuadrupedSupportPolygon quadrupedSupportPolygon = new QuadrupedSupportPolygon();
   private DoubleYoVariable robotTimestamp;
   
   private double simulateDuration; 
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      blockingSimulationRunner.destroySimulation();
      
      GlobalTimer.clearTimers();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   protected boolean setupAndTestForwardWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedControllerParameters quadrupedControllerParameters)
   {
      boolean success = true;
      
      setupTestParameters(1.0, 0.0, 0.0, RobotQuadrant.FRONT_RIGHT, 10.0); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();
      
      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);
      
      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);
      try
      {
         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e1)
      {
         success = false;
         e1.printStackTrace();
      }
      
      Vector3d startPosition = new Vector3d();
      rootJoint.getPosition(startPosition);
      
      try
      {
         blockingSimulationRunner.simulateAndBlock(simulateDuration);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         success = false;
         e.printStackTrace();
      }
            
      Vector3d endPosition = new Vector3d();
      rootJoint.getPosition(endPosition);

      Vector3d robotTranslation = new Vector3d();
      robotTranslation.sub(endPosition, startPosition);
      
      Vector3d expectedTranslation = new Vector3d(quadrupedControllerParameters.getStanceLength() / 4.0 * simulateDuration, 0.0, robotTranslation.getZ());

      success &= robotTranslation.epsilonEquals(expectedTranslation, 1e-2);

      if(DEBUG)
      {
         System.out.println("Expected translation:");
         System.out.println(expectedTranslation);
         System.out.println("Robot translation:");
         System.out.println(robotTranslation);
      }
      
      return success;
   }

   protected boolean setupAndTestBackwardWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedControllerParameters quadrupedControllerParameters)
   {
      boolean success = true;
      
      setupTestParameters(-1.0, 0.0, 0.0, RobotQuadrant.FRONT_RIGHT, 10.0); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();
      
      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);
      
      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);
      try
      {
         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e1)
      {
         success = false;
         e1.printStackTrace();
      }
      
      Vector3d startPosition = new Vector3d();
      rootJoint.getPosition(startPosition);
      
      try
      {
         blockingSimulationRunner.simulateAndBlock(simulateDuration);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         success = false;
         e.printStackTrace();
      }
            
      Vector3d endPosition = new Vector3d();
      rootJoint.getPosition(endPosition);

      Vector3d robotTranslation = new Vector3d();
      robotTranslation.sub(endPosition, startPosition);
      
      Vector3d expectedTranslation = new Vector3d(-quadrupedControllerParameters.getStanceLength() / 4.0 * simulateDuration, 0.0, robotTranslation.getZ());
      
      success &= robotTranslation.epsilonEquals(expectedTranslation, 1e-2);

      if(DEBUG)
      {
         System.out.println("Expected translation:");
         System.out.println(expectedTranslation);
         System.out.println("Robot translation:");
         System.out.println(robotTranslation);
      }
      
      return success;
   }
   
   protected boolean setupAndTestSidewaysLeftWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedControllerParameters quadrupedControllerParameters)
   {
      boolean success = true;
      
      setupTestParameters(0.0, 1.0, 0.0, RobotQuadrant.FRONT_RIGHT, 5.0); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();
      
      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);
      
      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);
      try
      {
         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e1)
      {
         success = false;
         e1.printStackTrace();
      }
      
      Vector3d startPosition = new Vector3d();
      rootJoint.getPosition(startPosition);
      
      try
      {
         blockingSimulationRunner.simulateAndBlock(simulateDuration);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         success = false;
         e.printStackTrace();
      }
      
      Vector3d endPosition = new Vector3d();
      rootJoint.getPosition(endPosition);
      
      Vector3d robotTranslation = new Vector3d();
      robotTranslation.sub(endPosition, startPosition);
      
      Vector3d expectedTranslation = new Vector3d(0.0, quadrupedControllerParameters.getStanceWidth() / 4.0 * simulateDuration, robotTranslation.getZ());
      
      success &= robotTranslation.epsilonEquals(expectedTranslation, 2e-2);
      
      if(DEBUG)
      {
         System.out.println("Expected translation:");
         System.out.println(expectedTranslation);
         System.out.println("Robot translation:");
         System.out.println(robotTranslation);
      }
      
      return success;
   }
   
   protected boolean setupAndTestSidewaysRightWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedControllerParameters quadrupedControllerParameters)
   {
      boolean success = true;
      
      setupTestParameters(0.0, -1.0, 0.0, RobotQuadrant.FRONT_RIGHT, 5.0); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();
      
      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);
      
      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);
      try
      {
         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e1)
      {
         success = false;
         e1.printStackTrace();
      }
      
      Vector3d startPosition = new Vector3d();
      rootJoint.getPosition(startPosition);
      
      try
      {
         blockingSimulationRunner.simulateAndBlock(simulateDuration);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         success = false;
         e.printStackTrace();
      }
      
      Vector3d endPosition = new Vector3d();
      rootJoint.getPosition(endPosition);
      
      Vector3d robotTranslation = new Vector3d();
      robotTranslation.sub(endPosition, startPosition);
      
      Vector3d expectedTranslation = new Vector3d(0.0, -quadrupedControllerParameters.getStanceWidth() / 4.0 * simulateDuration, robotTranslation.getZ());
      
      success &= robotTranslation.epsilonEquals(expectedTranslation, 2e-2);
      
      if(DEBUG)
      {
         System.out.println("Expected translation:");
         System.out.println(expectedTranslation);
         System.out.println("Robot translation:");
         System.out.println(robotTranslation);
      }
      
      return success;
   }
   
   private void setupTestParameters(double desiredVelocityX, double desiredVelocityY, double desiredYawRate, RobotQuadrant startingLeg, double simulateDuration)
   {
      desiredVelocity.setX(desiredVelocityX);
      desiredVelocity.setY(desiredVelocityY);
      this.desiredYawRate.set(desiredYawRate);
      swingLeg.set(startingLeg);
      this.simulateDuration = simulateDuration; 
      
      swingHeight.set(0.2);
      desiredSwingTime.set(0.4);
   }

   private void setupScs()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      
      scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(simulateDT, recordFrequency);

      scs.startOnAThread();
   }

   private void setupRobot(CommonQuadrupedReferenceFrames referenceFrames, QuadrupedControllerParameters quadrupedControllerParameters)
   {
      robot = new Robot("testRobot");
      rootJoint = new FloatingJoint("floating", new Vector3d(), robot);
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);
      
      robotTimestamp = robot.getYoTime();

      cartesianTrajectoryGenerator = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), registry);
      nextStepFootChooser = new LongestFeasibleStepChooser(quadrupedControllerParameters, referenceFrames, registry, yoGraphicsListRegistry);
      swingTargetGenerator = new MidFootZUpSwingTargetGenerator(quadrupedControllerParameters, referenceFrames, registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         YoFramePoint footPosition = new YoFramePoint(prefix + "footPosition", ReferenceFrame.getWorldFrame(), registry);

         FramePoint initialFootPosition = new FramePoint(referenceFrames.getFootFrame(robotQuadrant));
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
         initialFootPosition.setZ(0.0);

         footPosition.set(initialFootPosition);
         
         yoFootPositions.put(robotQuadrant, footPosition);
      }
   }
   
   private void createGraphicsAndArtifacts(QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      
      leftMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.LEFT), registry, 0.2);
      rightMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.RIGHT), registry, 0.2);
      nominalYawGraphic = new YoGraphicLineSegment("nominalYaw", centroid, nominalYawEndpoint, 0.2, YoAppearance.Yellow(), true, 0.02);
      
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

      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         AppearanceDefinition yoAppearance = this.getYoAppearance(robotQuadrant);
         YoFramePoint footPosition = yoFootPositions.get(robotQuadrant);
         YoGraphicPosition footPositionGraphic = new YoGraphicPosition(prefix + "footPositionViz", footPosition, 0.02, yoAppearance,
               GraphicType.BALL_WITH_CROSS);
         footPositionGraphics.put(robotQuadrant, footPositionGraphic);

         yoGraphicsListRegistry.registerYoGraphic(prefix + "feet", footPositionGraphic);
         yoGraphicsListRegistry.registerArtifact(prefix + "feetArtifact", footPositionGraphic.createArtifact());
      }      
      
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
      
      VisualizerUtils.createOverheadPlotter(scs, true, yoGraphicsListRegistry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
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
      return "test";
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
      
      if (!cartesianTrajectoryGenerator.isDone())
      {
         swingTimeTrajectoryTimeCurrent.set(robotTimestamp.getDoubleValue() - swingTimeTrajectoryTimeStart.getDoubleValue());
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
         FramePoint swingLegPosition = new FramePoint();
         cartesianTrajectoryGenerator.compute(simulateDT);
         cartesianTrajectoryGenerator.get(swingLegPosition);

         yoFootPositions.get(robotQuadrant).set(swingLegPosition);
      }
      else
      {
         RobotQuadrant robotQuadrant = swingLeg.getEnumValue();
         robotQuadrant = robotQuadrant.getNextRegularGaitSwingQuadrant();
         nextStepFootChooser.chooseNextSwingLeg(quadrupedSupportPolygon, robotQuadrant, desiredVelocity.getFrameTuple(), desiredYawRate.getDoubleValue());
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
      
      rootJoint.setPosition(centroidFramePoint.getPoint());
      drawSupportPolygon(fourFootSupportPolygon, supportPolygon);

      if(simulationTestingParameters.getCreateGUI())
         updateGraphics();
      
      ThreadTools.sleep(10);
   }

   private void updateGraphics()
   {
      nominalYawGraphic.update();
      leftMidZUpFrameViz.update();
      rightMidZUpFrameViz.update();
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
