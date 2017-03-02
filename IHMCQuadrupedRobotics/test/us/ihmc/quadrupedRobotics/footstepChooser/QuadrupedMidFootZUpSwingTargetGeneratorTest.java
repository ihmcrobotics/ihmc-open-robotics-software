package us.ihmc.quadrupedRobotics.footstepChooser;

import java.awt.Color;

import org.junit.After;
import org.junit.Before;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.CommonQuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.MidFootZUpSwingTargetGenerator;
import us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser.LongestFeasibleStepChooser;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.ControllerFailureException;
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
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class QuadrupedMidFootZUpSwingTargetGeneratorTest implements RobotController
{
   private static final boolean DEBUG = false;

   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   {
//      simulationTestingParameters.setKeepSCSUp(true);
   }
   private BlockingSimulationRunner blockingSimulationRunner;

   private static final double simulateDT = 0.01;
   private static final int recordFrequency = 1;

   private static final double EPSILON = 1e-3;

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
   private final FramePoint temporaryCentroid = new FramePoint();
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

   private QuadrantDependentList<Double> maxStepLengthsForward = new QuadrantDependentList<Double>();
   private QuadrantDependentList<Double> maxStepLengthsSideways = new QuadrantDependentList<Double>();
   private double stepDuration = 0.25;
   private int numberOfSteps = 20; //5 steps per legs
   private double simulateDuration = stepDuration * numberOfSteps;
   private double minimumDistanceFromSameSideFoot;

   private final QuadrantDependentList<YoFramePoint> yoFootPositionsBeforeStep = new QuadrantDependentList<YoFramePoint>();

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   protected boolean setupAndTestForwardWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      boolean success = true;

      setupTestParameters(1.0, 0.0, 0.0, RobotQuadrant.FRONT_RIGHT); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();

      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);

      //Simulate one tick so that the robot can be initialized
      blockingSimulationRunner.simulateNTicksAndBlock(1);

      int numberOfTicks = (int)(stepDuration / simulateDT);

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         blockingSimulationRunner.simulateNTicksAndBlock(numberOfTicks);

         double footDisplacementX = yoFootPositions.get(swingLeg.getEnumValue()).getX() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getX();
         double footDisplacementY = yoFootPositions.get(swingLeg.getEnumValue()).getY() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getY();

         if(stepIndex < 2) //the first 2 steps are only half steps because the feet are in a square configuration
         {
            success &= MathTools.isInsideBoundsInclusive(footDisplacementX, maxStepLengthsForward.get(swingLeg.getEnumValue()) / 2.0 - minimumDistanceFromSameSideFoot - EPSILON, maxStepLengthsForward.get(swingLeg.getEnumValue()) / 2.0 + EPSILON);
            success &= MathTools.epsilonEquals(footDisplacementY, 0.0, EPSILON);
         }
         else
         {
            success &= MathTools.isInsideBoundsInclusive(footDisplacementX, maxStepLengthsForward.get(swingLeg.getEnumValue()) - minimumDistanceFromSameSideFoot - EPSILON, maxStepLengthsForward.get(swingLeg.getEnumValue()) + EPSILON);
            success &= MathTools.epsilonEquals(footDisplacementY, 0.0, EPSILON);
         }

         if(DEBUG)
         {
            System.out.println("////////////////////////////////////");
            System.out.println("success = " + success);
            System.out.println(swingLeg.getEnumValue());
            System.out.println(" footDisplacementX");
            System.out.println(footDisplacementX);
            System.out.println(" footDisplacementY");
            System.out.println(footDisplacementY);
         }

         yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).set(yoFootPositions.get(swingLeg.getEnumValue()).getFramePointCopy());

         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }

      return success;
   }

   protected boolean setupAndTestBackwardWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      boolean success = true;

      setupTestParameters(-1.0, 0.0, 0.0, RobotQuadrant.FRONT_RIGHT); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();

      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);

      //Simulate one tick so that the robot can be initialized
      blockingSimulationRunner.simulateNTicksAndBlock(1);

      int numberOfTicks = (int)(stepDuration / simulateDT);

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         blockingSimulationRunner.simulateNTicksAndBlock(numberOfTicks);

         double footDisplacementX = yoFootPositions.get(swingLeg.getEnumValue()).getX() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getX();
         double footDisplacementY = yoFootPositions.get(swingLeg.getEnumValue()).getY() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getY();

         if(stepIndex < 2) //the first 2 steps are only half steps because the feet are in a square configuration
         {
            success &= MathTools.isInsideBoundsInclusive(footDisplacementX,  - maxStepLengthsForward.get(swingLeg.getEnumValue()) / 2.0 - EPSILON, - maxStepLengthsForward.get(swingLeg.getEnumValue()) / 2.0 + minimumDistanceFromSameSideFoot + EPSILON);
            success &= MathTools.epsilonEquals(footDisplacementY, 0.0, EPSILON);
         }
         else
         {
            success &= MathTools.isInsideBoundsInclusive(footDisplacementX,  - maxStepLengthsForward.get(swingLeg.getEnumValue()) - EPSILON, - maxStepLengthsForward.get(swingLeg.getEnumValue()) + minimumDistanceFromSameSideFoot + EPSILON);
            success &= MathTools.epsilonEquals(footDisplacementY, 0.0, EPSILON);
         }

         if(DEBUG)
         {
            System.out.println("////////////////////////////////////");
            System.out.println("success = " + success);
            System.out.println(swingLeg.getEnumValue());
            System.out.println(" footDisplacementX");
            System.out.println(footDisplacementX);
            System.out.println(" footDisplacementY");
            System.out.println(footDisplacementY);
         }

         yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).set(yoFootPositions.get(swingLeg.getEnumValue()).getFramePointCopy());

         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }

      return success;
   }

   protected boolean setupAndTestSidewaysLeftWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      boolean success = true;

      setupTestParameters(0.0, 1.0, 0.0, RobotQuadrant.FRONT_RIGHT); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();

      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);

      //Simulate one tick so that the robot can be initialized
      blockingSimulationRunner.simulateNTicksAndBlock(1);

      int numberOfTicks = (int)(stepDuration / simulateDT);

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         blockingSimulationRunner.simulateNTicksAndBlock(numberOfTicks);

         double footDisplacementX = yoFootPositions.get(swingLeg.getEnumValue()).getX() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getX();
         double footDisplacementY = yoFootPositions.get(swingLeg.getEnumValue()).getY() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getY();

         if(stepIndex < 2) //the first 2 steps are only half steps because the feet are in a square configuration
         {
            success &= MathTools.epsilonEquals(footDisplacementX, 0.0, EPSILON);
            success &= MathTools.isInsideBoundsInclusive(footDisplacementY, maxStepLengthsSideways.get(swingLeg.getEnumValue()) / 2.0 - minimumDistanceFromSameSideFoot - EPSILON, maxStepLengthsSideways.get(swingLeg.getEnumValue()) / 2.0 + EPSILON);
         }
         else
         {
            success &= MathTools.epsilonEquals(footDisplacementX, 0.0, EPSILON);
            success &= MathTools.isInsideBoundsInclusive(footDisplacementY, maxStepLengthsSideways.get(swingLeg.getEnumValue()) - minimumDistanceFromSameSideFoot - EPSILON, maxStepLengthsSideways.get(swingLeg.getEnumValue()) + EPSILON);
         }

         if(DEBUG)
         {
            System.out.println("////////////////////////////////////");
            System.out.println("success = " + success);
            System.out.println(swingLeg.getEnumValue());
            System.out.println(" footDisplacementX");
            System.out.println(footDisplacementX);
            System.out.println(" footDisplacementY");
            System.out.println(footDisplacementY);
         }

         yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).set(yoFootPositions.get(swingLeg.getEnumValue()).getFramePointCopy());

         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }

      return success;
   }

   protected boolean setupAndTestSidewaysRightWalking(QuadrupedReferenceFrames referenceFrames, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      boolean success = true;

      setupTestParameters(0.0, -1.0, 0.0, RobotQuadrant.FRONT_RIGHT); //ask for a very high speed to be sure to ask for the longest step
      setupRobot(referenceFrames, quadrupedControllerParameters);
      setupScs();

      if (simulationTestingParameters.getCreateGUI())
         createGraphicsAndArtifacts(referenceFrames, yoGraphicsListRegistry);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, simulateDuration + 0.5 * simulateDuration);

      //Simulate one tick so that the robot can be initialized
      blockingSimulationRunner.simulateNTicksAndBlock(1);

      int numberOfTicks = (int)(stepDuration / simulateDT);

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         blockingSimulationRunner.simulateNTicksAndBlock(numberOfTicks);

         double footDisplacementX = yoFootPositions.get(swingLeg.getEnumValue()).getX() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getX();
         double footDisplacementY = yoFootPositions.get(swingLeg.getEnumValue()).getY() - yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).getY();

         if(stepIndex < 2) //the first 2 steps are only half steps because the feet are in a square configuration
         {
            success &= MathTools.epsilonEquals(footDisplacementX, 0.0, EPSILON);
            success &= MathTools.isInsideBoundsInclusive(footDisplacementY,  - maxStepLengthsSideways.get(swingLeg.getEnumValue()) / 2.0 - EPSILON, - maxStepLengthsSideways.get(swingLeg.getEnumValue()) / 2.0 + minimumDistanceFromSameSideFoot + EPSILON);
         }
         else
         {
            success &= MathTools.epsilonEquals(footDisplacementX, 0.0, EPSILON);
            success &= MathTools.isInsideBoundsInclusive(footDisplacementY,  - maxStepLengthsSideways.get(swingLeg.getEnumValue()) - EPSILON, - maxStepLengthsSideways.get(swingLeg.getEnumValue()) + minimumDistanceFromSameSideFoot + EPSILON);
         }

         if(DEBUG)
         {
            System.out.println("////////////////////////////////////");
            System.out.println("success = " + success);
            System.out.println(swingLeg.getEnumValue());
            System.out.println(" footDisplacementX");
            System.out.println(footDisplacementX);
            System.out.println(" footDisplacementY");
            System.out.println(footDisplacementY);
         }

         yoFootPositionsBeforeStep.get(swingLeg.getEnumValue()).set(yoFootPositions.get(swingLeg.getEnumValue()).getFramePointCopy());

         blockingSimulationRunner.simulateNTicksAndBlock(1);
      }

      return success;
   }

   private void setupTestParameters(double desiredVelocityX, double desiredVelocityY, double desiredYawRate, RobotQuadrant startingLeg)
   {
      desiredVelocity.setX(desiredVelocityX);
      desiredVelocity.setY(desiredVelocityY);
      this.desiredYawRate.set(desiredYawRate);
      swingLeg.set(startingLeg);

      swingHeight.set(0.15);
      desiredSwingTime.set(stepDuration);
   }

   private void setupScs()
   {
      scs = new SimulationConstructionSet(robot, simulationTestingParameters);
      scs.setDT(simulateDT, recordFrequency);

      scs.startOnAThread();
   }

   private void setupRobot(CommonQuadrupedReferenceFrames referenceFrames, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters)
   {
      robot = new Robot("testRobot");
      rootJoint = new FloatingJoint("floating", new Vector3D(), robot);
      robot.getRobotsYoVariableRegistry();
      robot.setController(this);

      robotTimestamp = robot.getYoTime();

      cartesianTrajectoryGenerator = new ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator("swingLegTraj", ReferenceFrame.getWorldFrame(), registry);
      nextStepFootChooser = new LongestFeasibleStepChooser(quadrupedControllerParameters, referenceFrames, registry, yoGraphicsListRegistry);
      swingTargetGenerator = new MidFootZUpSwingTargetGenerator(quadrupedControllerParameters, referenceFrames, registry);

      minimumDistanceFromSameSideFoot = quadrupedControllerParameters.getMinimumDistanceFromSameSideFoot();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         YoFramePoint footPosition = new YoFramePoint(prefix + "footPosition", ReferenceFrame.getWorldFrame(), registry);
         YoFramePoint footPositionBeforeSwing = new YoFramePoint(prefix + "footPositionBeforeSwing", ReferenceFrame.getWorldFrame(), registry);

         FramePoint initialFootPosition = new FramePoint(ReferenceFrame.getWorldFrame());
         if(robotQuadrant.isQuadrantInFront())
            initialFootPosition.setX(quadrupedControllerParameters.getStanceLength());

         if(robotQuadrant.isQuadrantOnRightSide())
            initialFootPosition.setY(- quadrupedControllerParameters.getStanceWidth() / 2.0);
         else
            initialFootPosition.setY(quadrupedControllerParameters.getStanceWidth() / 2.0);

         ReferenceFrame hipPitchFrame = referenceFrames.getHipPitchFrame(robotQuadrant);
         RigidBodyTransform currenthipPitchFrameTransform = hipPitchFrame.getTransformToRoot();
         Vector3D hipPitchFrameTranslation = new Vector3D();
         currenthipPitchFrameTransform.getTranslation(hipPitchFrameTranslation );

         double robotHeight = 0.7 * referenceFrames.getLegLength(robotQuadrant);
         RigidBodyTransform preCorruptionTransform = new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(0.0, 0.0, robotHeight - hipPitchFrameTranslation.getZ()));
         hipPitchFrame.corruptTransformToParentPreMultiply(preCorruptionTransform);

         double maxStepLengthForward = Math.sqrt(Math.pow(referenceFrames.getLegLength(robotQuadrant), 2) - Math.pow(robotHeight, 2));
         double amountToSkew = Math.min(quadrupedControllerParameters.getMaxForwardSkew(), quadrupedControllerParameters.getStanceLength() / 2.0);
         maxStepLengthForward = 2.0 * Math.min(maxStepLengthForward, amountToSkew);

         double maxStepLengthSideways = Math.sqrt(Math.pow(referenceFrames.getLegLength(robotQuadrant), 2) - Math.pow(robotHeight, 2));
         amountToSkew = Math.min(quadrupedControllerParameters.getMaxLateralSkew(), quadrupedControllerParameters.getStanceWidth() / 2.0);
         maxStepLengthSideways= 2.0 * Math.min(maxStepLengthSideways, amountToSkew);

         maxStepLengthsForward.set(robotQuadrant, maxStepLengthForward);
         maxStepLengthsSideways.set(robotQuadrant, maxStepLengthSideways);

         if(DEBUG)
         {
            System.out.println("maxStepLenghtForward = " + maxStepLengthForward);
            System.out.println("maxStepLenghtSideways = " + maxStepLengthSideways);
         }

         initialFootPosition.setZ(0.0);

         footPosition.set(initialFootPosition);
         footPositionBeforeSwing.set(initialFootPosition);

         yoFootPositions.set(robotQuadrant, footPosition);
         yoFootPositionsBeforeStep.set(robotQuadrant, footPositionBeforeSwing);
      }
   }

   private void createGraphicsAndArtifacts(QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      leftMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.LEFT), registry, 0.2);
      rightMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.RIGHT), registry, 0.2);
      nominalYawGraphic = new YoGraphicLineSegment("nominalYaw", centroid, nominalYawEndpoint, 0.2, YoAppearance.Yellow(), true);
      nominalYawGraphic.setLineRadiusWhenOneMeterLong(0.02);


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
         footPositionGraphics.set(robotQuadrant, footPositionGraphic);

         yoGraphicsListRegistry.registerYoGraphic(prefix + "feet", footPositionGraphic);
         yoGraphicsListRegistry.registerArtifact(prefix + "feetArtifact", footPositionGraphic.createArtifact());
      }

      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
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
      return "QuadrupedMidFootZUpSwingTargetGeneratorTest";
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
         cartesianTrajectoryGenerator.getPosition(swingLegPosition);

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

      rootJoint.setPosition(temporaryCentroid.getPoint());
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
