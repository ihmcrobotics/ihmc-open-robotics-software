package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SmoothCMPBasedICPPlannerTest
{
   private static final String testClassName = "UltimateSmoothCMPBasedICPPlannerTest";
   private static final double epsilon = Epsilons.ONE_TEN_MILLIONTH;
   private final static double spatialEpsilonForDiscontinuity = 0.003; // m //TODO this should depend on the simulation dt
   private final static double spatialEpsilonForPlanningConsistency = 0.010; // m 
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean visualize = false;
   private static final boolean keepSCSUp = false;
   private static final boolean testAssertions = !keepSCSUp && true;

   // Simulation parameters
   private final double dt = 0.001;
   private final double omega = 3.5;

   // Physical parameters
   private final double robotMass = 150;
   private final double gravity = 9.81;
   private final double xToAnkle = 0.05;
   private final double yToAnkle = 0.20;
   private final double zToAnkle = -0.15;
   private final double toeWidth = 0.085;
   private final double heelWidth = 0.11;
   private final double footLength = 0.22;
   private final double footLengthBack = 0.085;
   private final double footLengthForward = footLength - footLengthBack;
   private final double coefficientOfFriction = 0.1;

   // Robot parameters
   private final FramePose initialPose = new FramePose(worldFrame, new Point3D(0.0, 0.0, 0.0), new Quaternion());

   // Planning parameters
   private final double defaultSwingTime = 0.6;
   private final double defaultTransferTime = 0.1;
   private final double defaultInitialSwingTime = defaultSwingTime;
   private final double defaultInitialTransferTime = 0.2;
   private final double defaultFinalSwingTime = defaultSwingTime;
   private final double defaultFinalTransferTime = 0.2;

   private final double stepWidth = 0.25;
   private final double stepLength = 0.5;
   private final int numberOfFootstepsToConsider = 3;
   private final int numberOfFootstepsForTest = 10;
   private final int numberOfPointsToCheckForConsistencyWhenAddingFootsteps = 3;
   private final int numberOfFootstepsToTestForConsistency = Math.min(numberOfFootstepsToConsider, 3);
   private final List<Point2D> contactPointsInFootFrame = Stream.of(new Point2D(footLengthForward, toeWidth / 2.0),
                                                                    new Point2D(footLengthForward, -toeWidth / 2.0),
                                                                    new Point2D(-footLengthBack, -heelWidth / 2.0),
                                                                    new Point2D(-footLengthBack, heelWidth / 2.0))
                                                                .collect(Collectors.toList());

   // Variables for testing and simulation
   private final FramePoint3D comPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   private final FramePoint3D icpPosition = new FramePoint3D();
   private final FrameVector3D icpVelocity = new FrameVector3D();
   private final FrameVector3D icpAcceleration = new FrameVector3D();
   private final FramePoint3D cmpPosition = new FramePoint3D();
   private final FrameVector3D cmpVelocity = new FrameVector3D();
   private final FrameVector3D centroidalAngularMomentum = new FrameVector3D();
   private final FrameVector3D centroidalTorque = new FrameVector3D();
   private final FramePoint3D copPosition = new FramePoint3D();
   private final FrameVector3D copVelocity = new FrameVector3D();
   private final FrameVector3D copAcceleration = new FrameVector3D();

   private YoVariableRegistry registry;
   private YoDouble yoTime;
   private YoBoolean inDoubleSupport;
   private SmoothCMPBasedICPPlanner planner;
   private SmoothCMPPlannerParameters plannerParameters;
   private SideDependentList<FootSpoof> feet;
   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ReferenceFrame> ankleZUpFrames;
   private SideDependentList<ReferenceFrame> soleZUpFrames;
   private ReferenceFrame midFeetZUpFrame;
   private SideDependentList<YoPlaneContactState> contactStates;
   private List<Footstep> footstepList;
   private List<FootstepTiming> timingList;
   private boolean newTestStartDiscontinuity, newTestStartConsistency;
   private List<CoPPointsInFoot> copWaypointsFromPreviousPlan;
   private List<FramePoint3D> icpCornerPointsFromPreviousPlan;
   private List<FramePoint3D> comCornerPointsFromPreviousPlan;
   private List<FramePoint3D> cmpCornerPointsFromPreviousPlan;
   private ArrayList<Updatable> updatables;

   // Variables for visualization
   private YoGraphicsListRegistry graphicsListRegistry;
   private int numberOfTrackBalls = 100;
   private double trackBallSize = 0.002;
   private int numberOfCornerPoints = numberOfFootstepsToConsider * 6 + 1;
   private double cornerPointBallSize = 0.005;
   private double footstepHeight = 0.02;
   private BagOfBalls comTrack, icpTrack, cmpTrack, copTrack;
   private BagOfBalls comInitialCornerPoints, icpInitialCornerPoints, comFinalCornerPoints, icpFinalCornerPoints, copCornerPoints;
   private List<YoFramePose> nextFootstepPoses;
   private SideDependentList<YoFramePose> currentFootLocations;
   private YoGraphicPosition comPositionGraphic, icpPositionGraphic, cmpPositionGraphic, copPositionGraphic;
   private SimulationConstructionSet scs;
   private int SCS_BUFFER_SIZE = 100000;
   private YoAppearanceRGBColor nextFootstepColor = new YoAppearanceRGBColor(Color.BLUE, 0.5);
   private YoAppearanceRGBColor leftFootstepColor = new YoAppearanceRGBColor(new Color(0.85f, 0.35f, 0.65f, 1.0f), 0.5);
   private YoAppearanceRGBColor rightFootstepColor = new YoAppearanceRGBColor(new Color(0.15f, 0.8f, 0.15f, 1.0f), 0.5);
   private Color comPointsColor = Color.BLACK;
   private Color icpPointsColor = Color.RED;
   private Color cmpPointsColor = Color.YELLOW;
   private Color copPointsColor = Color.ORANGE;

   @Before
   public void setupTest()
   {
      this.newTestStartDiscontinuity = true;
      this.registry = new YoVariableRegistry(testClassName);
      if (visualize)
         setupVisualization();
      this.yoTime = new YoDouble("TestTime", registry);
      this.feet = new SideDependentList<>();
      this.ankleZUpFrames = new SideDependentList<>();
      this.soleZUpFrames = new SideDependentList<>();
      this.contactStates = new SideDependentList<>();
      this.updatables = new ArrayList<>();

      for (RobotSide side : RobotSide.values())
      {
         String footName = side.getCamelCaseName();
         FootSpoof foot = new FootSpoof(footName + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInFootFrame, coefficientOfFriction);
         FramePose footPose = new FramePose(initialPose);
         footPose.appendTranslation(0.0, side.negateIfRightSide(stepWidth / 2.0), 0.0);
         foot.setSoleFrame(footPose);

         this.feet.put(side, foot);
         this.ankleZUpFrames.put(side, new ZUpFrame(worldFrame, foot.getFrameAfterParentJoint(), footName + "AnkleZUpFrame"));
         this.soleZUpFrames.put(side, new ZUpFrame(worldFrame, foot.getSoleFrame(), footName + "SoleZUpFrame"));
         YoPlaneContactState contactState = new YoPlaneContactState(footName + "ContactState", foot.getRigidBody(), foot.getSoleFrame(),
                                                                    foot.getContactPoints2d(), foot.getCoefficientOfFriction(), registry);
         contactState.setFullyConstrained();
         this.contactStates.put(side, contactState);
      }
      this.midFeetZUpFrame = new MidFootZUpGroundFrame("MidFeetFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      this.bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, registry, graphicsListRegistry);
      this.bipedSupportPolygons.updateUsingContactStates(contactStates);

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            if (visualize)
            {
               for (YoGraphicsList yoGraphicsList : graphicsListRegistry.getYoGraphicsLists())
               {
                  for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
                  {
                     yoGraphic.update();
                  }
               }
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               ankleZUpFrames.get(robotSide).update();
               soleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });
   }

   private void setupVisualization()
   {
      this.graphicsListRegistry = new YoGraphicsListRegistry();
      setupTrackBallsVisualization();
      setupCornerPointBallsVisualization();
      setupNextFootstepVisualization();
      setupCurrentFootPoseVisualization();
      setupPositionGraphics();
      setupSCS();
   }

   private void setupSCS()
   {
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, SCS_BUFFER_SIZE);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
   }

   private void startSCS()
   {
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();
   }

   private void setupPositionGraphics()
   {
      YoFramePoint yoCoMPosition = new YoFramePoint("CoMPositionForViz", worldFrame, registry);
      comPositionGraphic = new YoGraphicPosition("CoMPositionGraphic", yoCoMPosition, trackBallSize * 2, new YoAppearanceRGBColor(comPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint yoICPPosition = new YoFramePoint("ICPPositionForViz", worldFrame, registry);
      icpPositionGraphic = new YoGraphicPosition("ICPPositionGraphic", yoICPPosition, trackBallSize * 2, new YoAppearanceRGBColor(icpPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint yoCMPPosition = new YoFramePoint("CMPPositionForViz", worldFrame, registry);
      cmpPositionGraphic = new YoGraphicPosition("CMPPositionGraphic", yoCMPPosition, trackBallSize * 2, new YoAppearanceRGBColor(cmpPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint yoCoPPosition = new YoFramePoint("CoPPositionForViz", worldFrame, registry);
      copPositionGraphic = new YoGraphicPosition("CoPPositionGraphic", yoCoPPosition, trackBallSize * 2, new YoAppearanceRGBColor(copPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      graphicsListRegistry.registerYoGraphic("GraphicPositions", comPositionGraphic);
      graphicsListRegistry.registerArtifact("GraphicsArtifacts", comPositionGraphic.createArtifact());
      graphicsListRegistry.registerYoGraphic("GraphicPositions", icpPositionGraphic);
      graphicsListRegistry.registerArtifact("GraphicsArtifacts", icpPositionGraphic.createArtifact());
      graphicsListRegistry.registerYoGraphic("GraphicPositions", cmpPositionGraphic);
      graphicsListRegistry.registerArtifact("GraphicsArtifacts", cmpPositionGraphic.createArtifact());
      graphicsListRegistry.registerYoGraphic("GraphicPositions", copPositionGraphic);
      graphicsListRegistry.registerArtifact("GraphicsArtifacts", copPositionGraphic.createArtifact());
   }

   private void setupCurrentFootPoseVisualization()
   {
      currentFootLocations = new SideDependentList<>();
      for (RobotSide side : RobotSide.values())
      {
         Graphics3DObject footstepGraphic = new Graphics3DObject();
         footstepGraphic.addExtrudedPolygon(contactPointsInFootFrame, footstepHeight, side == RobotSide.LEFT ? leftFootstepColor : rightFootstepColor);
         YoFramePose footPose = new YoFramePose(side.getCamelCaseName() + "FootPose", worldFrame, registry);
         currentFootLocations.put(side, footPose);
         graphicsListRegistry.registerYoGraphic("currentFootPose", new YoGraphicShape(side.getCamelCaseName() + "FootViz", footstepGraphic, footPose, 1.0));
      }
   }

   private void setupNextFootstepVisualization()
   {
      nextFootstepPoses = new ArrayList<>();
      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         Graphics3DObject nextFootstepGraphic = new Graphics3DObject();
         nextFootstepGraphic.addExtrudedPolygon(contactPointsInFootFrame, footstepHeight, nextFootstepColor);
         YoFramePose nextFootstepPose = new YoFramePose("NextFootstep" + i + "Pose", worldFrame, registry);
         nextFootstepPoses.add(nextFootstepPose);
         graphicsListRegistry.registerYoGraphic("UpcomingFootsteps",
                                                new YoGraphicShape("NextFootstep" + i + "Viz", nextFootstepGraphic, nextFootstepPose, 1.0));
      }
   }

   private void setupCornerPointBallsVisualization()
   {
      comInitialCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize * 1.5, "CoMInitialCornerPoint",
                                              new YoAppearanceRGBColor(comPointsColor, 0.5), GraphicType.BALL, registry, graphicsListRegistry);
      comFinalCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "CoMFinalCornerPoint", new YoAppearanceRGBColor(comPointsColor, 0.0),
                                            GraphicType.BALL, registry, graphicsListRegistry);
      icpInitialCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize * 1.5, "ICPInitialCornerPoint",
                                              new YoAppearanceRGBColor(icpPointsColor, 0.5), GraphicType.BALL, registry, graphicsListRegistry);
      icpFinalCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "ICPFinalCornerPoint", new YoAppearanceRGBColor(icpPointsColor, 0.0),
                                            GraphicType.BALL, registry, graphicsListRegistry);
      copCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "CoPCornerPoint", new YoAppearanceRGBColor(copPointsColor, 0.0),
                                       GraphicType.BALL, registry, graphicsListRegistry);
   }

   private void setupTrackBallsVisualization()
   {
      comTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CoMTrack", new YoAppearanceRGBColor(comPointsColor, 0.0), registry, graphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "ICPTrack", new YoAppearanceRGBColor(icpPointsColor, 0.0), registry, graphicsListRegistry);
      cmpTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CMPTrack", new YoAppearanceRGBColor(cmpPointsColor, 0.0), registry, graphicsListRegistry);
      copTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CoPTrack", new YoAppearanceRGBColor(copPointsColor, 0.0), registry, graphicsListRegistry);
   }

   @After
   public void cleanUpTest()
   {
      if (scs != null)
         scs.closeAndDispose();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testForDiscontinuitiesWithoutAngularMomentum()
   {
      boolean isAMOn = false;
      setupPlanner(isAMOn);
      simulate(true, false, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testForDiscontinuitiesWithAngularMomentum()
   {
      boolean isAMOn = true;
      setupPlanner(isAMOn);
      simulate(true, false, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testForPlanningConsistencyWithoutAngularMomentum()
   {
      boolean isAMOn = false;
      setupPlanner(isAMOn);
      simulate(false, true, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testForPlanningConsistencyWithAngularMomentum()
   {
      boolean isAMOn = true;
      setupPlanner(isAMOn);
      simulate(false, true, true);
   }

   private void setupPlanner(boolean isAMOn)
   {
      plannerParameters = new SmoothCMPPlannerParameters()
      {
         @Override
         public boolean planWithAngularMomentum()
         {
            return isAMOn;
         };

         @Override
         public int getNumberOfFootstepsToConsider()
         {
            return numberOfFootstepsToConsider;
         }
      };
      this.planner = new SmoothCMPBasedICPPlanner(robotMass, bipedSupportPolygons, feet, plannerParameters.getNumberOfFootstepsToConsider(),
                                                  registry, graphicsListRegistry, gravity);
      this.planner.initializeParameters(plannerParameters);
      this.planner.setFinalTransferDuration(defaultFinalTransferTime);
      this.planner.setOmega0(omega);
   }

   // Variables for storing values 
   private final FramePoint3D comPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D icpPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D cmpPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D copPositionForDiscontinuity = new FramePoint3D();

   private void testForDiscontinuities()
   {
      if (!newTestStartDiscontinuity)
      {
         assertTrueLocal(comPositionForDiscontinuity.epsilonEquals(comPosition, spatialEpsilonForDiscontinuity));
         assertTrueLocal(icpPositionForDiscontinuity.epsilonEquals(icpPosition, spatialEpsilonForDiscontinuity));
         assertTrueLocal(cmpPositionForDiscontinuity.epsilonEquals(cmpPosition, spatialEpsilonForDiscontinuity * 4));
         assertTrueLocal(copPositionForDiscontinuity.epsilonEquals(copPosition, spatialEpsilonForDiscontinuity * 4));
      }
      else
         newTestStartDiscontinuity = false;
      getPredictedValue(comPositionForDiscontinuity, comPosition, comVelocity, dt);
      getPredictedValue(icpPositionForDiscontinuity, icpPosition, icpVelocity, dt);
      getPredictedValue(cmpPositionForDiscontinuity, cmpPosition, cmpVelocity, dt);
      getPredictedValue(copPositionForDiscontinuity, copPosition, copVelocity, dt);
   }

   private void getPredictedValue(FramePoint3D predictedValueToPack, FramePoint3D currentValue, FrameVector3D rateOfChange, double deltaT)
   {
      predictedValueToPack.setIncludingFrame(rateOfChange);
      predictedValueToPack.scale(deltaT);
      predictedValueToPack.changeFrame(currentValue.getReferenceFrame());
      predictedValueToPack.add(currentValue);
   }

   private void setupConsistencyChecks()
   {
      newTestStartConsistency = true;
      comCornerPointsFromPreviousPlan = new ArrayList<>();
      icpCornerPointsFromPreviousPlan = new ArrayList<>();
      cmpCornerPointsFromPreviousPlan = new ArrayList<>();
      copWaypointsFromPreviousPlan = new ArrayList<>();
      CoPPointsInFoot copPointsInFoot = new CoPPointsInFoot(testClassName, 0, new ReferenceFrame[] {worldFrame, feet.get(RobotSide.LEFT).getSoleFrame(),
            feet.get(RobotSide.RIGHT).getSoleFrame()}, registry);
      copWaypointsFromPreviousPlan.add(copPointsInFoot);
      for (int j = 0; j < 2; j++)
      {
         FramePoint3D newPoint = new FramePoint3D();
         comCornerPointsFromPreviousPlan.add(newPoint);
         newPoint = new FramePoint3D();
         icpCornerPointsFromPreviousPlan.add(newPoint);
         newPoint = new FramePoint3D();
         cmpCornerPointsFromPreviousPlan.add(newPoint);
      }

      for (int i = 0; i < numberOfFootstepsToTestForConsistency; i++)
      {
         copPointsInFoot = new CoPPointsInFoot(testClassName, i
               + 1, new ReferenceFrame[] {worldFrame, feet.get(RobotSide.LEFT).getSoleFrame(), feet.get(RobotSide.RIGHT).getSoleFrame()}, registry);
         copWaypointsFromPreviousPlan.add(copPointsInFoot);

         for (int j = 0; j < plannerParameters.getNumberOfCoPWayPointsPerFoot(); j++)
         {
            FramePoint3D newPoint = new FramePoint3D();
            comCornerPointsFromPreviousPlan.add(newPoint);
            newPoint = new FramePoint3D();
            icpCornerPointsFromPreviousPlan.add(newPoint);
            newPoint = new FramePoint3D();
            cmpCornerPointsFromPreviousPlan.add(newPoint);
         }
      }
   }

   private void testForPlanningConsistency(boolean isDoubleSupport, int stepNumber)
   {
      List<CoPPointsInFoot> copWaypointsFromPlanner = planner.getCoPWaypoints();
      List<FramePoint3D> icpInitialCornerPointsFromPlanner = planner.getInitialDesiredCapturePointPositions();
      List<FramePoint3D> icpFinalCornerPointsFromPlanner = planner.getFinalDesiredCapturePointPositions();
      List<FramePoint3D> comInitialCornerPointsFromPlanner = planner.getInitialDesiredCenterOfMassPositions();
      List<FramePoint3D> comFinalCornerPointsFromPlanner = planner.getFinalDesiredCenterOfMassPositions();

      if (!newTestStartConsistency)
      {
         int numberOfStepsToCheck = (numberOfFootstepsToTestForConsistency < (numberOfFootstepsForTest - stepNumber) ? numberOfFootstepsToTestForConsistency
               : (numberOfFootstepsForTest - stepNumber));

         testCoPConsistency(isDoubleSupport, stepNumber, numberOfStepsToCheck, copWaypointsFromPlanner);
         testICPConsistency(stepNumber, icpInitialCornerPointsFromPlanner, icpFinalCornerPointsFromPlanner, numberOfStepsToCheck);
         testCoMConsistency(stepNumber, comInitialCornerPointsFromPlanner, comFinalCornerPointsFromPlanner, numberOfStepsToCheck);
      }
      else
         newTestStartConsistency = false;

      updateCoPsStoredForConsistencyCheck(isDoubleSupport, copWaypointsFromPlanner);
      updateICPsForConsistencyCheck(isDoubleSupport, icpInitialCornerPointsFromPlanner, icpFinalCornerPointsFromPlanner);
      updateCoMsForConsistencyCheck(isDoubleSupport, comInitialCornerPointsFromPlanner, comFinalCornerPointsFromPlanner);
   }

   private void updateICPsForConsistencyCheck(boolean isDoubleSupport, List<FramePoint3D> icpInitialCornerPointsFromPlanner,
                                              List<FramePoint3D> icpFinalCornerPointsFromPlanner)
   {
      int indexDifference = isDoubleSupport ? plannerParameters.getTransferCoPPointsToPlan().length : (plannerParameters.getSwingCoPPointsToPlan().length + 1);
      icpCornerPointsFromPreviousPlan.get(0).set(icpInitialCornerPointsFromPlanner.get(indexDifference));
      for (int i = 1; i < icpCornerPointsFromPreviousPlan.size(); i++)
      {
         icpCornerPointsFromPreviousPlan.get(i).set(icpFinalCornerPointsFromPlanner.get(i + indexDifference - 1));
      }
   }

   private void updateCoMsForConsistencyCheck(boolean isDoubleSupport, List<FramePoint3D> comInitialCornerPointsFromPlanner,
                                              List<FramePoint3D> comFinalCornerPointsFromPlanner)
   {
      int indexDifference = isDoubleSupport ? plannerParameters.getTransferCoPPointsToPlan().length : (plannerParameters.getSwingCoPPointsToPlan().length + 1);
      comCornerPointsFromPreviousPlan.get(0).set(comInitialCornerPointsFromPlanner.get(indexDifference));
      for (int i = 1; i < comCornerPointsFromPreviousPlan.size(); i++)
      {
         comCornerPointsFromPreviousPlan.get(i).set(comFinalCornerPointsFromPlanner.get(i + indexDifference - 1));
      }
   }

   private void testICPConsistency(int stepNumber, List<FramePoint3D> icpInitialCornerPointsFromPlanner, List<FramePoint3D> icpFinalCornerPointsFromPlanner,
                                   int numberOfStepsToCheck)
   {
      assertTrueLocal("Plan number: " + stepNumber + " " + 0 + " Required: " + icpCornerPointsFromPreviousPlan.get(0).toString() + " Got: "
            + icpInitialCornerPointsFromPlanner.get(0).toString(),
                      icpCornerPointsFromPreviousPlan.get(0).epsilonEquals(icpInitialCornerPointsFromPlanner.get(0), spatialEpsilonForPlanningConsistency));

      int numberOfICPPointsToCheck = (numberOfStepsToCheck >= numberOfFootstepsToConsider) ? numberOfPointsToCheckForConsistencyWhenAddingFootsteps
            : numberOfStepsToCheck * plannerParameters.getNumberOfCoPWayPointsPerFoot() + 3;
      for (int i = 1; i < numberOfICPPointsToCheck - 1; i++)
      {
         assertTrueLocal("Plan number: " + stepNumber + " " + i + " Required: " + icpCornerPointsFromPreviousPlan.get(i).toString() + " Got: "
               + icpInitialCornerPointsFromPlanner.get(i).toString(),
                         icpCornerPointsFromPreviousPlan.get(i).epsilonEquals(icpInitialCornerPointsFromPlanner.get(i), spatialEpsilonForPlanningConsistency));
         assertTrueLocal("Plan number: " + stepNumber + " " + i + " Required: " + icpCornerPointsFromPreviousPlan.get(i).toString() + " Got: "
               + icpFinalCornerPointsFromPlanner.get(i - 1).toString(),
                         icpCornerPointsFromPreviousPlan.get(i).epsilonEquals(icpFinalCornerPointsFromPlanner.get(i - 1),
                                                                              spatialEpsilonForPlanningConsistency));
      }
      assertTrueLocal(icpCornerPointsFromPreviousPlan.get(numberOfICPPointsToCheck - 1)
                                                     .epsilonEquals(icpFinalCornerPointsFromPlanner.get(numberOfICPPointsToCheck - 2),
                                                                    spatialEpsilonForPlanningConsistency));
   }

   private void testCoMConsistency(int stepNumber, List<FramePoint3D> comInitialCornerPointsFromPlanner, List<FramePoint3D> comFinalCornerPointsFromPlanner,
                                   int numberOfStepsToCheck)
   {
      assertTrueLocal("Plan number: " + stepNumber + " " + 0 + " Required: " + comCornerPointsFromPreviousPlan.get(0).toString() + " Got: "
            + comInitialCornerPointsFromPlanner.get(0).toString(),
                      comCornerPointsFromPreviousPlan.get(0).epsilonEquals(comInitialCornerPointsFromPlanner.get(0), spatialEpsilonForPlanningConsistency));

      int numberOfICPPointsToCheck = (numberOfStepsToCheck >= numberOfFootstepsToConsider) ? numberOfPointsToCheckForConsistencyWhenAddingFootsteps
            : numberOfStepsToCheck * plannerParameters.getNumberOfCoPWayPointsPerFoot() + 3;
      for (int i = 1; i < numberOfICPPointsToCheck - 1; i++)
      {
         assertTrueLocal("Plan number: " + stepNumber + " " + i + " Required: " + comCornerPointsFromPreviousPlan.get(i).toString() + " Got: "
               + comInitialCornerPointsFromPlanner.get(i).toString(),
                         comCornerPointsFromPreviousPlan.get(i).epsilonEquals(comInitialCornerPointsFromPlanner.get(i), spatialEpsilonForPlanningConsistency));
         assertTrueLocal("Plan number: " + stepNumber + " " + i + " Required: " + comCornerPointsFromPreviousPlan.get(i).toString() + " Got: "
               + comFinalCornerPointsFromPlanner.get(i - 1).toString(),
                         comCornerPointsFromPreviousPlan.get(i).epsilonEquals(comFinalCornerPointsFromPlanner.get(i - 1),
                                                                              spatialEpsilonForPlanningConsistency));
      }
      assertTrueLocal(comCornerPointsFromPreviousPlan.get(numberOfICPPointsToCheck - 1)
                                                     .epsilonEquals(comFinalCornerPointsFromPlanner.get(numberOfICPPointsToCheck - 2),
                                                                    spatialEpsilonForPlanningConsistency));
   }

   private void updateCoPsStoredForConsistencyCheck(boolean isDoubleSupport, List<CoPPointsInFoot> copWaypointsFromPlanner)
   {
      // Update CoPs and footsteps
      int indexDifference = isDoubleSupport ? 0 : 1;
      copWaypointsFromPreviousPlan.get(0).reset();
      copWaypointsFromPlanner.get(indexDifference).getSwingFootLocation(tempFramePoint1);
      copWaypointsFromPreviousPlan.get(0).setSwingFootLocation(tempFramePoint1);
      copWaypointsFromPlanner.get(indexDifference).getSupportFootLocation(tempFramePoint1);
      copWaypointsFromPreviousPlan.get(0).setSupportFootLocation(tempFramePoint1);

      int finalCoPIndex = copWaypointsFromPlanner.get(indexDifference).getNumberOfCoPPoints() - 1;
      copWaypointsFromPlanner.get(indexDifference).getFinalCoPPosition(tempFramePoint1);
      copWaypointsFromPreviousPlan.get(0).addAndSetIncludingFrame(copWaypointsFromPlanner.get(indexDifference).getCoPPointList().get(finalCoPIndex), 0.0,
                                                                  tempFramePoint1);
      for (int i = 1; i < numberOfFootstepsToTestForConsistency + 1; i++)
         copWaypointsFromPreviousPlan.get(i).setIncludingFrame(copWaypointsFromPlanner.get(i + indexDifference));
   }

   private void testCoPConsistency(boolean isDoubleSupport, int stepNumber, int numberOfStepsToCheck, List<CoPPointsInFoot> copWaypointsFromPlanner)
   {
      // Test CoPs and footsteps
      for (int i = (isDoubleSupport ? 0 : 1); i < numberOfStepsToCheck; i++)
      {
         CoPPointsInFoot requiredCoPs = copWaypointsFromPreviousPlan.get(i);
         CoPPointsInFoot gotCoPs = copWaypointsFromPlanner.get(i);
         assertTrueLocal("Step number: " + (stepNumber + i) + " Required: " + requiredCoPs.getNumberOfCoPPoints() + " Got: " + gotCoPs.getNumberOfCoPPoints(),
                         requiredCoPs.getNumberOfCoPPoints() == gotCoPs.getNumberOfCoPPoints());

         requiredCoPs.getSwingFootLocation(tempFramePoint1);
         gotCoPs.getSwingFootLocation(tempFramePoint2);
         assertTrueLocal("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck + " Required: " + tempFramePoint1.toString() + " Got: "
               + tempFramePoint2.toString(), tempFramePoint1.epsilonEquals(tempFramePoint2, spatialEpsilonForPlanningConsistency));
         requiredCoPs.getSupportFootLocation(tempFramePoint1);
         gotCoPs.getSupportFootLocation(tempFramePoint2);
         assertTrueLocal("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck + " Required: " + tempFramePoint1.toString() + " Got: "
               + tempFramePoint2.toString(), tempFramePoint1.epsilonEquals(tempFramePoint2, spatialEpsilonForPlanningConsistency));

         for (int j = 0; j < requiredCoPs.getNumberOfCoPPoints(); j++)
         {
            assertTrueLocal("Step number: " + (stepNumber + i) + " Required : " + requiredCoPs.getCoPPointList().get(j).toString() + " Got: "
                  + gotCoPs.getCoPPointList().get(j).toString(), requiredCoPs.getCoPPointList().get(j) == gotCoPs.getCoPPointList().get(j));
            assertTrueLocal("Step number: " + (stepNumber + i) + " Required : " + requiredCoPs.get(j).toString() + " Got: " + gotCoPs.get(j).toString(),
                            requiredCoPs.get(j).epsilonEquals(gotCoPs.get(j), spatialEpsilonForPlanningConsistency));
         }
      }

      // Consistency of plans shorter than consideration needs this 
      if (numberOfStepsToCheck < numberOfFootstepsToConsider)
      {
         CoPPointsInFoot requiredCoPs = copWaypointsFromPreviousPlan.get(numberOfStepsToCheck);
         CoPPointsInFoot gotCoPs = copWaypointsFromPlanner.get(numberOfStepsToCheck);

         requiredCoPs.getSwingFootLocation(tempFramePoint1);
         gotCoPs.getSwingFootLocation(tempFramePoint2);
         assertTrueLocal("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck + " Required: " + tempFramePoint1.toString() + " Got: "
               + tempFramePoint2.toString(), tempFramePoint1.epsilonEquals(tempFramePoint2, spatialEpsilonForPlanningConsistency));
         requiredCoPs.getSupportFootLocation(tempFramePoint1);
         gotCoPs.getSupportFootLocation(tempFramePoint2);
         assertTrueLocal("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck + " Required: " + tempFramePoint1.toString() + " Got: "
               + tempFramePoint2.toString(), tempFramePoint1.epsilonEquals(tempFramePoint2, spatialEpsilonForPlanningConsistency));

         for (int j = 0; j < requiredCoPs.getNumberOfCoPPoints(); j++)
         {
            assertTrueLocal("Step number: " + (stepNumber + numberOfStepsToCheck) + " Required : " + requiredCoPs.getCoPPointList().get(j).toString() + " Got: "
                  + gotCoPs.getCoPPointList().get(j).toString(), requiredCoPs.getCoPPointList().get(j) == gotCoPs.getCoPPointList().get(j));
            assertTrueLocal("Step number: " + (stepNumber + numberOfStepsToCheck) + " Required : " + requiredCoPs.get(j).toString() + " Got: "
                  + gotCoPs.get(j).toString(), requiredCoPs.get(j).epsilonEquals(gotCoPs.get(j), spatialEpsilonForPlanningConsistency));
         }
      }
   }

   private void updateVisualizePerTick()
   {
      updateCoMTrack();
      updateICPTrack();
      updateCMPTrack();
      updateCoPTrack();
      updatePositionGraphics();
      scs.tickAndUpdate();
   }

   private void updateUpdatables(double time)
   {
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   private void updateCoMTrack()
   {
      comTrack.setBallLoop(comPosition);
   }

   private void updateICPTrack()
   {
      icpTrack.setBallLoop(icpPosition);
   }

   private void updateCMPTrack()
   {
      cmpTrack.setBallLoop(cmpPosition);
   }

   private void updateCoPTrack()
   {
      copTrack.setBallLoop(copPosition);
   }

   private void updatePositionGraphics()
   {
      comPositionGraphic.setPosition(comPosition);
      icpPositionGraphic.setPosition(icpPosition);
      cmpPositionGraphic.setPosition(cmpPosition);
      copPositionGraphic.setPosition(copPosition);
   }

   private void updateVisualization(int stepIndex)
   {
      updateCurrentFootsteps();
      updateNextFootsteps(stepIndex);
      updateCoMCornerPoints();
      updateICPCornerPoints();
      updateCoPCornerPoints();
   }

   private void updateCurrentFootsteps()
   {
      for (RobotSide side : RobotSide.values)
      {
         YoFramePose footPose = currentFootLocations.get(side);
         if (contactStates.get(side).inContact())
         {
            footPose.setFromReferenceFrame(feet.get(side).getSoleFrame());
         }
         else
         {
            footPose.setToNaN();
         }
      }
   }

   FrameConvexPolygon2d tempConvexPolygon = new FrameConvexPolygon2d();

   private void updateNextFootsteps(int stepIndex)
   {
      int numberOfStepToUpdate = (numberOfFootstepsForTest - stepIndex) < numberOfFootstepsToConsider ? (numberOfFootstepsForTest - stepIndex)
            : numberOfFootstepsToConsider;
      int nextStepIndex;
      for (nextStepIndex = 0; nextStepIndex < numberOfStepToUpdate; nextStepIndex++)
      {
         nextFootstepPoses.get(nextStepIndex).set(footstepList.get(stepIndex + nextStepIndex).getFootstepPose());
      }

      for (; nextStepIndex < numberOfFootstepsToConsider; nextStepIndex++)
      {
         nextFootstepPoses.get(nextStepIndex).setToNaN();
      }
   }

   private void updateCoMCornerPoints()
   {
      List<FramePoint3D> comInitialDesiredPositions = planner.getInitialDesiredCenterOfMassPositions();
      List<FramePoint3D> comFinalDesiredPositions = planner.getFinalDesiredCenterOfMassPositions();
      comInitialCornerPoints.reset();
      for (int i = 0; i < comInitialDesiredPositions.size(); i++)
      {
         comInitialCornerPoints.setBall(comInitialDesiredPositions.get(i));
      }
      comFinalCornerPoints.reset();
      for (int i = 0; i < comFinalDesiredPositions.size(); i++)
      {
         comFinalCornerPoints.setBall(comFinalDesiredPositions.get(i));
      }
   }

   private void updateICPCornerPoints()
   {
      List<FramePoint3D> icpInitialDesiredPositions = planner.getInitialDesiredCapturePointPositions();
      List<FramePoint3D> icpFinalDesiredPositions = planner.getFinalDesiredCapturePointPositions();
      icpInitialCornerPoints.reset();
      for (int i = 0; i < icpInitialDesiredPositions.size(); i++)
      {
         icpInitialCornerPoints.setBall(icpInitialDesiredPositions.get(i));
      }
      icpFinalCornerPoints.reset();
      for (int i = 0; i < icpFinalDesiredPositions.size(); i++)
      {
         icpFinalCornerPoints.setBall(icpFinalDesiredPositions.get(i));
      }
   }

   FramePoint3D tempFramePoint1 = new FramePoint3D();
   FramePoint3D tempFramePoint2 = new FramePoint3D();

   private void updateCoPCornerPoints()
   {
      List<CoPPointsInFoot> copCornerPointPositions = planner.getCoPWaypoints();
      copCornerPoints.reset();
      for (int i = 0; i < copCornerPointPositions.size(); i++)
      {
         CoPPointsInFoot copPoints = copCornerPointPositions.get(i);
         for (int j = 0; j < copPoints.getCoPPointList().size(); j++)
         {
            copPoints.getWaypointInWorldFrameReadOnly(j).getFrameTuple(tempFramePoint1);
            copCornerPoints.setBall(tempFramePoint1);
         }
      }
   }

   @SuppressWarnings("unused")
   private void simulate(boolean checkForDiscontinuities, boolean checkForPlanningConsistency, boolean checkIfDyanmicsAreSatisfied)
   {
      if (visualize)
         startSCS();
      planFootsteps();

      if (checkForPlanningConsistency)
         setupConsistencyChecks();

      inDoubleSupport = new YoBoolean("inDoubleSupport", registry);
      inDoubleSupport.set(true);

      for (RobotSide side : RobotSide.values)
         contactStates.get(side).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      for (int currentStepCount = 0; currentStepCount < numberOfFootstepsForTest;)
      {
         addFootsteps(currentStepCount, footstepList, timingList);
         updateContactState(currentStepCount);
         if (inDoubleSupport.getBooleanValue())
         {
            planner.setTransferToSide(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner.initializeForTransfer(yoTime.getDoubleValue());
         }
         else
         {
            planner.setSupportLeg(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner.initializeForSingleSupport(yoTime.getDoubleValue());
         }

         if (visualize)
            updateVisualization(currentStepCount);
         if (checkForPlanningConsistency)
            testForPlanningConsistency(inDoubleSupport.getBooleanValue(), currentStepCount);
         simulateTicks(checkForDiscontinuities, checkIfDyanmicsAreSatisfied, (inDoubleSupport.getBooleanValue()
               ? timingList.get(currentStepCount).getTransferTime() : timingList.get(currentStepCount).getSwingTime()));
         currentStepCount = updateStateMachine(currentStepCount);
      }

      addFootsteps(numberOfFootstepsForTest, footstepList, timingList);
      updateContactState(-1);
      planner.setTransferToSide(footstepList.get(numberOfFootstepsForTest - 1).getRobotSide());
      planner.initializeForStanding(yoTime.getDoubleValue());

      if (visualize)
         updateVisualization(numberOfFootstepsForTest);
      if (checkForPlanningConsistency)
         testForPlanningConsistency(true, numberOfFootstepsForTest);
      simulateTicks(checkForDiscontinuities, checkIfDyanmicsAreSatisfied, defaultFinalTransferTime);

      if (visualize && keepSCSUp)
         ThreadTools.sleepForever();
   }

   private int updateStateMachine(int currentStepCount)
   {
      if (inDoubleSupport.getBooleanValue())
      {
         inDoubleSupport.set(false);
      }
      else
      {
         inDoubleSupport.set(true);
         currentStepCount++;
      }
      return currentStepCount;
   }

   private void simulateTicks(boolean checkForDiscontinuities, boolean checkIfDyanmicsAreSatisfied, double totalTime)
   {
      for (double timeInState = 0.0; timeInState < totalTime; timeInState += dt)
      {
         simulateOneTick(checkForDiscontinuities, checkIfDyanmicsAreSatisfied);
      }
   }

   private void updateContactState(int currentStepCount)
   {
      if (inDoubleSupport.getBooleanValue())
      {
         for (RobotSide side : RobotSide.values)
            contactStates.get(side).setFullyConstrained();
      }
      else
      {
         RobotSide swingSide = footstepList.get(currentStepCount).getRobotSide();
         contactStates.get(swingSide).clear();
         contactStates.get(swingSide.getOppositeSide()).setFullyConstrained();
         FootSpoof foot = feet.get(swingSide);
         foot.setSoleFrame(footstepList.get(currentStepCount).getFootstepPose());
      }
      bipedSupportPolygons.updateUsingContactStates(contactStates);
   }

   private void simulateOneTick(boolean checkForDiscontinuities, boolean checkIfDyanmicsAreSatisfied)
   {
      getAllVariablesFromPlanner();
      updateUpdatables(yoTime.getDoubleValue());
      if (checkForDiscontinuities)
         testForDiscontinuities();
      if (checkIfDyanmicsAreSatisfied)
         testIfDynamicsAreSatisified();
      if (visualize)
         updateVisualizePerTick();
   }

   private void getAllVariablesFromPlanner()
   {
      yoTime.add(dt);
      planner.compute(yoTime.getDoubleValue());
      planner.getDesiredCenterOfPressurePosition(copPosition);
      planner.getDesiredCenterOfPressureVelocity(copVelocity);
      planner.getDesiredCenterOfPressureVelocity(copAcceleration);
      planner.getDesiredCentroidalAngularMomentum(centroidalAngularMomentum);
      planner.getDesiredCentroidalTorque(centroidalTorque);
      planner.getDesiredCentroidalMomentumPivotPosition(cmpPosition);
      planner.getDesiredCentroidalMomentumPivotVelocity(cmpVelocity);
      planner.getDesiredCapturePointPosition(icpPosition);
      planner.getDesiredCapturePointVelocity(icpVelocity);
      planner.getDesiredCapturePointAcceleration(icpAcceleration);
      planner.getDesiredCenterOfMassPosition(comPosition);
      planner.getDesiredCenterOfMassVelocity(comVelocity);
      planner.getDesiredCenterOfMassAcceleration(comAcceleration);
   }

   private void planFootsteps()
   {
      FootstepTestHelper footstepHelper = new FootstepTestHelper(feet);
      footstepList = footstepHelper.createFootsteps(stepWidth, stepLength, numberOfFootstepsForTest);
      timingList = new ArrayList<>(footstepList.size());
      timingList.add(new FootstepTiming(defaultInitialSwingTime, defaultInitialTransferTime));
      for (int i = 0; i < footstepList.size() - 1; i++)
         timingList.add(new FootstepTiming(defaultSwingTime, defaultTransferTime));
   }

   private void addFootsteps(int currentFootstepIndex, List<Footstep> footstepList, List<FootstepTiming> timingList)
   {
      planner.clearPlan();
      for (int i = currentFootstepIndex; i < Math.min(footstepList.size(), currentFootstepIndex + numberOfFootstepsToConsider); i++)
      {
         planner.addFootstepToPlan(footstepList.get(i), timingList.get(i));
      }
   }

   private void testIfDynamicsAreSatisified()
   {
      assertTrueLocal("CoM dynamics not satisfied, t: " + yoTime.getDoubleValue() + " COM Position: " + comPosition.toString() + " ICP Velocity: "
            + comVelocity.toString() + " ICP Position: " + icpPosition.toString(), checkCoMDynamics(comPosition, comVelocity, icpPosition));
      assertTrueLocal("ICP dynamics not satisfied, t: " + yoTime.getDoubleValue() + " ICP Position: " + icpPosition.toString() + " ICP Velocity: "
            + icpVelocity.toString() + " CMP Position: " + cmpPosition.toString(), checkICPDynamics(icpPosition, icpVelocity, cmpPosition));
   }

   FrameVector3D icpVelocityFromDynamics = new FrameVector3D();

   private boolean checkICPDynamics(FramePoint3D icpPosition, FrameVector3D icpVelocity, FramePoint3D cmpPosition)
   {
      icpVelocityFromDynamics.sub(icpPosition, cmpPosition);
      icpVelocityFromDynamics.scale(omega);
      return icpVelocity.epsilonEquals(icpVelocityFromDynamics, epsilon);
   }

   FrameVector3D comVelocityFromDynamics = new FrameVector3D();

   private boolean checkCoMDynamics(FramePoint3D comPosition, FrameVector3D comVelocity, FramePoint3D icpPosition)
   {
      comVelocityFromDynamics.sub(icpPosition, comPosition);
      comVelocityFromDynamics.scale(omega);
      return comVelocity.epsilonEquals(comVelocityFromDynamics, epsilon);
   }

   private void assertTrueLocal(boolean assertion)
   {
      assertTrueLocal(null, assertion);
   }

   private void assertTrueLocal(String statement, boolean assertion)
   {
      if (testAssertions)
         assertTrue(statement, assertion);
      else if (!assertion)
         PrintTools.error(statement);
   }
}