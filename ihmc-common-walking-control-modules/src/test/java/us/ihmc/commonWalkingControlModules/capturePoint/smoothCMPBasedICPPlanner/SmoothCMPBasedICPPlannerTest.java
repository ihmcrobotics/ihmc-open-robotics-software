package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.CMPTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
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
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class SmoothCMPBasedICPPlannerTest
{
   private static final String testClassName = "UltimateSmoothCMPBasedICPPlannerTest";
   private static final double epsilon = Epsilons.ONE_TEN_MILLIONTH;
   private final static double spatialEpsilonForDiscontinuity = 0.003; // m //TODO this should depend on the simulation dt
   private final static double spatialEpsilonForPlanningConsistency = 0.010; // m
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean visualize = false;
   private static final boolean keepSCSUp = false;

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
   private final FramePose3D initialPose = new FramePose3D(worldFrame, new Point3D(0.0, 0.0, 0.0), new Quaternion());

   // Planning parameters
   private final double defaultSwingTime = 0.6;
   private final double defaultTransferTime = 0.1;
   private final double defaultInitialSwingTime = defaultSwingTime;
   private final double defaultInitialTransferTime = 0.2;
   private final double defaultFinalSwingTime = defaultSwingTime;
   private final double defaultFinalTransferTime = 0.2;
   private final double defaultStandingTime = 0.2;

   private static final double stepWidth = 0.25;
   private final double stepLength = 0.5;
   private final int numberOfFootstepsToConsider = 3;
   private final int numberOfPointsToCheckForConsistencyWhenAddingFootsteps = 3;
   private final int numberOfFootstepsToTestForConsistency = Math.min(numberOfFootstepsToConsider, 3);
   private final List<Point2D> contactPointsInFootFrame = Stream
         .of(new Point2D(footLengthForward, toeWidth / 2.0), new Point2D(footLengthForward, -toeWidth / 2.0), new Point2D(-footLengthBack, -heelWidth / 2.0),
             new Point2D(-footLengthBack, heelWidth / 2.0)).collect(Collectors.toList());

   // Struct for testing and simulation
   private class ICPPlannerData
   {
      final FramePoint3D comPosition = new FramePoint3D();
      final FrameVector3D comVelocity = new FrameVector3D();
      final FrameVector3D comAcceleration = new FrameVector3D();
      final FramePoint3D icpPosition = new FramePoint3D();
      final FrameVector3D icpVelocity = new FrameVector3D();
      final FrameVector3D icpAcceleration = new FrameVector3D();
      final FramePoint3D cmpPosition = new FramePoint3D();
      final FrameVector3D cmpVelocity = new FrameVector3D();
      final FrameVector3D centroidalAngularMomentum = new FrameVector3D();
      final FrameVector3D centroidalTorque = new FrameVector3D();
      final FramePoint3D copPosition = new FramePoint3D();
      final FrameVector3D copVelocity = new FrameVector3D();
      final FrameVector3D copAcceleration = new FrameVector3D();
   }

   private final ICPPlannerData icpPlannerData1 = new ICPPlannerData();
   private final ICPPlannerData icpPlannerData2 = new ICPPlannerData();
   private final FramePose3D swingFootPose = new FramePose3D();

   private YoVariableRegistry registry;
   private YoDouble yoTime;
   private YoBoolean inDoubleSupport;
   private SmoothCMPBasedICPPlanner planner;
   private SmoothCMPPlannerParameters plannerParameters;
   private SideDependentList<FootSpoof> feet;
   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ReferenceFrame> ankleZUpFrames;
   private SideDependentList<ReferenceFrame> soleZUpFrames;
   private SideDependentList<ReferenceFrame> soleFrames;
   private ReferenceFrame midFeetZUpFrame;
   private SideDependentList<YoPlaneContactState> contactStates;
   private List<Footstep> footstepList;
   private List<FootstepTiming> timingList;
   private List<FootstepShiftFractions> shiftFractions;
   private boolean newTestStartDiscontinuity, newTestStartConsistency;
   private List<CoPPointsInFoot> copWaypointsFromPreviousPlan;
   private RecyclingArrayList<FramePoint3D> icpCornerPointsFromPreviousPlan;
   private RecyclingArrayList<FramePoint3D> comCornerPointsFromPreviousPlan;
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
   private List<YoFramePoseUsingYawPitchRoll> nextFootstepPoses;
   private SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootLocations;
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

   private int numberOfFootstepsForTest;

   @BeforeEach
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
      this.soleFrames = new SideDependentList<>();
      this.contactStates = new SideDependentList<>();
      this.updatables = new ArrayList<>();

      for (RobotSide side : RobotSide.values())
      {
         String footName = side.getCamelCaseName();
         FootSpoof foot = new FootSpoof(footName + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInFootFrame, coefficientOfFriction);
         FramePose3D footPose = new FramePose3D(initialPose);
         footPose.appendTranslation(0.0, side.negateIfRightSide(stepWidth / 2.0), 0.0);
         foot.setSoleFrame(footPose);

         this.feet.put(side, foot);
         this.ankleZUpFrames.put(side, new ZUpFrame(worldFrame, foot.getFrameAfterParentJoint(), footName + "AnkleZUpFrame"));
         this.soleZUpFrames.put(side, new ZUpFrame(worldFrame, foot.getSoleFrame(), footName + "SoleZUpFrame"));
         soleFrames.put(side, foot.getSoleFrame());
         YoPlaneContactState contactState = new YoPlaneContactState(footName + "ContactState", foot.getRigidBody(), foot.getSoleFrame(),
                                                                    foot.getContactPoints2d(), foot.getCoefficientOfFriction(), registry);
         contactState.setFullyConstrained();
         this.contactStates.put(side, contactState);
      }
      this.midFeetZUpFrame = new MidFootZUpGroundFrame("MidFeetFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      this.bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, graphicsListRegistry);
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
      SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(visualize);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, simulationTestingParameters);
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
      YoFramePoint3D yoCoMPosition = new YoFramePoint3D("CoMPositionForViz", worldFrame, registry);
      comPositionGraphic = new YoGraphicPosition("CoMPositionGraphic", yoCoMPosition, trackBallSize * 2, new YoAppearanceRGBColor(comPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint3D yoICPPosition = new YoFramePoint3D("ICPPositionForViz", worldFrame, registry);
      icpPositionGraphic = new YoGraphicPosition("ICPPositionGraphic", yoICPPosition, trackBallSize * 2, new YoAppearanceRGBColor(icpPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint3D yoCMPPosition = new YoFramePoint3D("CMPPositionForViz", worldFrame, registry);
      cmpPositionGraphic = new YoGraphicPosition("CMPPositionGraphic", yoCMPPosition, trackBallSize * 2, new YoAppearanceRGBColor(cmpPointsColor, 0.0),
                                                 GraphicType.BALL_WITH_ROTATED_CROSS);
      YoFramePoint3D yoCoPPosition = new YoFramePoint3D("CoPPositionForViz", worldFrame, registry);
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
         YoFramePoseUsingYawPitchRoll footPose = new YoFramePoseUsingYawPitchRoll(side.getCamelCaseName() + "FootPose", worldFrame, registry);
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
         YoFramePoseUsingYawPitchRoll nextFootstepPose = new YoFramePoseUsingYawPitchRoll("NextFootstep" + i + "Pose", worldFrame, registry);
         nextFootstepPoses.add(nextFootstepPose);
         graphicsListRegistry
               .registerYoGraphic("UpcomingFootsteps", new YoGraphicShape("NextFootstep" + i + "Viz", nextFootstepGraphic, nextFootstepPose, 1.0));
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

   @AfterEach
   public void cleanUpTest()
   {
      if (keepSCSUp)
      {
         ThreadTools.sleepForever();
      }

      if (scs != null && !keepSCSUp)
         scs.closeAndDispose();
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testForDiscontinuitiesWithoutAngularMomentum()
   {
      numberOfFootstepsForTest = 10;
      boolean isAMOn = false;
      this.planner = createPlanner(isAMOn, false, registry);
      simulate(true, false, true);
   }

   @Test
   public void testForDiscontinuitiesWithAngularMomentum()
   {
      numberOfFootstepsForTest = 10;
      boolean isAMOn = true;
      this.planner = createPlanner(isAMOn, false, registry);
      simulate(true, false, true);
   }

   @Test
   public void testForPlanningConsistencyWithoutAngularMomentum()
   {
      numberOfFootstepsForTest = 10;
      boolean isAMOn = false;
      this.planner = createPlanner(isAMOn, false, registry);
      simulate(false, true, true);
   }

   @Test
   public void testForPlanningConsistencyWithAngularMomentum()
   {
      numberOfFootstepsForTest = 10;

      boolean isAMOn = true;
      this.planner = createPlanner(isAMOn, false, registry);
      simulate(false, true, true);
   }

   @Test
   public void testForPlanningConsistencyWithAndWithoutContinuousReplanning()
   {
      numberOfFootstepsForTest = 10;

      boolean isAMOn = false;
      SmoothCMPBasedICPPlanner planner1 = createPlanner(isAMOn, false, new YoVariableRegistry("TestRegistry1"));
      SmoothCMPBasedICPPlanner planner2 = createPlanner(isAMOn, true, new YoVariableRegistry("TestRegistry2"));
      simulateAndAssertSamePlan(planner1, planner2);
   }

   private SmoothCMPBasedICPPlanner createPlanner(boolean isAMOn, boolean doContinuousReplanning, YoVariableRegistry parentRegistry)
   {
      plannerParameters = new SmoothCMPPlannerParameters()
      {
         @Override
         public boolean planSwingAngularMomentum()
         {
            return isAMOn;
         }

         @Override
         public boolean planTransferAngularMomentum()
         {
            return isAMOn;
         }

         @Override
         public int getNumberOfFootstepsToConsider()
         {
            return numberOfFootstepsToConsider;
         }

         @Override
         public boolean doContinuousReplanningForTransfer()
         {
            return doContinuousReplanning;
         }

         @Override
         public boolean doContinuousReplanningForSwing()
         {
            return doContinuousReplanning;
         }
      };

      SmoothCMPBasedICPPlanner planner = new SmoothCMPBasedICPPlanner(robotMass, bipedSupportPolygons, soleZUpFrames, feet, null, null, parentRegistry,
                                                                      graphicsListRegistry, gravity, plannerParameters);
      planner.setFinalTransferDuration(defaultFinalTransferTime);
      planner.setOmega0(omega);
      planner.ensureContinuityEnteringEachTransfer(true);
      return planner;
   }

   // Variables for storing values
   private final FramePoint3D comPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D icpPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D cmpPositionForDiscontinuity = new FramePoint3D();
   private final FramePoint3D copPositionForDiscontinuity = new FramePoint3D();

   private int tick = 0;

   private void testForDiscontinuities(ICPPlannerData plannerData)
   {
      if (!newTestStartDiscontinuity)
      {
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("CoM position doesn't pass continuity test on tick " + tick + ".", comPositionForDiscontinuity,
                                                              plannerData.comPosition, spatialEpsilonForDiscontinuity);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("ICP position doesn't pass continuity test on tick " + tick + ".", icpPositionForDiscontinuity,
                                                              plannerData.icpPosition, spatialEpsilonForDiscontinuity);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("CMP position doesn't pass continuity test on tick " + tick + ".", cmpPositionForDiscontinuity,
                                                              plannerData.cmpPosition, spatialEpsilonForDiscontinuity * 4);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("CoP position doesn't pass continuity test on tick " + tick + ".", copPositionForDiscontinuity,
                                                              plannerData.copPosition, spatialEpsilonForDiscontinuity * 4);
      }
      else
      {
         newTestStartDiscontinuity = false;
      }

      comPositionForDiscontinuity.scaleAdd(dt, plannerData.comVelocity, plannerData.comPosition);
      icpPositionForDiscontinuity.scaleAdd(dt, plannerData.icpVelocity, plannerData.icpPosition);
      cmpPositionForDiscontinuity.scaleAdd(dt, plannerData.cmpVelocity, plannerData.cmpPosition);
      copPositionForDiscontinuity.scaleAdd(dt, plannerData.copVelocity, plannerData.copPosition);
      tick++;
   }

   private void setupConsistencyChecks()
   {
      newTestStartConsistency = true;
      copWaypointsFromPreviousPlan = new ArrayList<>();
      CoPPointsInFoot copPointsInFoot = new CoPPointsInFoot(testClassName, 0, registry);
      copWaypointsFromPreviousPlan.add(copPointsInFoot);

      icpCornerPointsFromPreviousPlan = new RecyclingArrayList<>(FramePoint3D::new);
      comCornerPointsFromPreviousPlan = new RecyclingArrayList<>(FramePoint3D::new);

      for (int i = 0; i < numberOfFootstepsToTestForConsistency; i++)
      {
         copPointsInFoot = new CoPPointsInFoot(testClassName, i + 1, registry);
         copWaypointsFromPreviousPlan.add(copPointsInFoot);
      }
   }

   private void testForPlanningConsistency(boolean isDoubleSupport, int stepNumber)
   {
      List<CoPPointsInFoot> copWaypointsFromPlanner = planner.getReferenceCoPGenerator().getWaypoints();
      List<? extends FramePoint3DReadOnly> icpInitialCornerPointsFromPlanner = planner.getReferenceICPGenerator().getICPPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> icpFinalCornerPointsFromPlanner = planner.getReferenceICPGenerator().getICPPositionDesiredFinalList();
      List<? extends FramePoint3DReadOnly> comInitialCornerPointsFromPlanner = planner.getReferenceCoMGenerator().getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> comFinalCornerPointsFromPlanner = planner.getReferenceCoMGenerator().getCoMPositionDesiredFinalList();

      if (!newTestStartConsistency)
      {

         int numberOfStepsToCheck = (numberOfFootstepsToTestForConsistency < (numberOfFootstepsForTest - stepNumber) ?
               numberOfFootstepsToTestForConsistency :
               (numberOfFootstepsForTest - stepNumber));

         testCoPConsistency(isDoubleSupport, stepNumber, numberOfStepsToCheck, copWaypointsFromPlanner);
         testICPConsistency(stepNumber, icpInitialCornerPointsFromPlanner, icpFinalCornerPointsFromPlanner, numberOfStepsToCheck);
         testCoMConsistency(stepNumber, comInitialCornerPointsFromPlanner, comFinalCornerPointsFromPlanner, numberOfStepsToCheck);
      }
      else
      {
         newTestStartConsistency = false;
      }

      updateCoPsStoredForConsistencyCheck(isDoubleSupport, copWaypointsFromPlanner);
      updateICPsForConsistencyCheck(isDoubleSupport, icpInitialCornerPointsFromPlanner, icpFinalCornerPointsFromPlanner);
      updateCoMsForConsistencyCheck(isDoubleSupport, comInitialCornerPointsFromPlanner, comFinalCornerPointsFromPlanner);
   }

   private void updateICPsForConsistencyCheck(boolean isDoubleSupport, List<? extends FramePoint3DReadOnly> icpInitialCornerPointsFromPlanner,
                                              List<? extends FramePoint3DReadOnly> icpFinalCornerPointsFromPlanner)
   {
      icpCornerPointsFromPreviousPlan.clear();
      int indexDifference = isDoubleSupport ? plannerParameters.getTransferCoPPointsToPlan().length : (plannerParameters.getSwingCoPPointsToPlan().length + 1);
      icpCornerPointsFromPreviousPlan.add().set(icpInitialCornerPointsFromPlanner.get(Math.min(indexDifference, icpInitialCornerPointsFromPlanner.size() - 1)));
      for (int i = 1; i < icpFinalCornerPointsFromPlanner.size() - (indexDifference - 1); i++)
      {
         icpCornerPointsFromPreviousPlan.add().set(icpFinalCornerPointsFromPlanner.get(i + indexDifference - 1));
      }
   }

   private void updateCoMsForConsistencyCheck(boolean isDoubleSupport, List<? extends FramePoint3DReadOnly> comInitialCornerPointsFromPlanner,
                                              List<? extends FramePoint3DReadOnly> comFinalCornerPointsFromPlanner)
   {
      comCornerPointsFromPreviousPlan.clear();
      int indexDifference = isDoubleSupport ? plannerParameters.getTransferCoPPointsToPlan().length : (plannerParameters.getSwingCoPPointsToPlan().length + 1);
      comCornerPointsFromPreviousPlan.add().set(comInitialCornerPointsFromPlanner.get(indexDifference));
      for (int i = 1; i < comInitialCornerPointsFromPlanner.size() - (indexDifference - 1); i++)
      {
         comCornerPointsFromPreviousPlan.add().set(comFinalCornerPointsFromPlanner.get(i + indexDifference - 1));
      }
   }

   private void testICPConsistency(int stepNumber, List<? extends FramePoint3DReadOnly> icpInitialCornerPointsFromPlanner,
                                   List<? extends FramePoint3DReadOnly> icpFinalCornerPointsFromPlanner, int numberOfStepsToCheck)
   {
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + 0, icpCornerPointsFromPreviousPlan.get(0),
                                                           icpInitialCornerPointsFromPlanner.get(0), spatialEpsilonForPlanningConsistency);

      int numberOfICPPointsToCheck = (numberOfStepsToCheck >= numberOfFootstepsToConsider) ?
            numberOfPointsToCheckForConsistencyWhenAddingFootsteps :
            numberOfStepsToCheck * plannerParameters.getNumberOfCoPWayPointsPerFoot() + 3;
      for (int i = 1; i < numberOfICPPointsToCheck - 1; i++)
      {
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + i, icpCornerPointsFromPreviousPlan.get(i),
                                                              icpInitialCornerPointsFromPlanner.get(i), spatialEpsilonForPlanningConsistency);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + i, icpCornerPointsFromPreviousPlan.get(i),
                                                              icpInitialCornerPointsFromPlanner.get(i), spatialEpsilonForPlanningConsistency);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + i, icpCornerPointsFromPreviousPlan.get(i),
                                                              icpFinalCornerPointsFromPlanner.get(i - 1), spatialEpsilonForPlanningConsistency);
      }
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(icpCornerPointsFromPreviousPlan.get(numberOfICPPointsToCheck - 1),
                                                           icpFinalCornerPointsFromPlanner.get(numberOfICPPointsToCheck - 2),
                                                           spatialEpsilonForPlanningConsistency);
   }

   private void testCoMConsistency(int stepNumber, List<? extends FramePoint3DReadOnly> comInitialCornerPointsFromPlanner,
                                   List<? extends FramePoint3DReadOnly> comFinalCornerPointsFromPlanner, int numberOfStepsToCheck)
   {
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + 0, comCornerPointsFromPreviousPlan.get(0),
                                                           comInitialCornerPointsFromPlanner.get(0), spatialEpsilonForPlanningConsistency);

      int numberOfICPPointsToCheck = (numberOfStepsToCheck >= numberOfFootstepsToConsider) ?
            numberOfPointsToCheckForConsistencyWhenAddingFootsteps :
            numberOfStepsToCheck * plannerParameters.getNumberOfCoPWayPointsPerFoot() + 3;
      for (int i = 1; i < numberOfICPPointsToCheck - 1; i++)
      {
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + i, comCornerPointsFromPreviousPlan.get(i),
                                                              comInitialCornerPointsFromPlanner.get(i), spatialEpsilonForPlanningConsistency);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " " + i, comCornerPointsFromPreviousPlan.get(i),
                                                              comFinalCornerPointsFromPlanner.get(i - 1), spatialEpsilonForPlanningConsistency);
      }
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comCornerPointsFromPreviousPlan.get(numberOfICPPointsToCheck - 1),
                                                           comFinalCornerPointsFromPlanner.get(numberOfICPPointsToCheck - 2),
                                                           spatialEpsilonForPlanningConsistency);
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
      copWaypointsFromPreviousPlan.get(0).addWaypoint(copWaypointsFromPlanner.get(indexDifference).getCoPPointList().get(finalCoPIndex), 0.0, tempFramePoint1);
      int numberToCheck = Math.min(numberOfFootstepsToTestForConsistency + 1, copWaypointsFromPlanner.size() - indexDifference);
      for (int i = 1; i < numberToCheck; i++)
         copWaypointsFromPreviousPlan.get(i).set(copWaypointsFromPlanner.get(i + indexDifference));
   }

   private void testCoPConsistency(boolean isDoubleSupport, int stepNumber, int numberOfStepsToCheck, List<CoPPointsInFoot> copWaypointsFromPlanner)
   {
      // Test CoPs and footsteps
      for (int i = (isDoubleSupport ? 0 : 1); i < numberOfStepsToCheck; i++)
      {
         CoPPointsInFoot previousPlannedCoPWaypoints = copWaypointsFromPreviousPlan.get(i);
         CoPPointsInFoot actualPlannedCoPWaypoints = copWaypointsFromPlanner.get(i);
         assertEquals(
               "Step number: " + (stepNumber + i) + " Required: " + previousPlannedCoPWaypoints.getNumberOfCoPPoints() + " Got: " + actualPlannedCoPWaypoints
                     .getNumberOfCoPPoints(), previousPlannedCoPWaypoints.getNumberOfCoPPoints(), actualPlannedCoPWaypoints.getNumberOfCoPPoints());

         previousPlannedCoPWaypoints.getSwingFootLocation(tempFramePoint1);
         actualPlannedCoPWaypoints.getSwingFootLocation(tempFramePoint2);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck, tempFramePoint1, tempFramePoint2,
                                                 spatialEpsilonForPlanningConsistency);
         previousPlannedCoPWaypoints.getSupportFootLocation(tempFramePoint1);
         actualPlannedCoPWaypoints.getSupportFootLocation(tempFramePoint2);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck, tempFramePoint1, tempFramePoint2,
                                                 spatialEpsilonForPlanningConsistency);

         for (int j = 0; j < previousPlannedCoPWaypoints.getNumberOfCoPPoints(); j++)
         {
            assertEquals("Step number: " + (stepNumber + i) + ", waypoint number: " + j, previousPlannedCoPWaypoints.getCoPPointList().get(j),
                         actualPlannedCoPWaypoints.getCoPPointList().get(j));
            assertTrajectoryPointEquals("Step number: " + (stepNumber + i) + ", waypoint number: " + j, previousPlannedCoPWaypoints.get(j),
                                        actualPlannedCoPWaypoints.get(j), spatialEpsilonForPlanningConsistency);
         }
      }

      // Consistency of plans shorter than consideration needs this
      if (numberOfStepsToCheck < numberOfFootstepsToConsider)
      {
         CoPPointsInFoot requiredCoPs = copWaypointsFromPreviousPlan.get(numberOfStepsToCheck);
         CoPPointsInFoot gotCoPs = copWaypointsFromPlanner.get(numberOfStepsToCheck);

         requiredCoPs.getSwingFootLocation(tempFramePoint1);
         gotCoPs.getSwingFootLocation(tempFramePoint2);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck, tempFramePoint1, tempFramePoint2,
                                                 spatialEpsilonForPlanningConsistency);
         requiredCoPs.getSupportFootLocation(tempFramePoint1);
         gotCoPs.getSupportFootLocation(tempFramePoint2);
         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("Plan number: " + stepNumber + " StepNumber: " + numberOfStepsToCheck, tempFramePoint1, tempFramePoint2,
                                                 spatialEpsilonForPlanningConsistency);

         for (int j = 0; j < requiredCoPs.getNumberOfCoPPoints(); j++)
         {
            assertEquals("Step number: " + (stepNumber + numberOfStepsToCheck), requiredCoPs.getCoPPointList().get(j), gotCoPs.getCoPPointList().get(j));
            assertTrajectoryPointEquals("Step number: " + (stepNumber + numberOfStepsToCheck), requiredCoPs.get(j), gotCoPs.get(j),
                                        spatialEpsilonForPlanningConsistency);
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
      comTrack.setBallLoop(icpPlannerData1.comPosition);
   }

   private void updateICPTrack()
   {
      icpTrack.setBallLoop(icpPlannerData1.icpPosition);
   }

   private void updateCMPTrack()
   {
      cmpTrack.setBallLoop(icpPlannerData1.cmpPosition);
   }

   private void updateCoPTrack()
   {
      copTrack.setBallLoop(icpPlannerData1.copPosition);
   }

   private void updatePositionGraphics()
   {
      comPositionGraphic.setPosition(icpPlannerData1.comPosition);
      icpPositionGraphic.setPosition(icpPlannerData1.icpPosition);
      cmpPositionGraphic.setPosition(icpPlannerData1.cmpPosition);
      copPositionGraphic.setPosition(icpPlannerData1.copPosition);
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
         YoFramePoseUsingYawPitchRoll footPose = currentFootLocations.get(side);
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

   private void updateNextFootsteps(int stepIndex)
   {
      int numberOfStepToUpdate =
            (numberOfFootstepsForTest - stepIndex) < numberOfFootstepsToConsider ? (numberOfFootstepsForTest - stepIndex) : numberOfFootstepsToConsider;
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
      List<? extends FramePoint3DReadOnly> comInitialDesiredPositions = planner.getReferenceCoMGenerator().getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> comFinalDesiredPositions = planner.getReferenceCoMGenerator().getCoMPositionDesiredFinalList();
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
      List<? extends FramePoint3DReadOnly> icpInitialDesiredPositions = planner.getReferenceICPGenerator().getICPPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> icpFinalDesiredPositions = planner.getReferenceICPGenerator().getICPPositionDesiredFinalList();
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
      List<CoPPointsInFoot> copCornerPointPositions = planner.getReferenceCoPGenerator().getWaypoints();
      copCornerPoints.reset();
      for (int i = 0; i < copCornerPointPositions.size(); i++)
      {
         CoPPointsInFoot copPoints = copCornerPointPositions.get(i);
         for (int j = 0; j < copPoints.getNumberOfCoPPoints(); j++)
         {
            copCornerPoints.setBall(copPoints.getWaypointInWorld(j));
         }
      }
   }

   @SuppressWarnings("unused")
   private void simulateAndAssertSamePlan(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2)
   {
      planFootsteps();

      inDoubleSupport = new YoBoolean("inDoubleSupport", registry);
      inDoubleSupport.set(true);

      for (RobotSide side : RobotSide.values)
         contactStates.get(side).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      for (int currentStepCount = 0; currentStepCount < numberOfFootstepsForTest; )
      {
         addFootsteps(currentStepCount, footstepList, timingList, shiftFractions, planner1);
         addFootsteps(currentStepCount, footstepList, timingList, shiftFractions, planner2);

         updateContactState(currentStepCount, 0.0);
         for (RobotSide robotSide : RobotSide.values)
         {
            soleZUpFrames.get(robotSide).update();
         }

         if (inDoubleSupport.getBooleanValue())
         {
            planner1.setTransferToSide(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner1.initializeForTransfer(yoTime.getDoubleValue());

            planner2.setTransferToSide(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner2.initializeForTransfer(yoTime.getDoubleValue());
         }
         else
         {
            planner1.setSupportLeg(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner1.initializeForSingleSupport(yoTime.getDoubleValue());

            planner2.setSupportLeg(footstepList.get(currentStepCount).getRobotSide().getOppositeSide());
            planner2.initializeForSingleSupport(yoTime.getDoubleValue());
         }

         simulateTicksAndAssertSamePlan(
               (inDoubleSupport.getBooleanValue() ? timingList.get(currentStepCount).getTransferTime() : timingList.get(currentStepCount).getSwingTime()),
               currentStepCount, planner1, planner2);
         currentStepCount = updateStateMachine(currentStepCount);
      }

      addFootsteps(numberOfFootstepsForTest, footstepList, timingList, shiftFractions, planner1);
      addFootsteps(numberOfFootstepsForTest, footstepList, timingList, shiftFractions, planner2);

      updateContactState(-1, 0.0);

      planner1.setTransferToSide(footstepList.get(numberOfFootstepsForTest - 1).getRobotSide());
      planner1.initializeForStanding(yoTime.getDoubleValue());
      planner2.setTransferToSide(footstepList.get(numberOfFootstepsForTest - 1).getRobotSide());
      planner2.initializeForStanding(yoTime.getDoubleValue());
      simulateTicksAndAssertSamePlan(defaultFinalTransferTime, -1, planner1, planner2);
   }

   @SuppressWarnings("unused")
   private void simulate(boolean checkForDiscontinuities, boolean checkForPlanningConsistency, boolean checkIfDynamicsAreSatisfied)
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

      // setting the initial conditions
      planner.setFinalTransferDuration(defaultFinalTransferTime);
      planner.setDefaultPhaseTimes(defaultSwingTime, defaultTransferTime);
      planner.holdCurrentICP(new FramePoint3D(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid()));
      planner.initializeForStanding(yoTime.getDoubleValue());
      simulateTicks(checkForDiscontinuities, checkIfDynamicsAreSatisfied, defaultStandingTime, -1);

      for (int currentStepCount = 0; currentStepCount < numberOfFootstepsForTest; )
      {
         addFootsteps(currentStepCount, footstepList, timingList, shiftFractions, planner);
         updateContactState(currentStepCount, 0.0);
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
         simulateTicks(checkForDiscontinuities, checkIfDynamicsAreSatisfied, (inDoubleSupport.getBooleanValue() ?
               timingList.get(currentStepCount).getTransferTime() :
               timingList.get(currentStepCount).getSwingTime()), currentStepCount);
         currentStepCount = updateStateMachine(currentStepCount);
      }

      addFootsteps(numberOfFootstepsForTest, footstepList, timingList, shiftFractions, planner);
      updateContactState(-1, 0.0);
      planner.setTransferToSide(footstepList.get(numberOfFootstepsForTest - 1).getRobotSide());
      planner.initializeForTransfer(yoTime.getDoubleValue());

      if (visualize)
         updateVisualization(numberOfFootstepsForTest);
      if (checkForPlanningConsistency)
         testForPlanningConsistency(true, numberOfFootstepsForTest);
      simulateTicks(checkForDiscontinuities, checkIfDynamicsAreSatisfied, defaultFinalTransferTime, -1);

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

   private void simulateTicksAndAssertSamePlan(double totalTime, int currentStepCount, SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2)
   {
      for (double timeInState = 0.0; timeInState < totalTime; timeInState += dt)
      {
         updateContactState(currentStepCount, timeInState / totalTime);
         for (RobotSide robotSide : RobotSide.values)
         {
            soleZUpFrames.get(robotSide).update();
         }

         simulateOneTickAndAssertSamePlan(planner1, planner2);
      }
   }

   private void simulateTicks(boolean checkForDiscontinuities, boolean checkIfDyanmicsAreSatisfied, double totalTime, int currentStepCount)
   {
      for (double timeInState = 0.0; timeInState < totalTime; timeInState += dt)
      {
         updateContactState(currentStepCount, timeInState / totalTime);
         simulateOneTick(checkForDiscontinuities, checkIfDyanmicsAreSatisfied);
      }
   }

   private void updateContactState(int currentStepCount, double percentageOfPhase)
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

         if (currentStepCount > 0)
         {
            swingFootPose.set(footstepList.get(currentStepCount - 1).getFootstepPose());
         }
         else
         {
            swingFootPose.set(initialPose);
            swingFootPose.appendTranslation(0.0, swingSide.negateIfRightSide(stepWidth / 2.0), 0.0);
         }

         FramePose3D endOfSwing = footstepList.get(currentStepCount).getFootstepPose();
         swingFootPose.interpolate(endOfSwing, percentageOfPhase);

         FootSpoof foot = feet.get(swingSide);
         foot.setSoleFrame(swingFootPose);
      }

      bipedSupportPolygons.updateUsingContactStates(contactStates);
   }

   private void simulateOneTickAndAssertSamePlan(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2)
   {
      yoTime.add(dt);
      getAllVariablesFromPlanner(planner1, icpPlannerData1);
      getAllVariablesFromPlanner(planner2, icpPlannerData2);

      assertCoPWaypointsAreEqual(planner1, planner2, 1e-10);
      assertCMPWaypointsAreEqual(planner1, planner2, 1e-10);
      assertICPWaypointsAreEqual(planner1, planner2, 1e-10);
      assertCoMPlansAreEqual(planner1, planner2, 1e-10);
      assertPlansAreEqual(icpPlannerData1, icpPlannerData2, 1e-10);
   }

   private static void assertCoPWaypointsAreEqual(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2, double epsilon)
   {
      List<CoPPointsInFoot> waypoints1 = planner1.getReferenceCoPGenerator().getWaypoints();
      List<CoPPointsInFoot> waypoints2 = planner2.getReferenceCoPGenerator().getWaypoints();

      for (int i = 0; i < waypoints1.size(); i++)
      {
         CoPPointsInFoot pointsInFoot1 = waypoints1.get(i);
         CoPPointsInFoot pointsInFoot2 = waypoints2.get(i);

         for (int j = 0; j < pointsInFoot1.getNumberOfCoPPoints(); j++)
         {
            YoFrameEuclideanTrajectoryPoint coPTrajectoryPoint1 = pointsInFoot1.get(j);
            YoFrameEuclideanTrajectoryPoint coPTrajectoryPoint2 = pointsInFoot2.get(j);
            Assert.assertTrue(coPTrajectoryPoint1.epsilonEquals(coPTrajectoryPoint2, epsilon));
         }
      }
   }

   private static void assertCMPWaypointsAreEqual(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2, double epsilon)
   {
      List<CMPTrajectory> swingCMPTrajectories1 = planner1.getReferenceCMPGenerator().getSwingCMPTrajectories();
      List<CMPTrajectory> swingCMPTrajectories2 = planner2.getReferenceCMPGenerator().getSwingCMPTrajectories();
      assertCMPTrajectoryListsAreEqual(epsilon, swingCMPTrajectories1, swingCMPTrajectories2);

      List<CMPTrajectory> transferCMPTrajectories1 = planner1.getReferenceCMPGenerator().getTransferCMPTrajectories();
      List<CMPTrajectory> transferCMPTrajectories2 = planner2.getReferenceCMPGenerator().getTransferCMPTrajectories();
      assertCMPTrajectoryListsAreEqual(epsilon, transferCMPTrajectories1, transferCMPTrajectories2);
   }

   private static void assertCMPTrajectoryListsAreEqual(double epsilon, List<CMPTrajectory> cmpTrajectories1, List<CMPTrajectory> cmpTrajectories2)
   {
      for (int i = 0; i < cmpTrajectories1.size(); i++)
      {
         CMPTrajectory cmpTrajectory1 = cmpTrajectories1.get(i);
         CMPTrajectory cmpTrajectory2 = cmpTrajectories2.get(i);

         for (int j = 0; j < cmpTrajectory1.getNumberOfSegments(); j++)
         {
            Trajectory3D segment1 = cmpTrajectory1.getSegment(j);
            Trajectory3D segment2 = cmpTrajectory2.getSegment(j);
            Assert.assertEquals(segment1.getInitialTime(), segment2.getInitialTime(), epsilon);
            Assert.assertEquals(segment1.getFinalTime(), segment2.getFinalTime(), epsilon);

            for (int k = 0; k < 3; k++)
            {
               Trajectory trajectory1 = segment1.getTrajectory(k);
               Trajectory trajectory2 = segment2.getTrajectory(k);

               for (int l = 0; l < trajectory1.getNumberOfCoefficients(); l++)
               {
                  Assert.assertEquals(trajectory1.getCoefficient(l), trajectory2.getCoefficient(l), epsilon);
               }
            }
         }
      }
   }

   private static void assertICPWaypointsAreEqual(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2, double epsilon)
   {
      List<? extends FramePoint3DReadOnly> icpPositionDesiredInitialList1 = planner1.getReferenceICPGenerator().getICPPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> icpPositionDesiredInitialList2 = planner2.getReferenceICPGenerator().getICPPositionDesiredInitialList();
      assertFramePointListsAreEqual(icpPositionDesiredInitialList1, icpPositionDesiredInitialList2, epsilon);

      List<? extends FramePoint3DReadOnly> icpPositionDesiredFinalList1 = planner1.getReferenceICPGenerator().getICPPositionDesiredFinalList();
      List<? extends FramePoint3DReadOnly> icpPositionDesiredFinalList2 = planner2.getReferenceICPGenerator().getICPPositionDesiredFinalList();
      assertFramePointListsAreEqual(icpPositionDesiredFinalList1, icpPositionDesiredFinalList2, epsilon);
   }

   private static void assertCoMPlansAreEqual(SmoothCMPBasedICPPlanner planner1, SmoothCMPBasedICPPlanner planner2, double epsilon)
   {
      List<? extends FramePoint3DReadOnly> coMPositionDesiredInitialList1 = planner1.getReferenceCoMGenerator().getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> coMPositionDesiredInitialList2 = planner2.getReferenceCoMGenerator().getCoMPositionDesiredInitialList();
      assertFramePointListsAreEqual(coMPositionDesiredInitialList1, coMPositionDesiredInitialList2, epsilon);

      List<? extends FramePoint3DReadOnly> coMPositionDesiredFinalList1 = planner1.getReferenceCoMGenerator().getCoMPositionDesiredFinalList();
      List<? extends FramePoint3DReadOnly> coMPositionDesiredFinalList2 = planner2.getReferenceCoMGenerator().getCoMPositionDesiredFinalList();
      assertFramePointListsAreEqual(coMPositionDesiredFinalList1, coMPositionDesiredFinalList2, epsilon);
   }

   private static void assertFramePointListsAreEqual(List<? extends FramePoint3DReadOnly> waypointList1, List<? extends FramePoint3DReadOnly> waypointList2,
                                                     double epsilon)
   {
      for (int i = 0; i < waypointList1.size(); i++)
      {
         FramePoint3DReadOnly icpWaypoint1 = waypointList1.get(i);
         FramePoint3DReadOnly icpWaypoint2 = waypointList2.get(i);
         EuclidFrameTestTools.assertFrameTuple3DEquals(icpWaypoint1, icpWaypoint2, epsilon);
      }
   }

   private static void assertPlansAreEqual(ICPPlannerData planData1, ICPPlannerData planData2, double epsilon)
   {
      // cop data
      EuclidCoreTestTools.assertTuple3DEquals(planData1.copPosition, planData2.copPosition, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.copVelocity, planData2.copVelocity, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.copAcceleration, planData2.copAcceleration, epsilon);

      // angular momentum data
      EuclidCoreTestTools.assertTuple3DEquals(planData1.centroidalAngularMomentum, planData2.centroidalAngularMomentum, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.centroidalTorque, planData2.centroidalTorque, epsilon);

      // cmp data
      EuclidCoreTestTools.assertTuple3DEquals(planData1.cmpPosition, planData2.cmpPosition, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.cmpVelocity, planData2.cmpVelocity, epsilon);

      // icp data
      EuclidCoreTestTools.assertTuple3DEquals(planData1.icpPosition, planData2.icpPosition, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.icpVelocity, planData2.icpVelocity, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.icpAcceleration, planData2.icpAcceleration, epsilon);

      // com data
      EuclidCoreTestTools.assertTuple3DEquals(planData1.comPosition, planData2.comPosition, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.comVelocity, planData2.comVelocity, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(planData1.comAcceleration, planData2.comAcceleration, epsilon);
   }

   private void simulateOneTick(boolean checkForDiscontinuities, boolean checkIfDyanmicsAreSatisfied)
   {
      yoTime.add(dt);
      getAllVariablesFromPlanner(planner, icpPlannerData1);
      updateUpdatables(yoTime.getDoubleValue());
      if (checkForDiscontinuities)
         testForDiscontinuities(icpPlannerData1);
      if (checkIfDyanmicsAreSatisfied)
         testIfDynamicsAreSatisfied();
      if (visualize)
         updateVisualizePerTick();
   }

   private void getAllVariablesFromPlanner(SmoothCMPBasedICPPlanner planner, ICPPlannerData plannerData)
   {
      planner.compute(yoTime.getDoubleValue());
      planner.getDesiredCenterOfPressurePosition(plannerData.copPosition);
      planner.getDesiredCenterOfPressureVelocity(plannerData.copVelocity);
      planner.getDesiredCenterOfPressureVelocity(plannerData.copAcceleration);
      plannerData.centroidalAngularMomentum.setIncludingFrame(planner.desiredCentroidalAngularMomentum);
      plannerData.centroidalTorque.setIncludingFrame(planner.desiredCentroidalTorque);
      planner.getDesiredCentroidalMomentumPivotPosition(plannerData.cmpPosition);
      planner.getDesiredCentroidalMomentumPivotVelocity(plannerData.cmpVelocity);
      planner.getDesiredCapturePointPosition(plannerData.icpPosition);
      planner.getDesiredCapturePointVelocity(plannerData.icpVelocity);
      planner.getDesiredCapturePointAcceleration(plannerData.icpAcceleration);
      planner.getDesiredCenterOfMassPosition(plannerData.comPosition);
      plannerData.comVelocity.setIncludingFrame(planner.desiredCoMVelocity);
      plannerData.comAcceleration.setIncludingFrame(planner.desiredCoMAcceleration);
   }

   private void planFootsteps()
   {
      FootstepTestHelper footstepHelper = new FootstepTestHelper(feet);
      footstepList = footstepHelper.createFootsteps(stepWidth, stepLength, numberOfFootstepsForTest);
      timingList = new ArrayList<>(footstepList.size());
      shiftFractions = new ArrayList<>(footstepList.size());
      timingList.add(new FootstepTiming(defaultInitialSwingTime, defaultInitialTransferTime));
      shiftFractions.add(new FootstepShiftFractions());
      for (int i = 0; i < footstepList.size() - 1; i++)
      {
         timingList.add(new FootstepTiming(defaultSwingTime, defaultTransferTime));
         shiftFractions.add(new FootstepShiftFractions());
      }
   }

   private void addFootsteps(int currentFootstepIndex, List<Footstep> footstepList, List<FootstepTiming> timingList,
                             List<FootstepShiftFractions> shiftFractions, SmoothCMPBasedICPPlanner planner)
   {
      planner.clearPlan();
      for (int i = currentFootstepIndex; i < Math.min(footstepList.size(), currentFootstepIndex + numberOfFootstepsToConsider); i++)
      {
         planner.addFootstepToPlan(footstepList.get(i), timingList.get(i), shiftFractions.get(i));
      }
   }

   private void testIfDynamicsAreSatisfied()
   {
      assertTrue("CoM dynamics not satisfied, t: " + yoTime.getDoubleValue() + " COM Position: " + icpPlannerData1.comPosition.toString() + " ICP Velocity: "
                       + icpPlannerData1.comVelocity.toString() + " ICP Position: " + icpPlannerData1.icpPosition.toString(),
                 checkCoMDynamics(icpPlannerData1.comPosition, icpPlannerData1.comVelocity, icpPlannerData1.icpPosition));
      assertTrue("ICP dynamics not satisfied, t: " + yoTime.getDoubleValue() + " ICP Position: " + icpPlannerData1.icpPosition.toString() + " ICP Velocity: "
                       + icpPlannerData1.icpVelocity.toString() + " CMP Position: " + icpPlannerData1.cmpPosition.toString(),
                 checkICPDynamics(icpPlannerData1.icpPosition, icpPlannerData1.icpVelocity, icpPlannerData1.cmpPosition));
   }

   private boolean checkICPDynamics(FramePoint3D icpPosition, FrameVector3D icpVelocity, FramePoint3D cmpPosition)
   {
      FrameVector3D icpVelocityFromDynamics = new FrameVector3D();

      icpVelocityFromDynamics.sub(icpPosition, cmpPosition);
      icpVelocityFromDynamics.scale(omega);
      return icpVelocity.epsilonEquals(icpVelocityFromDynamics, epsilon);
   }

   private boolean checkCoMDynamics(FramePoint3D comPosition, FrameVector3D comVelocity, FramePoint3D icpPosition)
   {
      FrameVector3D comVelocityFromDynamics = new FrameVector3D();

      comVelocityFromDynamics.sub(icpPosition, comPosition);
      comVelocityFromDynamics.scale(omega);

      return comVelocity.epsilonEquals(comVelocityFromDynamics, epsilon);
   }

   private static void assertTrajectoryPointEquals(String prefix, YoFrameEuclideanTrajectoryPoint expected, YoFrameEuclideanTrajectoryPoint actual, double epsilon)
   {
      Assert.assertEquals(prefix, expected.getTime(), actual.getTime(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(prefix, expected.getPosition(), actual.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(prefix, expected.getLinearVelocity(), actual.getLinearVelocity(), epsilon);
   }
}