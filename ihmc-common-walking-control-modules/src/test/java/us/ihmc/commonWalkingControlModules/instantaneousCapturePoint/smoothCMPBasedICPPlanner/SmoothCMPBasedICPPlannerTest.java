package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner;

import static org.junit.Assert.assertTrue;

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
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SmoothCMPBasedICPPlannerTest
{
   private static final String testClassName = "UltimateSmoothCMPBasedICPPlannerTest";
   private static final double epsilon = Epsilons.ONE_TEN_MILLIONTH;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final boolean VISUALIZE = true;
   private static final boolean keepSCSUp = false;

   // Simulation parameters
   private final double dt = 0.0001;
   private final double omega = 3.5;

   // Physical parameters
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

   private final double stepWidth = 0.5;
   private final double stepLength = 0.5;
   private final int numberOfFootstepsToConsider = 3;
   private final int numberOfFootstepsToTest = 10;
   private final List<Point2D> contactPointsInFootFrame = Stream.of(new Point2D(footLengthForward, toeWidth / 2.0),
                                                                    new Point2D(footLengthForward, -toeWidth / 2.0),
                                                                    new Point2D(footLengthBack, heelWidth / 2.0),
                                                                    new Point2D(footLengthBack, -heelWidth / 2.0))
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
   private final FrameVector3D cmpAcceleration = new FrameVector3D();
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
   private FullRobotModel dummyRobotForTesting;
   private List<Footstep> footstepList;
   private List<FootstepTiming> timingList;

   // Variables for visualization
   private YoGraphicsListRegistry graphicsListRegistry;
   private int numberOfTrackBalls = 100;
   private double trackBallSize = 0.02;
   private int numberOfCornerPoints = numberOfFootstepsToConsider * 6 + 1;
   private double cornerPointBallSize = 0.05;
   private double footstepHeight = 0.01;
   private BagOfBalls comTrack, icpTrack, cmpTrack, copTrack;
   private BagOfBalls comCornerPoints, icpCornerPoints, copCornerPoints;
   private List<YoFramePose> nextFootstepPoses;
   private SideDependentList<YoFramePose> currentFootLocations;
   private ArrayList<Updatable> updatables;
   private SimulationConstructionSet scs;
   private int SCS_BUFFER_SIZE = 100000;

   @Before
   public void setupTest()
   {
      this.registry = new YoVariableRegistry(testClassName);
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
      if (VISUALIZE)
         setupVisualization();

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            if (VISUALIZE)
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

   private void setupCurrentFootPoseVisualization()
   {
      currentFootLocations = new SideDependentList<>();
      for (RobotSide side : RobotSide.values())
      {
         Graphics3DObject footstepGraphic = new Graphics3DObject();
         footstepGraphic.addExtrudedPolygon(contactPointsInFootFrame, footstepHeight);
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
         nextFootstepGraphic.addExtrudedPolygon(contactPointsInFootFrame, footstepHeight);
         YoFramePose nextFootstepPose = new YoFramePose("NextFootstep" + i + "Pose", worldFrame, registry);
         nextFootstepPoses.add(nextFootstepPose);
         graphicsListRegistry.registerYoGraphic("UpcomingFootsteps",
                                                new YoGraphicShape("NextFootstep" + i + "Viz", nextFootstepGraphic, nextFootstepPose, 1.0));
      }
   }

   private void setupCornerPointBallsVisualization()
   {
      comCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "CoMCornerPoint", YoAppearance.Black(), registry, graphicsListRegistry);
      icpCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "ICPCornerPoint", YoAppearance.White(), registry, graphicsListRegistry);
      copCornerPoints = new BagOfBalls(numberOfCornerPoints, cornerPointBallSize, "CoPCornerPoint", YoAppearance.White(), registry, graphicsListRegistry);
   }

   private void setupTrackBallsVisualization()
   {
      comTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CoMTrack", YoAppearance.Black(), registry, graphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "ICPTrack", YoAppearance.Red(), registry, graphicsListRegistry);
      cmpTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CMPTrack", YoAppearance.Purple(), registry, graphicsListRegistry);
      copTrack = new BagOfBalls(numberOfTrackBalls, trackBallSize, "CoPTrack", YoAppearance.Green(), registry, graphicsListRegistry);
   }

   @After
   public void cleanUpTest()
   {
      if(scs != null)
         scs.closeAndDispose();
   }

   @Test
   public void testForDiscontinuitiesWithoutAngularMomentum()
   {
      boolean isAMOn = false;
      setupPlanner(isAMOn);
      simulate(true, false, true);
   }

   @Test
   public void testForDiscontinuitiesWithAngularMomentum()
   {
      boolean isAMOn = true;
      setupPlanner(isAMOn);
      simulate(true, false, true);
   }

   @Test
   public void testForPlanningConsistencyWithoutAngularMomentum()
   {
      boolean isAMOn = false;
      setupPlanner(isAMOn);
      simulate(false, true, true);
   }

   @Test
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
      this.planner = new SmoothCMPBasedICPPlanner(dummyRobotForTesting, bipedSupportPolygons, feet, plannerParameters.getNumberOfFootstepsToConsider(),
                                                  plannerParameters.getNumberOfCoPWayPointsPerFoot(), registry, graphicsListRegistry, gravity);
   }

   private void testForDiscontinuities()
   {

   }

   private void testForPlanningConsistency()
   {

   }

   private void updateVisualizePerTick(double time)
   {
      updateCoMTrack();
      updateICPTrack();
      updateCMPTrack();
      updateCoPTrack();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   private void updateCoMTrack()
   {
      comTrack.setBall(comPosition);
   }

   private void updateICPTrack()
   {
      icpTrack.setBall(icpPosition);
   }

   private void updateCMPTrack()
   {
      cmpTrack.setBall(cmpPosition);
   }

   private void updateCoPTrack()
   {
      copTrack.setBall(copPosition);
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
      int numberOfStepToUpdate = (numberOfFootstepsToTest - stepIndex) < numberOfFootstepsToConsider ? (numberOfFootstepsToTest - stepIndex)
            : numberOfFootstepsToConsider;
      for (int i = 0; i < numberOfStepToUpdate; i++)
      {
         nextFootstepPoses.get(i).set(footstepList.get(stepIndex + i).getFootstepPose());
      }
   }

   private void updateCoMCornerPoints()
   {

   }

   private void updateICPCornerPoints()
   {

   }

   private void updateCoPCornerPoints()
   {

   }

   @SuppressWarnings("unused")
   private void simulate(boolean checkForDiscontinuities, boolean checkForPlanningConsistency, boolean checkIfDyanmicsAreSatisfied)
   {
      if (VISUALIZE)
         startSCS();

      planFootsteps();
      inDoubleSupport = new YoBoolean("inDoubleSupport", registry);
      inDoubleSupport.set(true);

      for (RobotSide side : RobotSide.values)
         contactStates.get(side).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      for (int currentStepCount = 0; currentStepCount < numberOfFootstepsToTest;)
      {
         planner.clearPlan();
         addFootsteps(currentStepCount, footstepList, timingList);
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
         updateVisualization(currentStepCount);

         for (double timeInState = 0.0; timeInState < (inDoubleSupport.getBooleanValue() ? timingList.get(currentStepCount).getTransferTime()
               : timingList.get(currentStepCount).getSwingTime()); timeInState += dt)
         {
            simulateOneTick(checkForDiscontinuities, checkIfDyanmicsAreSatisfied);
         }

         // Prep for the next state
         if (inDoubleSupport.getBooleanValue())
         {
            inDoubleSupport.set(false);
         }
         else
         {
            inDoubleSupport.set(true);
            currentStepCount++;
         }
      }

      if (VISUALIZE && keepSCSUp)
         ThreadTools.sleepForever();
   }

   private void simulateOneTick(boolean checkForDiscontinuities, boolean checkIfDyanmicsAreSatisfied)
   {
      yoTime.add(dt);
      if (VISUALIZE)
         updateVisualizePerTick(yoTime.getDoubleValue());
      if (checkForDiscontinuities)
         testForDiscontinuities();
      if (checkIfDyanmicsAreSatisfied)
         testIfDynamicsAreSatisified();
   }

   private void planFootsteps()
   {
      FootstepTestHelper footstepHelper = new FootstepTestHelper(feet);
      footstepList = footstepHelper.createFootsteps(stepWidth, stepLength, numberOfFootstepsToTest);
      timingList = new ArrayList<>(footstepList.size());
      timingList.add(new FootstepTiming(defaultSwingTime, defaultInitialTransferTime));
      for (int i = 0; i < footstepList.size() - 1; i++)
         timingList.add(new FootstepTiming(defaultSwingTime, defaultTransferTime));
   }

   private void addFootsteps(int currentFootstepIndex, List<Footstep> footstepList, List<FootstepTiming> timingList)
   {
      for (int i = currentFootstepIndex; i < Math.min(footstepList.size(), currentFootstepIndex + numberOfFootstepsToConsider); i++)
      {
         planner.addFootstepToPlan(footstepList.get(i), timingList.get(i));
      }
   }

   private void testIfDynamicsAreSatisified()
   {
      assertTrue("CoM dynamics not satisfied, t: " + yoTime.getDoubleValue() + " COM Position: " + comPosition.toString() + " ICP Velocity: "
            + comVelocity.toString() + " ICP Position: " + icpPosition.toString(), checkCoMDynamics(comPosition, comVelocity, icpPosition));
      assertTrue("ICP dynamics not satisfied, t: " + yoTime.getDoubleValue() + " ICP Position: " + icpPosition.toString() + " ICP Velocity: "
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
}