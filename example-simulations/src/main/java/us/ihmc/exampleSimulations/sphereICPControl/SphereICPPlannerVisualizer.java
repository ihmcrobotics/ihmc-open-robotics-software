package us.ihmc.exampleSimulations.sphereICPControl;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.thoughtworks.xstream.io.StreamException;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.exampleSimulations.sphereICPControl.controllers.SphereControlToolbox.SphereSmoothCMPPlannerParameters;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SphereICPPlannerVisualizer
{
   private static final boolean useTwoCMPs = true;
   private static final int BUFFER_SIZE = 16000;
   private static final double initialTransferDuration = 1.0;

   private static final double singleSupportDuration = 0.6; /// 0.7;
   private static final double doubleSupportDuration = 0.05; // 0.4; //0.25;

   private static final double maxDurationForSmoothingEntryToExitCMPSwitch = 0.5;
   private static final double timeSpentOnExitCMPInPercentOfStepTime = 0.5; // singleSupportDuration
   // /
   // stepTime;
   private static final double doubleSupportSplitFraction = 0.5;

   private static final int maxNumberOfFootstepToPoll = 30;

   private static final double footLengthForControl = 0.2;
   private static final double toeWidthForControl = 0.15;
   private static final double footWidthForControl = 0.15;

   private static final boolean USE_FLAT_GROUND_WALKING_TRACK_FOOTSTEP_PROVIDER = false;
   private static final boolean USE_SCRIPT = false;
   private static final boolean USE_FEET_OSCILLATOR = false;
   private static final boolean USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z = false;
   private static final boolean USE_RANDOM_CONTACT_POINTS = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private final SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();

   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private BipedSupportPolygons bipedSupportPolygons;

   private final SimulationConstructionSet scs;
   private final SmoothCMPBasedICPPlanner icpPlanner;

   private final YoDouble yoTime;
   private final double dt = 0.006;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPViz");
   private final YoFramePoint3D eCMP = new YoFramePoint3D("eCMP", worldFrame, registry);
   private final YoFramePoint3D desiredICP = new YoFramePoint3D("desiredICP", worldFrame, registry);
   private final YoFrameVector3D desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);

   private final Random random = new Random(21616L);
   private final double steppingError = 0.0 * 0.05;

   private final double footLinearOscillationAmplitude = 0.0 * 0.1;
   private final double footYawOscillationAmplitude = 0.2;
   private final double footPitchRollOscillationAmplitude = 0.1;
   private final double footXYYawOscillationFrequency = 1.0;
   private final double footPitchRollOscillationFrequency = 1.0;
   private final double footZShift = 0.0 * 1.5;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());

   private final ArrayList<Updatable> updatables = new ArrayList<>();

   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final YoFramePoint2D centerOfMass = new YoFramePoint2D("centerOfMass", worldFrame, registry);
   private final YoFrameVector2D centerOfMassVelocity = new YoFrameVector2D("centerOfMassVelocity", worldFrame, registry);

   private final double omega0 = 3.4;
   private final double mass = 100.0;
   private final double gravity = 9.81;

   private final BagOfBalls cmpTrack, icpTrack;
   private final int simulatedTicksPerGraphicUpdate = 16;
   private final double trailingDuration = 4.0 * (singleSupportDuration + doubleSupportDuration);
   private final int numberOfBalls = (int) (trailingDuration / dt / simulatedTicksPerGraphicUpdate);

   private FootstepTestHelper footstepTestHelper;

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public SphereICPPlannerVisualizer()
   {

      yoGraphicsListRegistry.registerArtifact("Capture Point", new YoGraphicPosition("eCMP", eCMP, 0.012, YoAppearance.Purple(), YoGraphicPosition.GraphicType.CROSS).createArtifact());
      yoGraphicsListRegistry.registerArtifact("Capture Point", new YoGraphicPosition("Desired Capture Point", desiredICP, 0.01, YoAppearance.Yellow(), GraphicType.ROTATED_CROSS).createArtifact());
      yoGraphicsListRegistry.registerArtifact("Center of Mass", new YoGraphicPosition("Center Of Mass", centerOfMass, 0.01, YoAppearance.Grey(), GraphicType.ROTATED_CROSS).createArtifact());

      if (USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z)
         desiredICP.setZ(footZShift);

      icpPlanner = setupPlanner(yoGraphicsListRegistry, registry);

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false));

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<Point2D>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0));

      cmpTrack = new BagOfBalls(numberOfBalls, 0.003, "eCMP", YoAppearance.Purple(), registry, yoGraphicsListRegistry);
      icpTrack = new BagOfBalls(numberOfBalls, 0.003, "ICP", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);

      List<Footstep> footsteps;
      List<FootstepTiming> timings;
      if (USE_SCRIPT)
         //         footstepProvider = createScriptBasedFootstepProvider("UpAndDownSlantedCB.xml", "SlantedCB.xml", "LongSidesteps.xml", "TestSteppingSameSideSeveralTimes.xml");
         footsteps = createScriptBasedFootstepProvider("LongStepsForward.xml");

         //         footstepProvider = createScriptBasedFootstepProvider("LongSidesteps.xml");
         //      else if (USE_FLAT_GROUND_WALKING_TRACK_FOOTSTEP_PROVIDER)
         //         footstepProvider = footstepProviderTestHelper.createFlatGroundWalkingTrackFootstepProvider(registry, updatables);
         //      else
         footsteps = footstepTestHelper.createFootsteps(0.25, 0.20, 20);
         timings = new ArrayList<>();
         setFootstepTimings(timings, footsteps.size());

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               soleFrames.get(robotSide).update();
               soleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulate(footsteps, timings);
      ThreadTools.sleepForever();
   }

   private void setFootstepTimings(List<FootstepTiming> timings, int numberOfSteps)
   {
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         double swingTime = singleSupportDuration, transferTime = doubleSupportDuration;

         switch (stepIndex)
         {
         case 0:
            // start with very slow swings and transfers
            transferTime = 1.0; // initial transfer
            swingTime = 3.0;
            break;
         case 1:
            // do a default step
            break;
         case 2:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            break;
         case 3:
            // do a default step
            break;
         case 4:
            // do a default step
            break;
         case 5:
            // do a fast swing and transfer
            transferTime = 0.2;
            swingTime = 0.6;
            break;
         case 6:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            break;
         case 7:
            // do a fast swing and transfer
            transferTime = 0.2;
            swingTime = 0.6;
            break;
         case 8:
            // do a slow swing
            transferTime = 1.0;
            swingTime = 3.0;
            break;
         case 9:
            // do a slow transfer and a fast swing
            transferTime = 3.0;
            swingTime = 0.6;
            break;
         default:
            break;
         }

         FootstepTiming timing = new FootstepTiming();
         timing.setTimings(swingTime, transferTime);
         timings.add(timing);
      }
   }

   private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void simulate(List<Footstep> footsteps, List<FootstepTiming> timings)
   {
      boolean isInDoubleSupport = true;

      Footstep nextFootstep = null;
      int currentFostepPolled = 0;

      FootstepTiming nextFootstepTiming = null;

      while (currentFostepPolled <= maxNumberOfFootstepToPoll)
      {
         icpPlanner.clearPlan();
         if (isInDoubleSupport)
         {
            currentFostepPolled++;

            if (footsteps.isEmpty())
               break;

            nextFootstep = footsteps.remove(0);
            nextFootstepTiming = timings.remove(0);
         }

         Footstep nextNextFootstep = footsteps.size() > 0 ? footsteps.get(0) : null;
         Footstep nextNextNextFootstep = footsteps.size() > 1 ? footsteps.get(1) : null;
         FootstepTiming nextNextFootstepTiming = timings.size() > 0 ? timings.get(0) : null;
         FootstepTiming nextNextNextFootstepTiming = timings.size() > 1 ? timings.get(1) : null;

         updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

         icpPlanner.addFootstepToPlan(nextFootstep, nextFootstepTiming);
         icpPlanner.addFootstepToPlan(nextNextFootstep, nextNextFootstepTiming);
         icpPlanner.addFootstepToPlan(nextNextNextFootstep, nextNextNextFootstepTiming);

         if (isInDoubleSupport)
         {
            for (RobotSide robotSide : RobotSide.values)
               contactStates.get(robotSide).setFullyConstrained();
         }
         else if (nextFootstep != null)
         {
            RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
            contactStates.get(supportSide.getOppositeSide()).clear();
         }

         bipedSupportPolygons.updateUsingContactStates(contactStates);
         callUpdatables();
         if (isInDoubleSupport)
         {
            if (footsteps.isEmpty())
            {
               icpPlanner.initializeForStanding(yoTime.getDoubleValue());
            }
            else
            {
               RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();
               icpPlanner.setTransferToSide(transferToSide);
               icpPlanner.initializeForTransfer(yoTime.getDoubleValue());

            }
            isInDoubleSupport = false;
         }
         else if (nextFootstep != null)
         {
            RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
            icpPlanner.setSupportLeg(supportSide);
            icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

            FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
            FramePose3D nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
            nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
            nextSupportPose.changeFrame(worldFrame);
            footSpoof.setSoleFrame(nextSupportPose);
            if (nextFootstep.getPredictedContactPoints() == null)
               contactStates.get(supportSide.getOppositeSide()).setContactFramePoints(footSpoof.getContactPoints2d());
            else
               contactStates.get(supportSide.getOppositeSide()).setContactPoints(nextFootstep.getPredictedContactPoints());
            isInDoubleSupport = true;
         }

         while (true)
         {
            simulateOneTick();
            if (icpPlanner.isDone())
               break;
         }
      }

      for (double additionalTime = 0.0; additionalTime < 1.0; additionalTime += dt)
         simulateOneTick();
   }

   private int counter = 0;
   private final FrameVector2D tempVector2d = new FrameVector2D();

   private void simulateOneTick()
   {
      callUpdatables();
      xYAndYawFeetOscillations(yoTime.getDoubleValue());
      pitchRollFeetOscillations(yoTime.getDoubleValue());
      updateFootViz();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      icpPlanner.compute(yoTime.getDoubleValue());
      icpPlanner.getDesiredCapturePointPosition(desiredICP);
      icpPlanner.getDesiredCapturePointVelocity(desiredICPVelocity);
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega0, eCMP);

      centerOfMassVelocity.set(desiredICP);
      centerOfMassVelocity.sub(centerOfMass);
      centerOfMassVelocity.scale(omega0);

      tempVector2d.set(centerOfMassVelocity);
      tempVector2d.scale(dt);
      centerOfMass.add(tempVector2d);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         icpTrack.setBallLoop(desiredICP);
         cmpTrack.setBallLoop(eCMP);
      }
      yoTime.add(dt);

      scs.tickAndUpdate();
   }

   private void updateFootViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (contactStates.get(robotSide).inContact())
         {
            FramePose3D footPose = new FramePose3D(contactableFeet.get(robotSide).getSoleFrame());
            footPose.changeFrame(worldFrame);
            currentFootPoses.get(robotSide).set(footPose);
         }
         else
         {
            currentFootPoses.get(robotSide).setToNaN();
         }
      }
   }

   private void xYAndYawFeetOscillations(double time)
   {
   }

   private void pitchRollFeetOscillations(double time)
   {
   }

   private void generateRandomPredictedContactPoints(Footstep footstep)
   {
      FrameConvexPolygon2D randomSupportPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d()));
      List<FramePoint2D> randomPredictedContactPointList = new ArrayList<>();

      double minX = 0.5 * randomSupportPolygon.getMinX();
      double minY = 0.5 * randomSupportPolygon.getMinY();
      double maxX = 0.5 * randomSupportPolygon.getMaxX();
      double maxY = 0.5 * randomSupportPolygon.getMaxY();
      FramePoint2D randomLineOrigin = EuclidFrameRandomTools.nextFramePoint2D(random, randomSupportPolygon.getReferenceFrame(), minX, maxX, minY, maxY);
      FrameVector2D randomLineVector = EuclidFrameRandomTools.nextFrameVector2D(random, randomSupportPolygon.getReferenceFrame());
      FrameLine2D randomLine = new FrameLine2D(randomLineOrigin, randomLineVector);

      convexPolygonTools.cutPolygonWithLine(randomLine, randomSupportPolygon, RobotSide.generateRandomRobotSide(random));
      randomSupportPolygon.update();

      while (randomSupportPolygon.getNumberOfVertices() < 4)
      {
         FramePoint2D duplicate = EuclidFrameRandomTools.nextFramePoint2D(random, randomSupportPolygon.getReferenceFrame(), 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3);
         duplicate.add(new FramePoint2D(randomSupportPolygon.getVertex(0)));
         randomSupportPolygon.addVertex(duplicate);
         randomSupportPolygon.update();
      }

      if (randomSupportPolygon.getNumberOfVertices() != 4)
         System.out.println();

      for (int i = 0; i < randomSupportPolygon.getNumberOfVertices(); i++)
         randomPredictedContactPointList.add(new FramePoint2D(randomSupportPolygon.getVertex(i)));

      footstep.setPredictedContactPoints(randomPredictedContactPointList);
   }

   private void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep, Footstep nextNextNextFootstep)
   {
      if (nextFootstep == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.clear();
         yoNextNextFootstepPolygon.clear();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextFootstep.getPredictedContactPoints() == null)
         nextFootstep.setPredictedContactPoints(contactableFeet.get(nextFootstep.getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrame(nextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextFootstepPose = new FramePose3D(nextFootstep.getSoleReferenceFrame());
      yoNextFootstepPose.setMatchingFrame(nextFootstepPose);

      if (nextNextFootstep == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.clear();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextNextFootstep.getPredictedContactPoints() == null)
         nextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrame(nextNextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextNextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextNextFootstepPose = new FramePose3D(nextNextFootstep.getSoleReferenceFrame());
      yoNextNextFootstepPose.setMatchingFrame(nextNextFootstepPose);

      if (nextNextNextFootstep == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.clear();
         return;
      }

      if (nextNextNextFootstep.getPredictedContactPoints() == null)
         nextNextNextFootstep.setPredictedContactPoints(contactableFeet.get(nextNextNextFootstep.getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrame(nextNextNextFootstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(nextNextNextFootstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.set(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(nextNextNextFootstep.getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setMatchingFrame(nextNextNextFootstepPose);
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }

   private List<Footstep> createScriptBasedFootstepProvider(String... scriptFileNames)
   {
      ArrayList<Footstep> footsteps = new ArrayList<>();

      for (String scriptFileName : scriptFileNames)
      {
         ScriptFileLoader scriptFileLoader = null;
         try
         {
            scriptFileLoader = new ScriptFileLoader(getClass().getResourceAsStream(scriptFileName));
         }
         catch (IOException | StreamException e)
         {
            System.out.println("Script being loaded: " + scriptFileName);
            e.printStackTrace();
         }
         ArrayList<ScriptObject> scriptObjects = scriptFileLoader.readIntoList();
         scriptFileLoader.close();
         ArrayList<FootstepDataListMessage> footstepDataLists = new ArrayList<>();
         for (ScriptObject scriptObject : scriptObjects)
         {
            if (scriptObject.getScriptObject() instanceof FootstepDataListMessage)
               footstepDataLists.add((FootstepDataListMessage) scriptObject.getScriptObject());
         }

         for (FootstepDataListMessage footstepDataList : footstepDataLists)
         {
            List<FootstepDataMessage> dataList = footstepDataList.getFootstepDataList();
            for (int i = 0; i < dataList.size(); i++)
            {
               FootstepDataMessage footstepData = dataList.get(i);
               RobotSide robotSide = RobotSide.fromByte(footstepData.getRobotSide());
               Point3D position = footstepData.getLocation();
               Quaternion orientation = footstepData.getOrientation();
               footsteps.add(footstepTestHelper.createFootstep(robotSide, position, orientation));
            }
         }
      }


      return corruptFootsteps(footsteps);
   }

   private List<Footstep> corruptFootsteps(final ArrayList<Footstep> footsteps)
   {
      if (USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z)
      {
         for (Footstep footstep : footsteps)
            footstep.setZ(footstep.getZ() + footZShift);
      }
//      else
//         for (Footstep footstep : footsteps)
//            footstep.setZ(footstep.getZ() + 0.10);

      if (USE_RANDOM_CONTACT_POINTS)
         for (Footstep footstep : footsteps)
            generateRandomPredictedContactPoints(footstep);

      return footsteps;
   }

   private SmoothCMPBasedICPPlanner setupPlanner(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         if (USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z)
            startingPose.setZ(footZShift);
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePoseUsingYawPitchRoll(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet);

      SmoothCMPPlannerParameters capturePointPlannerParameters = new SphereSmoothCMPPlannerParameters();

      SmoothCMPBasedICPPlanner icpPlanner = new SmoothCMPBasedICPPlanner(mass, bipedSupportPolygons, soleZUpFrames, contactableFeet, null, yoTime, registry,
                                                                         yoGraphicsListRegistry, gravity, capturePointPlannerParameters);
      icpPlanner.setOmega0(omega0);
      icpPlanner.setFinalTransferDuration(1.0);
      return icpPlanner;
   }

   public static void main(String[] args)
   {
      new SphereICPPlannerVisualizer();
   }
}
