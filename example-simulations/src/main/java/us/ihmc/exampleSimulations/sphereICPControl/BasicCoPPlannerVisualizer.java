package us.ihmc.exampleSimulations.sphereICPControl;

import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.footLengthForControl;
import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.footWidthForControl;
import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.toeWidthForControl;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultLeftColor;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultRightColor;
import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.BasicCoPPlanner;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
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
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class BasicCoPPlannerVisualizer
{
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.006;
   private final ArrayList<Updatable> updatables = new ArrayList<>();

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private ReferenceFrame midFeetZUpFrame;

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private static final double singleSupportDuration = 0.6; /// 0.7;
   private static final double doubleSupportDuration = 0.05; // 0.4; //0.25;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPViz");

   private final YoFramePoint3D yoDesiredCoP = new YoFramePoint3D("desiredCoP", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final int simulatedTicksPerGraphicUpdate = 1;
   private final double trailingDuration = 4.0 * (singleSupportDuration + doubleSupportDuration);
   private final int numberOfBalls = (int) (trailingDuration / dt / simulatedTicksPerGraphicUpdate);

   private BipedSupportPolygons bipedSupportPolygons;
   private final BagOfBalls copTrack;

   private final BasicCoPPlanner copPlanner;

   public BasicCoPPlannerVisualizer()
   {
      copTrack = new BagOfBalls(numberOfBalls, 0.005, "CoP", YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);

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

      FootstepTestHelper footstepTestHelper = new FootstepTestHelper(contactableFeet);
      List<FootstepTiming> timings = new ArrayList<>();
      List<Footstep> footsteps = footstepTestHelper.createFootsteps(0.25, 0.20, 20);
      for (int i = 0; i < footsteps.size(); i++)
         timings.add(new FootstepTiming(singleSupportDuration, doubleSupportDuration));

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
      copPlanner = new BasicCoPPlanner(contactableFeet, midFeetZUpFrame, registry);

      YoGraphicPosition desiredCoPGraphic = new YoGraphicPosition("desiredCoP", yoDesiredCoP, 0.01, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("desiredCoP", desiredCoPGraphic);
      yoGraphicsListRegistry.registerArtifact("desiredCoP", desiredCoPGraphic.createArtifact());

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

   private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void simulate(List<Footstep> footsteps, List<FootstepTiming> timings)
   {
      boolean isInDoubleSupport = true;

      Footstep nextFootstep = null;
      int currentFootstepPolled = 0;

      FootstepTiming nextFootstepTiming = null;

      while (currentFootstepPolled <= footsteps.size())
      {
         copPlanner.clearPlan();
         if (isInDoubleSupport)
         {
            currentFootstepPolled++;

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

         copPlanner.submitFootstep(nextFootstep, nextFootstepTiming);
         copPlanner.submitFootstep(nextNextFootstep, nextNextFootstepTiming);
         copPlanner.submitFootstep(nextNextNextFootstep, nextNextNextFootstepTiming);

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
               copPlanner.initializeForStanding();
            else
               copPlanner.initializeForTransfer(yoTime.getDoubleValue());

            isInDoubleSupport = false;
         }
         else if (nextFootstep != null)
         {
            RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
            copPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

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

         while (yoTime.getDoubleValue() < 20.0)
         {
            simulateOneTick();
            if (copPlanner.isDone())
               break;
         }
      }

      for (double additionalTime = 0.0; additionalTime < 1.0; additionalTime += dt)
         simulateOneTick();
   }

   private int counter = 0;
   private final FramePoint3D desiredCoP = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();

   private void simulateOneTick()
   {
      callUpdatables();
      updateFootViz();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      copPlanner.compute(yoTime.getDoubleValue());
      copPlanner.getDesiredCoPData(desiredCoP, desiredCoPVelocity, desiredCoPAcceleration);
      yoDesiredCoP.set(desiredCoP);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         copTrack.setBallLoop(desiredCoP);
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

   public static void main(String[] args)
   {
      new BasicCoPPlannerVisualizer();
   }
}
