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

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPJumpingDDPCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
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
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationData;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class SLIPJumpingDDPCalculatorVisualizer
{
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.01;

   private static final double maxSupportForExtension = 0.3;
   private static final double firstTransferDuration = 0.3;
   private static final double secondTransferDuration = 0.3;
   private static final double landingAngle = Math.toRadians(20);

   private static final double nominalComHeight = 1.0;
   private static final double length = 1.5;
   private static final double gravityZ = 9.81;
   private static final double height = 0.3;

   private static final double mass = 150.0;

   private static final int numberOfJumps = 3;
   private static final int numberOfHeights = 2;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPViz");

   private final List<YoFramePose> yoNextFootstepPoses = new ArrayList<>();
   private final List<YoFramePose> yoNextNextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2d> yoNextFootstepPolygons = new ArrayList<>();
   private final List<YoFrameConvexPolygon2d> yoNextNextFootstepPolygons = new ArrayList<>();

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean computeNextPass = new YoBoolean("computeNextPass", registry);
   private final YoInteger iterations = new YoInteger("iterations", registry);

   private final int simulatedTicksPerGraphicUpdate = 1;
   private final int numberOfBalls = (int) (3.0 / dt / simulatedTicksPerGraphicUpdate);

   private final List<BagOfBalls> copTracks = new ArrayList<>();
   private final List<BagOfBalls> comTracks = new ArrayList<>();
   private final List<SLIPJumpingDDPCalculator> ddpSolvers = new ArrayList<>();

   private final List<Footstep> leftFoot = new ArrayList<>();
   private final List<Footstep> rightFoot = new ArrayList<>();


   private final YoInteger updatesPerRequest = new YoInteger("updatesPerRequest", registry);
   private final YoDouble trajectoryDT = new YoDouble("trajectoryDT", registry);

   public SLIPJumpingDDPCalculatorVisualizer()
   {
      for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
      {
         copTracks.add(new BagOfBalls(numberOfBalls, 0.005, "CoP" + i, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry));
         comTracks.add(new BagOfBalls(numberOfBalls, 0.005, "CoM" + i, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry));
         ddpSolvers.add(new SLIPJumpingDDPCalculator(dt, mass, nominalComHeight, gravityZ));

         yoNextFootstepPoses.add(new YoFramePose("nextFootstepPose" + i, worldFrame, registry));
         yoNextNextFootstepPoses.add(new YoFramePose("nextNextFootstepPose" + i, worldFrame, registry));
         yoNextFootstepPolygons.add(new YoFrameConvexPolygon2d("nextFootstep" + i, "", worldFrame, 4, registry));
         yoNextNextFootstepPolygons.add(new YoFrameConvexPolygon2d("nextNextFootstep" + i, "", worldFrame, 4, registry));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = new FramePose3D(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         YoFramePose currentFootPose = new YoFramePose(sidePrefix + "FootPose", worldFrame, registry);

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPose, 1.0));
      }

      updatesPerRequest.set(10);
      trajectoryDT.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
               ddpSolvers.get(i).setDeltaT(trajectoryDT.getDoubleValue());
         }
      });
      trajectoryDT.set(dt);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
      {
         yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep" + i, footstepGraphics, yoNextFootstepPoses.get(i), 1.0));
         yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep" + i, footstepGraphics, yoNextNextFootstepPoses.get(i), 1.0));

         yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep" + i, yoNextFootstepPolygons.get(i), Color.blue, false));
         yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep" + i, yoNextNextFootstepPolygons.get(i), Color.blue, false));
      }

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
      scs.setCameraPosition(-5.0, 5.0, 5.0);
      scs.setCameraFix(0.0, 0.0, 0.5);

      scs.startOnAThread();

      for (int i = 0; i < numberOfJumps; i++)
      {
         double lengthFraction = ((double)(numberOfJumps - i)) / ((double) numberOfJumps);
         for (int j = 0; j < numberOfHeights; j++)
         {
            double heightFraction = ((double) j) / ((double) numberOfHeights);
            leftFoot.add(new Footstep(RobotSide.LEFT, new FramePose3D(new FramePoint3D(worldFrame, lengthFraction * length, 0.1 - 0.5 * (numberOfHeights * i + j), heightFraction * height), new FrameQuaternion())));
            rightFoot.add(new Footstep(RobotSide.RIGHT, new FramePose3D(new FramePoint3D(worldFrame, lengthFraction * length, -0.1 - 0.5 * (numberOfHeights * i + j), heightFraction * height), new FrameQuaternion())));
         }
      }

      simulate();
      ThreadTools.sleepForever();
   }


   private final FramePoint3D startPoint = new FramePoint3D();
   private final FramePoint3D endPoint = new FramePoint3D();
   private final FramePoint3D apexPoint = new FramePoint3D();

   private void simulate()
   {
      for (int jumpNumber = 0; jumpNumber < numberOfJumps; jumpNumber++)
      {
         double lengthFraction = ((double) (numberOfJumps - jumpNumber)) / ((double) numberOfJumps);
         for (int heightNumber = 0; heightNumber < numberOfHeights; heightNumber++)
         {
            int number = jumpNumber * numberOfHeights + heightNumber;

            double heightFraction = ((double) heightNumber) / ((double) numberOfHeights);
            startPoint.set(0, -0.5 * number, 0.0);
            endPoint.set(lengthFraction * length, -0.5 * number, heightFraction * height);

            updateUpcomingFootstepsViz(number, leftFoot.get(number), rightFoot.get(number));

            DenseMatrix64F currentCoMState;
            currentCoMState = new DenseMatrix64F(SLIPState.stateVectorSize, 1);
            currentCoMState.set(SLIPState.y, 0, -0.5 * number);
            currentCoMState.set(SLIPState.z, 0, 1.0);

            double jumpLength = startPoint.distance(endPoint);
            double heightChange = endPoint.getZ() - startPoint.getZ();
            double flightDuration = Math.sqrt(2.0 * (heightChange + jumpLength * Math.tan(landingAngle)) / gravityZ);
            double apexHeight = 0.5 * Math.pow(jumpLength * Math.tan(landingAngle), 2.0) / (flightDuration * flightDuration * gravityZ);

            apexPoint.interpolate(startPoint, endPoint, 0.5);
            apexPoint.setZ(apexHeight + nominalComHeight);

            double nominalInitialStiffness = 4.0 * Math.PI * Math.PI * mass / Math.pow(Math.min(firstTransferDuration, maxSupportForExtension), 2.0);
            double nominalFinalStiffness = 4.0 * Math.PI * Math.PI * mass / Math.pow(Math.min(secondTransferDuration, maxSupportForExtension), 2.0);

            ddpSolvers.get(number).initialize(currentCoMState, startPoint, apexPoint, endPoint, firstTransferDuration, flightDuration, secondTransferDuration,
                                  nominalInitialStiffness, nominalFinalStiffness);
         }
      }

      plotCoMPlan();

      while (true)
      {
         if (computeNextPass.getBooleanValue())
         {
            computeNextPass.set(false);

            for (int i = 0; i < updatesPerRequest.getIntegerValue(); i++)
            {
               for (int number = 0; number < numberOfJumps * numberOfHeights; number++)
                  ddpSolvers.get(number).singleSolve();
            }

            plotCoMPlan();
         }

         scs.tickAndUpdate();
         yoTime.add(dt);
      }
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private void plotCoMPlan()
   {
      for (int number = 0; number < numberOfJumps * numberOfHeights; number++)
      {
         DiscreteOptimizationData trajectory = ddpSolvers.get(number).getOptimalSequence();

         comTracks.get(number).reset();

         for (int i = 0; i < trajectory.size(); i++)
         {
            DenseMatrix64F control = trajectory.getControl(i);
            DenseMatrix64F state = trajectory.getState(i);

            tempPoint.set(control.get(SLIPState.xF), control.get(SLIPState.yF), 0.0);
            copTracks.get(number).setBallLoop(tempPoint);

            tempPoint.set(state.get(SLIPState.x), state.get(SLIPState.y), state.get(SLIPState.z));
            comTracks.get(number).setBallLoop(tempPoint);
         }
      }
   }



   private void updateUpcomingFootstepsViz(int footstepIndex, Footstep nextFootstep, Footstep nextNextFootstep)
   {
      double polygonShrinkAmount = 0.005;
      updateFootstepViz(nextFootstep, yoNextFootstepPolygons.get(footstepIndex), yoNextFootstepPoses.get(footstepIndex), polygonShrinkAmount);
      updateFootstepViz(nextNextFootstep, yoNextNextFootstepPolygons.get(footstepIndex), yoNextNextFootstepPoses.get(footstepIndex), polygonShrinkAmount);
   }

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void updateFootstepViz(Footstep footstep, YoFrameConvexPolygon2d convexPolygon2d, YoFramePose framePose, double polygonShrinkAmount)
   {
      footstep.setPredictedContactPoints(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d());
      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(footstep.getSoleReferenceFrame(), footstep.getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      convexPolygon2d.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(footstep.getSoleReferenceFrame());
      framePose.setAndMatchFrame(nextNextNextFootstepPose);

   }

   public static void main(String[] args)
   {
      new SLIPJumpingDDPCalculatorVisualizer();
   }
}
