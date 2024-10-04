package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.trajectoryOptimization.slipJumping.SLIPJumpingDDPCalculator;
import us.ihmc.trajectoryOptimization.slipJumping.SLIPState;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class SLIPJumpingDDPCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.01;

   public static final double footWidthForControl = 0.11; //0.12; // 0.08;   //0.124887;
   public static final double toeWidthForControl = 0.085; //0.095; // 0.07;   //0.05;   //
   public static final double footLengthForControl = 0.22; //0.255;

   public static final Color defaultLeftColor = new Color(0f, 0.1f, 0.9f, 1f); //Blue
   public static final Color defaultRightColor = new Color(1.0f, 0.4f, 0.f, 1f); //Orange


   private static final double maxSupportForExtension = 0.3;
   private static final double firstTransferDuration = 0.3;
   private static final double secondTransferDuration = 0.3;
   private static final double landingAngle = Math.toRadians(20);

   private static final double nominalComHeight = 1.0;
   private static final double length = 1.5;
   private static final double gravityZ = 9.81;
   private static final double height = 0.1;

   private static final double mass = 150.0;

   private static final int numberOfJumps = 3;
   private static final int numberOfHeights = 2;

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final YoRegistry registry = new YoRegistry("ICPViz");

   private final List<YoFramePoseUsingYawPitchRoll> yoNextFootstepPoses = new ArrayList<>();
   private final List<YoFramePoseUsingYawPitchRoll> yoNextNextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoNextFootstepPolygons = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoNextNextFootstepPolygons = new ArrayList<>();

   private final YoBoolean computeNextPass = new YoBoolean("computeNextPass", registry);

   private final List<BagOfBalls> copTracks = new ArrayList<>();
   private final List<BagOfBalls> comTracks = new ArrayList<>();
   private final List<SLIPJumpingDDPCalculator> ddpSolvers = new ArrayList<>();

   private final List<FramePose3D> leftFoot = new ArrayList<>();
   private final List<FramePose3D> rightFoot = new ArrayList<>();

   private final YoInteger updatesPerRequest = new YoInteger("updatesPerRequest", registry);
   private final YoDouble trajectoryDT = new YoDouble("trajectoryDT", registry);

   public SLIPJumpingDDPCalculatorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      int simulatedTicksPerGraphicUpdate = 1;
      int numberOfBalls = (int) (3.0 / dt / simulatedTicksPerGraphicUpdate);

      for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
      {
         copTracks.add(new BagOfBalls(numberOfBalls,
                                      0.005,
                                      "CoP" + i,
                                      YoAppearance.Red(),
                                      YoGraphicPosition.GraphicType.BALL,
                                      registry,
                                      yoGraphicsListRegistry));
         comTracks.add(new BagOfBalls(numberOfBalls,
                                      0.005,
                                      "CoM" + i,
                                      YoAppearance.Black(),
                                      YoGraphicPosition.GraphicType.BALL,
                                      registry,
                                      yoGraphicsListRegistry));
         ddpSolvers.add(new SLIPJumpingDDPCalculator(dt, mass, nominalComHeight, gravityZ));

         yoNextFootstepPoses.add(new YoFramePoseUsingYawPitchRoll("nextFootstepPose" + i, worldFrame, registry));
         yoNextNextFootstepPoses.add(new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose" + i, worldFrame, registry));
         yoNextFootstepPolygons.add(new YoFrameConvexPolygon2D("nextFootstep" + i, "", worldFrame, 4, registry));
         yoNextNextFootstepPolygons.add(new YoFrameConvexPolygon2D("nextNextFootstep" + i, "", worldFrame, 4, registry));
      }

      footPolygon.addVertex(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
      footPolygon.addVertex(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
      footPolygon.addVertex(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
      footPolygon.addVertex(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
      footPolygon.update();

      YoFramePoseUsingYawPitchRoll leftCurrentFootPose = new YoFramePoseUsingYawPitchRoll("leftFootPose", worldFrame, registry);
      YoFramePoseUsingYawPitchRoll rightCurrentFootPose = new YoFramePoseUsingYawPitchRoll("rightFootPose", worldFrame, registry);

      Graphics3DObject leftFootGraphics = new Graphics3DObject();
      Graphics3DObject rightFootGraphics = new Graphics3DObject();
      leftFootGraphics.addExtrudedPolygon(footPolygon, 0.02, YoAppearance.Color(defaultLeftColor));
      rightFootGraphics.addExtrudedPolygon(footPolygon, 0.02, YoAppearance.Color(defaultRightColor));
      yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape("leftFootViz", leftFootGraphics, leftCurrentFootPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape("rightFootViz", rightFootGraphics, rightCurrentFootPose, 1.0));

      updatesPerRequest.set(10);
      trajectoryDT.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
               ddpSolvers.get(i).setDeltaT(trajectoryDT.getDoubleValue());
         }
      });
      trajectoryDT.set(dt);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      footstepGraphics.addExtrudedPolygon(footPolygon, 0.02, YoAppearance.Color(Color.blue));

      for (int i = 0; i < numberOfJumps * numberOfHeights; i++)
      {
         yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps",
                                                  new YoGraphicShape("nextFootstep" + i, footstepGraphics, yoNextFootstepPoses.get(i), 1.0));
         yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps",
                                                  new YoGraphicShape("nextNextFootstep" + i, footstepGraphics, yoNextNextFootstepPoses.get(i), 1.0));

         yoGraphicsListRegistry.registerArtifact("upcomingFootsteps",
                                                 new YoArtifactPolygon("nextFootstep" + i, yoNextFootstepPolygons.get(i), Color.blue, false));
         yoGraphicsListRegistry.registerArtifact("upcomingFootsteps",
                                                 new YoArtifactPolygon("nextNextFootstep" + i, yoNextNextFootstepPolygons.get(i), Color.blue, false));
      }

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoRegistry(registry);
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
         double lengthFraction = ((double) (numberOfJumps - i)) / ((double) numberOfJumps);
         for (int j = 0; j < numberOfHeights; j++)
         {
            double heightFraction = ((double) j) / ((double) numberOfHeights);
            leftFoot.add(new FramePose3D(new FramePoint3D(worldFrame, lengthFraction * length, 0.1 - 0.5 * (numberOfHeights * i + j), heightFraction * height),
                                         new FrameQuaternion()));
            rightFoot.add(new FramePose3D(new FramePoint3D(worldFrame,
                                                           lengthFraction * length,
                                                           -0.1 - 0.5 * (numberOfHeights * i + j),
                                                           heightFraction * height), new FrameQuaternion()));
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

            DMatrixRMaj currentCoMState;
            currentCoMState = new DMatrixRMaj(SLIPState.stateVectorSize, 1);
            currentCoMState.set(SLIPState.y, 0, -0.5 * number);
            currentCoMState.set(SLIPState.z, 0, 1.0);

            double jumpLength = startPoint.distance(endPoint);
            double heightChange = endPoint.getZ() - startPoint.getZ();
            double flightDuration = Math.sqrt(2.0 * (heightChange + jumpLength * Math.tan(landingAngle)) / gravityZ);
            double apexHeight = 0.5 * Math.pow(jumpLength * Math.tan(landingAngle), 2.0) / (flightDuration * flightDuration * gravityZ);
            apexHeight += heightChange;

            apexPoint.interpolate(startPoint, endPoint, 0.5);
            apexPoint.setZ(apexHeight + nominalComHeight);

            double nominalInitialStiffness = 4.0 * Math.PI * Math.PI * mass / Math.pow(Math.min(firstTransferDuration, maxSupportForExtension), 2.0);
            double nominalFinalStiffness = 4.0 * Math.PI * Math.PI * mass / Math.pow(Math.min(secondTransferDuration, maxSupportForExtension), 2.0);

            ddpSolvers.get(number)
                      .initialize(currentCoMState,
                                  startPoint,
                                  apexPoint,
                                  endPoint,
                                  firstTransferDuration,
                                  flightDuration,
                                  secondTransferDuration,
                                  nominalInitialStiffness,
                                  nominalFinalStiffness);
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
            DMatrixRMaj control = trajectory.getControl(i);
            DMatrixRMaj state = trajectory.getState(i);

            tempPoint.set(control.get(SLIPState.xF), control.get(SLIPState.yF), 0.0);
            copTracks.get(number).setBallLoop(tempPoint);

            tempPoint.set(state.get(SLIPState.x), state.get(SLIPState.y), state.get(SLIPState.z));
            comTracks.get(number).setBallLoop(tempPoint);
         }
      }
   }

   private void updateUpcomingFootstepsViz(int footstepIndex, FramePose3DReadOnly nextFootstep, FramePose3DReadOnly nextNextFootstep)
   {
      yoNextFootstepPoses.get(footstepIndex).set(nextFootstep);
      yoNextNextFootstepPoses.get(footstepIndex).set(nextNextFootstep);
      yoNextFootstepPolygons.get(footstepIndex).set(footPolygon);
      yoNextNextFootstepPolygons.get(footstepIndex).set(footPolygon);
   }

   public static void main(String[] args)
   {
      new SLIPJumpingDDPCalculatorVisualizer();
   }
}
