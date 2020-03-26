package us.ihmc.commonWalkingControlModules.controlModules.pelvis;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.BipedCoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SimpleCoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightPartialDerivativesData;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.dataStructures.Vertex2DSupplierList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.ArrayTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.fail;

public class CenterOfMassHeightControlStateTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());

   private boolean makeAssertions = true;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testWalkingUpRamp()
   {
      //TODO: Make more assertions. Right now we just assert continuity, so this is more a human visualizer and manual tester than an automatic unit test...

      RigidBody elevator = new RigidBody("elevator", worldFrame);

      SixDoFJoint floatingJoint = new SixDoFJoint("sixDofJoint", elevator);
      RigidBody pelvis = new RigidBody("pelvis", floatingJoint, 0.1, 0.1, 0.1, 5.0, new Vector3D());
      MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      Random random = new Random(1776L);

      double minimumHeightAboveGround = 0.595 + 0.03;
      double nominalHeightAboveGround = 0.675 + 0.03;
      double maximumHeightAboveGround = 0.735 + 0.03;

      double doubleSupportPercentageIn = 0.3;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("LookAheadCoMHeightTrajectoryGeneratorTest");

      YoEnum<RobotSide> supportLegFrameSide = new YoEnum<RobotSide>("supportLegFrameSide", registry, RobotSide.class);

      SimulationTestingParameters testingParameters = SimulationTestingParameters.createFromSystemProperties();
      testingParameters.setKeepSCSUp(true);
      testingParameters.setDataBufferSize(2048);

      double dt = 0.01;

      Robot robot = new Robot("Dummy");
      YoDouble yoTime = robot.getYoTime();
      SideDependentList<RigidBodyTransform> anklePositionsInSoleFrame = new SideDependentList<>(new RigidBodyTransform(), new RigidBodyTransform());

      setupStuff(yoGraphicsListRegistry, registry);

      PDGains gains = new PDGains();

      CenterOfMassHeightControlState heightControlState = new CenterOfMassHeightControlState(minimumHeightAboveGround,
                                                                                             nominalHeightAboveGround,
                                                                                             maximumHeightAboveGround,
                                                                                             0.0,
                                                                                             elevator,
                                                                                             anklePositionsInSoleFrame,
                                                                                             pelvisFrame,
                                                                                             pelvisFrame,
                                                                                             ankleZUpFrames,
                                                                                             9.81,
                                                                                             yoTime,
                                                                                             dt,
                                                                                             registry,
                                                                                             yoGraphicsListRegistry);

      heightControlState.setMinimumHeightAboveGround(0.705);
      heightControlState.setNominalHeightAboveGround(0.7849999999999999);
      heightControlState.setMaximumHeightAboveGround(0.9249999999999999);
      heightControlState.setCoMHeightDriftCompensation(false);
      heightControlState.setGains(gains, () -> 10.0);

      double omega0 = 3.0;

      SideDependentList<FrameConvexPolygon2DReadOnly> feetPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         feetPolygons.put(robotSide, new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFeet.get(robotSide).getContactPoints2d())));
      }
      ICPPlannerParameters parameters = new SmoothCMPPlannerParameters(1.0);
      SmoothCMPBasedICPPlanner smoothCMPBasedICPPlanner = new SmoothCMPBasedICPPlanner(10.0, feetPolygons, soleFrames, contactableFeet, null, yoTime,
                                                                                       registry, yoGraphicsListRegistry, 9.81, parameters);
      smoothCMPBasedICPPlanner.setOmega0(omega0);
      smoothCMPBasedICPPlanner.setFinalTransferDuration(1.0);
      smoothCMPBasedICPPlanner.holdCurrentICP(new FramePoint3D(worldFrame, 0, 0, 0.785));
//      smoothCMPBasedICPPlanner.initializeForStanding(0.0);

      YoFramePoint3D desiredICPPosition = new YoFramePoint3D("desiredICPPosition", worldFrame, registry);
      YoFrameVector3D desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);
      YoGraphicPosition desiredICPGraphic = new YoGraphicPosition("desiredICPGraphic", desiredICPPosition, 0.05, YoAppearance.Black(), GraphicType.SOLID_BALL);

      yoGraphicsListRegistry.registerYoGraphic("Test", desiredICPGraphic);

      yoGraphicsListRegistry.registerArtifact("Test", desiredICPGraphic.createArtifact());

      FrameVector2D dummyDesiredICPVelocity = new FrameVector2D();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, testingParameters);
      scs.setDT(dt, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      CameraConfiguration cameraConfigurationOne = new CameraConfiguration("one");
      cameraConfigurationOne.setCameraTracking(false, false, false, false);
      cameraConfigurationOne.setCameraDolly(false, false, false, false);
      cameraConfigurationOne.setCameraFix(new Point3D(11.25, 11.8, 0.6));
      cameraConfigurationOne.setCameraPosition(new Point3D(10.1, 10.3, 0.3));
      scs.setupCamera(cameraConfigurationOne);

      scs.maximizeMainWindow();

      scs.startOnAThread();

      FootstepTestHelper footstepTestTools = new FootstepTestHelper(contactableFeet);

      List<Footstep> footsteps = generateFootstepsForRamp(footstepTestTools);

      double time = 0.0;
      double dsRatio = 0.6;
      double stepTime = 2.0;


      Footstep transferFromFootstep = footsteps.remove(0);
      Footstep previousFootstep = transferFromFootstep;

      Footstep transferToFootstep;

      while (footsteps.size() > 2)
      {

         transferFromFootstep = previousFootstep;
         smoothCMPBasedICPPlanner.clearPlan();
         for (int i = 0; i < Math.min(3, footsteps.size()); i++)
         {
            FootstepTiming timing = new FootstepTiming();
            timing.setTimings((1.0 - dsRatio) * stepTime, dsRatio * stepTime);
            FootstepShiftFractions fractions = new FootstepShiftFractions();
            smoothCMPBasedICPPlanner.addFootstepToPlan(footsteps.get(i), timing, fractions);
         }
         transferToFootstep = footsteps.remove(0);

         previousFootstep = transferToFootstep;

         FootSpoof transferFromFootSpoof = contactableFeet.get(transferFromFootstep.getRobotSide());
         FramePoint3D transferFromFootFramePoint = new FramePoint3D();
         transferFromFootstep.getPosition(transferFromFootFramePoint);
         FrameQuaternion transferFromFootOrientation = new FrameQuaternion();
         transferFromFootstep.getOrientation(transferFromFootOrientation);
         transferFromFootSpoof.setPose(transferFromFootFramePoint, transferFromFootOrientation);

         FootSpoof transferToFootSpoof = contactableFeet.get(transferToFootstep.getRobotSide());
         FramePoint3D transferToFootFramePoint = new FramePoint3D();
         transferToFootstep.getPosition(transferToFootFramePoint);
         FrameQuaternion transferToFootOrientation = new FrameQuaternion();
         transferToFootstep.getOrientation(transferToFootOrientation);
         transferToFootSpoof.setPose(transferToFootFramePoint, transferToFootOrientation);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
         transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
         transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
         transferToAndNextFootstepsData.setTransferFromDesiredFootstep(null);
         transferToAndNextFootstepsData.setNextFootstep(null);

         transferToAndNextFootstepsData.setTransferToSide(transferToFootstep.getRobotSide());

         RobotSide supportLeg = transferFromFootstep.getRobotSide();
         supportLegFrameSide.set(supportLeg);

         // Initialize at the beginning of single support.
         heightControlState.initialize(transferToAndNextFootstepsData, 0.0);

         scs.tickAndUpdate();

         smoothCMPBasedICPPlanner.initializeForTransfer(time);

         int numberOfTicks = (int) (stepTime / dt);
         for (int i = 0; i < numberOfTicks; i++)
         {

            if (i == 0.35 * numberOfTicks)
            {
               // Initialize again at the beginning of double support.
               heightControlState.initialize(transferToAndNextFootstepsData, 0.0);
               supportLeg = null;
            }
            else if (i == 0.65 * numberOfTicks)
            {
               smoothCMPBasedICPPlanner.initializeForSingleSupport(time);
               supportLeg = transferToFootstep.getRobotSide();
            }

            time += dt;
            robot.setTime(time);

            FramePoint3DReadOnly transferFromFootPosition = transferFromFootstep.getFootstepPose().getPosition();
            FramePoint3DReadOnly transferToFootPosition = transferToFootstep.getFootstepPose().getPosition();

            FramePoint3D queryPosition = new FramePoint3D();

            double alpha = ((double) i) / ((double) (numberOfTicks - 1));

            queryPosition.interpolate(transferFromFootPosition, transferToFootPosition, alpha);

            floatingJoint.getJointPose().getPosition().setX(queryPosition.getX());
            floatingJoint.getJointPose().getPosition().setY(queryPosition.getY());
            floatingJoint.updateFramesRecursively();

            boolean switchSupportSides = random.nextBoolean();
            if (switchSupportSides)
            {
               supportLegFrameSide.set(supportLegFrameSide.getEnumValue().getOppositeSide());
               heightControlState.setSupportLeg(supportLegFrameSide.getEnumValue());
            }

            boolean isInDoubleSupport = supportLeg == null;

            smoothCMPBasedICPPlanner.compute(time);
            // FIXME should be CoM
            smoothCMPBasedICPPlanner.getDesiredCapturePointPosition(desiredICPPosition);
            smoothCMPBasedICPPlanner.getDesiredCapturePointVelocity(desiredICPVelocity);
            dummyDesiredICPVelocity.set(desiredICPVelocity);
            heightControlState.computeDesiredCoMHeightAcceleration(dummyDesiredICPVelocity, isInDoubleSupport, omega0, false, null);

            FramePoint3D comPosition = new FramePoint3D();
            floatingJoint.getJointPose().getPosition().setZ(comPosition.getZ());
            floatingJoint.updateFramesRecursively();

            scs.tickAndUpdate();
         }
      }

      scs.gotoInPointNow();
      scs.tick(2);
      scs.setInPoint();
      scs.cropBuffer();

      double[] desiredCoMPositionXData = scs.getDataBuffer().getEntry(scs.getVariable("desiredCoMPositionX")).getData();
      double[] desiredCoMPositionYData = scs.getDataBuffer().getEntry(scs.getVariable("desiredCoMPositionY")).getData();
      double[] desiredCoMPositionZData = scs.getDataBuffer().getEntry(scs.getVariable("desiredCoMPositionZ")).getData();

      double maxChangePerTick = dt * 0.75;
      boolean isDesiredCoMPositionXContinuous = ArrayTools.isContinuous(desiredCoMPositionXData, maxChangePerTick);
      boolean isDesiredCoMPositionYContinuous = ArrayTools.isContinuous(desiredCoMPositionYData, maxChangePerTick);
      boolean isDesiredCoMPositionZContinuous = ArrayTools.isContinuous(desiredCoMPositionZData, maxChangePerTick);

      if (!isDesiredCoMPositionXContinuous || !isDesiredCoMPositionYContinuous || !isDesiredCoMPositionZContinuous)
      {
         double xMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredCoMPositionXData);
         double yMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredCoMPositionYData);
         double zMaxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(desiredCoMPositionZData);

         System.err.println("desiredCoMPositionXData xMaxChange = " + xMaxChange);
         System.err.println("desiredCoMPositionYData yMaxChange = " + yMaxChange);
         System.err.println("desiredCoMPositionZData yMaxChange = " + zMaxChange);

         System.err.println("maxChangePerTick = " + maxChangePerTick);

         if (makeAssertions)
            fail("Desired CoM position is not continuous!");
      }

      if (testingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
   }

   private void setupStuff(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
         double footLengthForControl = 0.3;
         double toeWidthForControl = 0.15;
         double footWidthForControl = 0.2;

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
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot",
                                                                           foot,
                                                                           soleFrame,
                                                                           contactFramePoints,
                                                                           coefficientOfFriction,
                                                                           registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }
   }

   public List<Footstep> generateFootstepsForRamp(FootstepTestHelper footstepProviderTestHelper)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT,
                                                              new Point3D(-0.007, -0.164, 0.001),
                                                              new Quaternion(-0.000, 0.004, -0.000, 1.000)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3D(-0.007, 0.164, 0.001), new Quaternion(0.000, 0.004, 0.000, 1.000)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3D(0.593, -0.086, 0.009), new Quaternion(0.000, 0.000, 0.000, 1.000)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3D(1.190, 0.169, 0.069), new Quaternion(-0.009, -0.052, 0.011, 0.999)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT,
                                                              new Point3D(1.772, -0.072, 0.127),
                                                              new Quaternion(0.009, -0.051, -0.004, 0.999)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3D(2.378, 0.177, 0.188), new Quaternion(-0.010, -0.052, 0.006, 0.999)));

      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT,
                                                              new Point3D(2.990, -0.068, 0.240),
                                                              new Quaternion(0.010, -0.050, -0.006, 0.999)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3D(3.596, 0.174, 0.300), new Quaternion(-0.015, -0.052, -0.003, 0.999)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT,
                                                              new Point3D(4.181, -0.078, 0.354),
                                                              new Quaternion(0.013, -0.050, -0.015, 0.999)));

      return footsteps;
   }
}
