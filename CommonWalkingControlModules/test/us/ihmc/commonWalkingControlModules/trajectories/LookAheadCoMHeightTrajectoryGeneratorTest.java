package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.thread.ThreadTools;

public class LookAheadCoMHeightTrajectoryGeneratorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<FramePose>(new FramePose(), new FramePose());

   private boolean makeAssertions = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testLookAheadCoMHeightTrajectoryGenerator()
   {
      //TODO: Make more assertions. Right now we just assert continuity, so this is more a human visualizer and manual tester than an automatic unit test...

      PoseReferenceFrame pelvisFrame = new PoseReferenceFrame("pelvisFrame", worldFrame);

      Random random = new Random(1776L);

      double minimumHeightAboveGround = 0.595 + 0.03;
      double nominalHeightAboveGround = 0.675 + 0.03;
      double maximumHeightAboveGround = 0.735 + 0.03;

      double doubleSupportPercentageIn = 0.3;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("LookAheadCoMHeightTrajectoryGeneratorTest");

      EnumYoVariable<RobotSide> supportLegFrameSide = new EnumYoVariable<RobotSide>("supportLegFrameSide", registry, RobotSide.class);

      SimulationTestingParameters testingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      testingParameters.setDataBufferSize(2048);

      Robot robot = new Robot("Dummy");
      DoubleYoVariable yoTime = robot.getYoTime();

      setupStuff(yoGraphicsListRegistry, registry);
      LookAheadCoMHeightTrajectoryGenerator lookAheadCoMHeightTrajectoryGenerator = new LookAheadCoMHeightTrajectoryGenerator(minimumHeightAboveGround, nominalHeightAboveGround,
            maximumHeightAboveGround, 0.0, doubleSupportPercentageIn, pelvisFrame, pelvisFrame, ankleZUpFrames, yoTime, yoGraphicsListRegistry, registry);

      lookAheadCoMHeightTrajectoryGenerator.setCoMHeightDriftCompensation(true);

      double dt = 0.01;

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, testingParameters);
      scs.setDT(dt, 2);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);

      CameraConfiguration cameraConfigurationOne = new CameraConfiguration("one");
      cameraConfigurationOne.setCameraTracking(false, false, false, false);
      cameraConfigurationOne.setCameraDolly(false, false, false, false);
      cameraConfigurationOne.setCameraFix(new Point3d(11.25, 11.8, 0.6));
      cameraConfigurationOne.setCameraPosition(new Point3d(10.1, 10.3, 0.3));
      scs.setupCamera(cameraConfigurationOne);

      CameraConfiguration cameraConfigurationTwo = new CameraConfiguration("two");
      cameraConfigurationTwo.setCameraTracking(false, false, false, false);
      cameraConfigurationTwo.setCameraDolly(false, false, false, false);
      cameraConfigurationTwo.setCameraFix(new Point3d(11.5, 14.25, 1.1));
      cameraConfigurationTwo.setCameraPosition(new Point3d(16.5, 10.9, 0.3));
      scs.setupCamera(cameraConfigurationTwo);

      ViewportConfiguration viewportConfiguration = new ViewportConfiguration("twoViews");
      viewportConfiguration.addCameraView("one", 0, 0, 1, 1);
      viewportConfiguration.addCameraView("two", 1, 0, 1, 1);

      scs.setupViewport(viewportConfiguration);

      scs.selectViewport("twoViews");
      scs.maximizeMainWindow();

      scs.startOnAThread();

      ArrayList<Updatable> updatables = new ArrayList<Updatable>();

      FootstepTestHelper footstepTestTools = new FootstepTestHelper(contactableFeet, ankleFrames);

      //    double stepWidth = 0.3;
      //    double stepLength = 0.75;
      //    int numberOfSteps = 30;
      //    FootstepProvider footstepProvider = footstepProviderTestHelper.createFootsteps(stepWidth, stepLength, numberOfSteps);

      //    FootstepProvider footstepProvider = footstepProviderTestHelper.createFlatGroundWalkingTrackFootstepProvider(registry, updatables);

      List<Footstep> footsteps = generateFootstepsForATestCase(footstepTestTools);

      boolean changeZRandomly = false;
      double maxZChange = 0.6;

      double time = 0.0;

      int stepNumber = 0;
      int maxNumberOfSteps = 80;

      Footstep transferFromFootstep = footsteps.remove(0);
      Footstep previousFootstep = transferFromFootstep;

      Footstep transferToFootstep;

      while ((footsteps.size() > 2) && (stepNumber < maxNumberOfSteps))
      {
         stepNumber++;

         transferFromFootstep = previousFootstep;
         transferToFootstep = footsteps.remove(0);

         if (changeZRandomly)
            transferToFootstep.setZ(transferToFootstep.getZ() + RandomTools.generateRandomDouble(random, 0.0, maxZChange));
         previousFootstep = transferToFootstep;

         Footstep upcomingFootstep = footsteps.get(0);

         FootSpoof transferFromFootSpoof = contactableFeet.get(transferFromFootstep.getRobotSide());
         FramePoint transferFromFootFramePoint = new FramePoint();
         transferFromFootstep.getPositionIncludingFrame(transferFromFootFramePoint);
         FrameOrientation transferFromFootOrientation = new FrameOrientation();
         transferFromFootstep.getOrientationIncludingFrame(transferFromFootOrientation);
         transferFromFootSpoof.setPose(transferFromFootFramePoint, transferFromFootOrientation);

         FootSpoof transferToFootSpoof = contactableFeet.get(transferToFootstep.getRobotSide());
         FramePoint transferToFootFramePoint = new FramePoint();
         transferToFootstep.getPositionIncludingFrame(transferToFootFramePoint);
         FrameOrientation transferToFootOrientation = new FrameOrientation();
         transferToFootstep.getOrientationIncludingFrame(transferToFootOrientation);
         transferToFootSpoof.setPose(transferToFootFramePoint, transferToFootOrientation);

         TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();
         transferToAndNextFootstepsData.setTransferFromFootstep(transferFromFootstep);
         transferToAndNextFootstepsData.setTransferToFootstep(transferToFootstep);
         transferToAndNextFootstepsData.setNextFootstep(upcomingFootstep);

         transferToAndNextFootstepsData.setTransferToSide(transferToFootstep.getRobotSide());

         RobotSide supportLeg = transferFromFootstep.getRobotSide();
         supportLegFrameSide.set(supportLeg);

         List<PlaneContactState> listOfContactStates = new ArrayList<PlaneContactState>();
         for (YoPlaneContactState contactState : contactStates)
         {
            listOfContactStates.add(contactState);
         }

         // Initialize at the beginning of single support.
         lookAheadCoMHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsData, 0.0);
         CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack = new CoMHeightPartialDerivativesData();

         scs.tickAndUpdate();

         double stepTime = 2.0;

         int numberOfTicks = (int) (stepTime / dt);
         for (int i = 0; i < numberOfTicks; i++)
         {
            if (i == 0.35 * numberOfTicks)
            {
               // Initialize again at the beginning of double support.
               lookAheadCoMHeightTrajectoryGenerator.initialize(transferToAndNextFootstepsData, 0.0);
               supportLeg = null;
            }
            else if (i == 0.65 * numberOfTicks)
            {
               supportLeg = transferToFootstep.getRobotSide();
            }

            time = time + dt;
            robot.setTime(time);

            for (Updatable updatable : updatables)
            {
               updatable.update(time);
            }

            FramePoint2d transferFromFootPosition = new FramePoint2d();
            FramePoint2d transferToFootPosition = new FramePoint2d();

            transferFromFootstep.getPosition2d(transferFromFootPosition);
            transferToFootstep.getPosition2d(transferToFootPosition);

            FramePoint2d queryPosition = new FramePoint2d();

            double alpha = ((double) i) / ((double) (numberOfTicks - 1));

            queryPosition.interpolate(transferFromFootPosition, transferToFootPosition, alpha);

            pelvisFrame.setX(queryPosition.getX());
            pelvisFrame.setY(queryPosition.getY());
            pelvisFrame.update();

            boolean switchSupportSides = RandomTools.generateRandomBoolean(random);
            if (switchSupportSides)
            {
               supportLegFrameSide.set(supportLegFrameSide.getEnumValue().getOppositeSide());
               lookAheadCoMHeightTrajectoryGenerator.setSupportLeg(supportLegFrameSide.getEnumValue());
            }

            boolean isInDoubleSupport = supportLeg == null;
            lookAheadCoMHeightTrajectoryGenerator.solve(coMHeightPartialDerivativesDataToPack, isInDoubleSupport);

            FramePoint comPosition = new FramePoint();
            pelvisFrame.setZ(comPosition.getZ());
            pelvisFrame.update();

            coMHeightPartialDerivativesDataToPack.getCoMHeight(comPosition);

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
         List<Point2d> contactPointsInSoleFrame = new ArrayList<Point2d>();
         double footLengthForControl = 0.3;
         double toeWidthForControl = 0.15;
         double footWidthForControl = 0.2;

         contactPointsInSoleFrame.add(new Point2d(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2d(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);

         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));

         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction,
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

   public List<Footstep> generateFootstepsForATestCase(FootstepTestHelper footstepProviderTestHelper)
   {
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3d(13.498008237591158, 12.933984676794712, 0.08304866902498514),
            new Quat4d(0.712207206856358, -0.002952488685311897, 0.003108390899721778, 0.7019562060545106)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3d(13.170202005363656, 12.938562106620529, 0.08304880466443637),
            new Quat4d(0.7118317189761082, -0.0030669544205584594, 0.002995905842662467, 0.7023369719716335)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3d(13.331337238526865, 13.209631212022698, 0.07946232652345472),
            new Quat4d(0.7118303580231159, -0.0031545801122483245, 0.018597667622730154, 0.7020980820227274)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3d(13.080837909205398, 13.505772370798573, 0.07489977483062672),
            new Quat4d(0.7118264756261261, -0.003742268262861186, 0.003495827367880344, 0.7023367021713666)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3d(13.370848128885667, 13.814155292179699, 0.3745783670265563),
            new Quat4d(0.7116005831473223, 0.01831580652938022, 0.024589083079517595, 0.7019148939072867)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3d(12.979449419903972, 14.098074875951077, 0.37701256522207105),
            new Quat4d(0.7118265348281387, -0.00374226855753469, 0.003495827052433593, 0.7023366421694281)));

      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3d(13.370848128885667, 13.814155292179699, 0.3745783670265563),
            new Quat4d(0.7116005831473223, 0.01831580652938022, 0.024589083079517595, 0.7019148939072867)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.LEFT, new Point3d(12.979449419903972, 14.098074875951077, 0.37701256522207105),
            new Quat4d(0.7118265348281387, -0.00374226855753469, 0.003495827052433593, 0.7023366421694281)));
      footsteps.add(footstepProviderTestHelper.createFootstep(RobotSide.RIGHT, new Point3d(13.370848128885667, 13.814155292179699, 0.3745783670265563),
            new Quat4d(0.7116005831473223, 0.01831580652938022, 0.024589083079517595, 0.7019148939072867)));

      return footsteps;
   }

}
