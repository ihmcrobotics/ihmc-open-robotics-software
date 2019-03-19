package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.AngularMomentumEstimationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MutationTestFacilitator;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootstepAngularMomentumPredictorTest
{
   // Physical parameters used for testing
   private final double omega0 = 3.4; // s^-1
   private final double stepLength = 0.5; // m
   private final double stepWidth = 0.25; // m
   private final double swingTime = 1.0; // s
   private final double transferTime = 0.2; //s
   private final double footLength = 0.12;
   private final double footwidth = 0.065;
   private final double robotMass = 150;
   private final double gravityZ = 9.81;
   private final double comHeight = gravityZ / (omega0 * omega0);
   private final String testName = "TesAngularMomentum";

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SmoothCMPPlannerParameters testParameters = new SmoothCMPPlannerParameters()
   {
      @Override
      public boolean planSwingAngularMomentum()
      {
         return true;
      }

      @Override
      public boolean planTransferAngularMomentum()
      {
         return true;
      }
   };
   AngularMomentumEstimationParameters angularMomentumEstimationParameters = testParameters.getAngularMomentumEstimationParameters();
   private final double swingLegMass = angularMomentumEstimationParameters.getPercentageSwingLegMass() * robotMass;
   private final double supportLegMass = angularMomentumEstimationParameters.getPercentageSupportLegMass() * robotMass;

   // Variables for testing
   private final YoVariableRegistry testRegistry = new YoVariableRegistry("AngularMomentumTestRegistry");
   private final YoDouble omega = new YoDouble("AngularMomentumTestOmega", testRegistry); // Taking the for Atlas
   private final YoInteger numberOfUpcomingFootsteps = new YoInteger("NumberOfUpcomingFootsteps", testRegistry);
   private final FramePoint3D currentLocation = new FramePoint3D();
   private final FrameVector3D walkingDirectionUnitVector = new FrameVector3D();
   private final FrameQuaternion robotOrientation = new FrameQuaternion();
   private final QuaternionBasedTransform rightTransform = new QuaternionBasedTransform(new Quaternion(0.0, 0.0, Math.sin(-Math.PI / 4.0),
                                                                                                       Math.cos(-Math.PI / 4.0)),
                                                                                        new FramePoint3D());
   private final QuaternionBasedTransform leftTransform = new QuaternionBasedTransform(new Quaternion(0.0, 0.0, Math.sin(Math.PI / 4.0),
                                                                                                      Math.cos(Math.PI / 4.0)),
                                                                                       new FrameVector3D());
   private final List<YoDouble> swingDurations = new ArrayList<YoDouble>();
   private final List<YoDouble> transferDurations = new ArrayList<YoDouble>();
   private final List<YoDouble> touchdownDurations = new ArrayList<YoDouble>();
   private final List<YoDouble> swingSplitFractions = new ArrayList<YoDouble>();
   private final List<YoDouble> transferSplitFractions = new ArrayList<YoDouble>();
   private final List<YoDouble> swingShiftFractions = new ArrayList<YoDouble>();
   private final List<FootstepData> upcomingFootstepsData = new ArrayList<>();

   // Some variables for setters, getters and intermediate computations
   private final FramePoint3D tempFramePoint1 = new FramePoint3D(), tempFramePoint2 = new FramePoint3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   private FootstepAngularMomentumPredictor angularMomentumGenerator;
   private ReferenceCoPTrajectoryGenerator copTrajectoryGenerator;
   private ReferenceICPTrajectoryGenerator icpTrajectoryGenerator;
   private ReferenceCoMTrajectoryGenerator comTrajectoryGenerator;
   private SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private ReferenceFrame midFeetFrame;
   private YoInteger numberOfFootstepsToConsider;
   private YoBoolean isInitialTransfer, isStanding, isDoubleSupport;
   private FrameTrajectory3D comPositionTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D swingFootPositionTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D stanceFootPositionTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D swingFootRelativePositionTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D stanceFootRelativePositionTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D swingFootRelativeVelocityTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D stanceFootRelativeVelocityTrajectory = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D swingFootAngularMomentum = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D stanceFootAngularMomentum = new FrameTrajectory3D(10, worldFrame);
   private FrameTrajectory3D angularMomentumTrajectory = new FrameTrajectory3D(10, worldFrame);
   private TrajectoryMathTools trajectoryMathTools = new TrajectoryMathTools(14);
   private FrameVector3D zeroVector = new FrameVector3D(worldFrame);

   @BeforeEach
   public void setupTest()
   {
      omega.set(omega0);
      List<Point2D> contactPoints = new ArrayList<>(4);
      contactPoints.add(new Point2D(footLength / 2.0, footwidth / 2.0));
      contactPoints.add(new Point2D(footLength / 2.0, -footwidth / 2.0));
      contactPoints.add(new Point2D(-footLength / 2.0, -footwidth / 2.0));
      contactPoints.add(new Point2D(-footLength / 2.0, footwidth / 2.0));

      currentLocation.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0); // Assuming ground location to always be at zero height

      double worldFrameAngle = 0.0;
      robotOrientation.set(worldFrame, new Quaternion(0.0, 0.0, Math.sin(worldFrameAngle / 2.0), Math.cos(worldFrameAngle / 2.0)));
      walkingDirectionUnitVector.setIncludingFrame(worldFrame, Math.cos(worldFrameAngle), Math.sin(worldFrameAngle), 0.0); // Robot walks in the direction its facing

      for (RobotSide side : RobotSide.values)
      {
         FootSpoof foot = new FootSpoof(testName + side.getCamelCaseName() + "Foot", 0.0, 0.0, 0.084, contactPoints, 0.1);
         getFootLocationFromCoMLocation(tempFramePoint1, side, currentLocation, walkingDirectionUnitVector, stepLength, stepWidth);
         foot.setPose(tempFramePoint1, robotOrientation);
         contactableFeet.put(side, foot);
         soleFrames.put(side, foot.getSoleFrame());
         ankleFrames.put(side, foot.getFrameAfterParentJoint());
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(side.getCamelCaseName(), foot.getRigidBody(), foot.getSoleFrame(),
                                                                           foot.getContactPoints2d(), foot.getCoefficientOfFriction(), testRegistry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(side, yoPlaneContactState);
      }
      for (int i = 0; i < testParameters.getNumberOfFootstepsToConsider(); i++)
      {
         YoDouble tempYoDouble = new YoDouble(testName + "Step" + i + "SwingDuration", testRegistry);
         tempYoDouble.set(swingTime);
         swingDurations.add(tempYoDouble);
         tempYoDouble = new YoDouble(testName + "Step" + i + "TransferDuration", testRegistry);
         tempYoDouble.set(transferTime);
         transferDurations.add(tempYoDouble);
         tempYoDouble = new YoDouble(testName + "Step" + i + "touchdownDuration", testRegistry);
         tempYoDouble.set(0.0);
         touchdownDurations.add(tempYoDouble);
         tempYoDouble = new YoDouble(testName + "Step" + i + "SwingSplitFraction", testRegistry);
         tempYoDouble.set(testParameters.getSwingSplitFraction());
         swingSplitFractions.add(tempYoDouble);
         tempYoDouble = new YoDouble(testName + "Step" + i + "SwingShiftFraction", testRegistry);
         tempYoDouble.set(testParameters.getSwingDurationShiftFraction());
         swingShiftFractions.add(tempYoDouble);
         tempYoDouble = new YoDouble(testName + "Step" + i + "TransferSplit", testRegistry);
         tempYoDouble.set(testParameters.getTransferSplitFraction());
         transferSplitFractions.add(tempYoDouble);
      }
      YoDouble tempYoDouble = new YoDouble(testName + "Step" + testParameters.getNumberOfFootstepsToConsider() + "TransferDuration", testRegistry);
      tempYoDouble.set(transferTime);
      transferDurations.add(tempYoDouble);
      tempYoDouble = new YoDouble(testName + "Step" + testParameters.getNumberOfFootstepsToConsider() + "TransferSplit", testRegistry);
      tempYoDouble.set(testParameters.getTransferSplitFraction());
      transferSplitFractions.add(tempYoDouble);
      isStanding = new YoBoolean(testName + "IsStanding", testRegistry);
      isDoubleSupport = new YoBoolean(testName + "IsDoubleSupport", testRegistry);
      isInitialTransfer = new YoBoolean(testName + "IsInitialTransfer", testRegistry);

      numberOfFootstepsToConsider = new YoInteger(testName + "NumberOfFootstepsToConsider", testRegistry);
      numberOfFootstepsToConsider.set(testParameters.getNumberOfFootstepsToConsider());
      midFeetFrame = new MidFootZUpGroundFrame(testName + "MidFeetFrame", soleFrames.get(RobotSide.RIGHT), soleFrames.get(RobotSide.LEFT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons( midFeetFrame, soleFrames, soleFrames, testRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      copTrajectoryGenerator = new ReferenceCoPTrajectoryGenerator(testName + "CoPGenerator", testParameters.getNumberOfFootstepsToConsider(),
                                                                   bipedSupportPolygons, contactableFeet, numberOfFootstepsToConsider, swingDurations,
                                                                   transferDurations, touchdownDurations, swingSplitFractions, swingShiftFractions,
                                                                   transferSplitFractions, numberOfUpcomingFootsteps, upcomingFootstepsData, soleFrames,
                                                                   testRegistry);
      icpTrajectoryGenerator = new ReferenceICPTrajectoryGenerator(testName, omega, numberOfFootstepsToConsider, isInitialTransfer, isStanding, false, testRegistry, null);
      comTrajectoryGenerator = new ReferenceCoMTrajectoryGenerator(testName, omega, numberOfFootstepsToConsider, isInitialTransfer, isDoubleSupport,
                                                                   testRegistry);
      int maxNumberOfStepsToConsider = 4;
      angularMomentumGenerator = new FootstepAngularMomentumPredictor(testName, omega, true, maxNumberOfStepsToConsider, testRegistry);
      copTrajectoryGenerator.initializeParameters(testParameters);
      angularMomentumGenerator.initializeParameters(testParameters, robotMass, gravityZ);
   }

   private void getFootLocationFromCoMLocation(FramePoint3D footLocationToPack, RobotSide side, FramePoint3D comLocation,
                                               FrameVector3D walkingDirectionUnitVector, double stepLength, double stepWidth)
   {
      FrameVector3D frameVectorForTempResults = new FrameVector3D();
      footLocationToPack.setIncludingFrame(comLocation);
      frameVectorForTempResults.setIncludingFrame(walkingDirectionUnitVector);
      frameVectorForTempResults.scale(stepLength / 2.0);
      footLocationToPack.add(frameVectorForTempResults);

      frameVectorForTempResults.setIncludingFrame(walkingDirectionUnitVector);
      if (side == RobotSide.LEFT)
         frameVectorForTempResults.applyTransform(leftTransform);
      else
         frameVectorForTempResults.applyTransform(rightTransform);
      frameVectorForTempResults.scale(stepWidth / 2.0);

      footLocationToPack.add(frameVectorForTempResults);
   }

   private void clear()
   {
      testRegistry.clear();
      copTrajectoryGenerator.clear();
      icpTrajectoryGenerator.reset();
      comTrajectoryGenerator.reset();
      angularMomentumGenerator.clear();
      upcomingFootstepsData.clear();
      numberOfUpcomingFootsteps.set(0);
   }

   @AfterEach
   public void tearDownTest()
   {
      clear();
   }

   @Test
   public void testAngularMomentumInitialTransfer()
   {
      setupInputs();
      isInitialTransfer.set(true);
      isStanding.set(true);
      isDoubleSupport.set(true);
      copTrajectoryGenerator.computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), RobotSide.LEFT, null);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      icpTrajectoryGenerator.initializeForTransferFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(),
                                                           copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.computeTrajectoryStartingFromTransfer(copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                                   icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());

      List<? extends FramePoint3DReadOnly> comInitialPositionList = comTrajectoryGenerator.getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> comFinalPositionList = comTrajectoryGenerator.getCoMPositionDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredFinalList();
      List<CoPPointsInFoot> copWaypointList = copTrajectoryGenerator.getWaypoints();

      angularMomentumGenerator.addCopAndComSetpointsToPlan(copTrajectoryGenerator.getWaypoints(), comInitialPositionList, comFinalPositionList,
                                                           comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList,
                                                           copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), isStanding.getValue());

      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();
      checkAngularMomentumTrajectories(0, 0, comInitialPositionList, comFinalPositionList, comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList,
                                       comFinalAccelerationList, copWaypointList, swingAngularMomentumTrajectories, transferAngularMomentumTrajectories);
   }

   private void computeDoubleSupportFootTrajectories(List<CoPPointsInFoot> copWaypointList, int stepIndex)
   {
      copWaypointList.get(stepIndex).getSupportFootLocation(tempFramePoint1);
      swingFootPositionTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, transferTime, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
      copWaypointList.get(stepIndex + 1).getSupportFootLocation(tempFramePoint1);
      stanceFootPositionTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, transferTime, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
   }

   private void computeSingleSupportFootTrajectories(List<CoPPointsInFoot> copWaypointList, int stepIndex)
   {
      copWaypointList.get(stepIndex).getSupportFootLocation(tempFramePoint1);
      copWaypointList.get(stepIndex + 1).getSwingFootLocation(tempFramePoint2);
      swingFootPositionTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, swingTime, tempFramePoint1, zeroVector, tempFramePoint2, zeroVector);
      copWaypointList.get(stepIndex + 1).getSupportFootLocation(tempFramePoint1);
      stanceFootPositionTrajectory.setQuinticWithZeroTerminalAcceleration(0.0, swingTime, tempFramePoint1, zeroVector, tempFramePoint1, zeroVector);
   }

   private void computeAngularMomentumTrajectory()
   {
      TrajectoryMathTools.subtractByTrimming(swingFootRelativePositionTrajectory, swingFootPositionTrajectory, comPositionTrajectory);
      TrajectoryMathTools.getDerivative(swingFootRelativeVelocityTrajectory, swingFootRelativePositionTrajectory);
      TrajectoryMathTools.subtractByTrimming(stanceFootRelativePositionTrajectory, stanceFootPositionTrajectory, comPositionTrajectory);
      TrajectoryMathTools.getDerivative(stanceFootRelativeVelocityTrajectory, stanceFootRelativePositionTrajectory);
      trajectoryMathTools.crossProduct(swingFootAngularMomentum, swingFootRelativePositionTrajectory, swingFootRelativeVelocityTrajectory);
      trajectoryMathTools.crossProduct(stanceFootAngularMomentum, stanceFootRelativePositionTrajectory, stanceFootRelativeVelocityTrajectory);
      TrajectoryMathTools.scale(swingLegMass, swingFootAngularMomentum);
      TrajectoryMathTools.scale(supportLegMass, stanceFootAngularMomentum);
      TrajectoryMathTools.add(angularMomentumTrajectory, swingFootAngularMomentum, stanceFootAngularMomentum);
   }

   private int setCoMTrajectory(double initialTime, double finalTime,
                                List<? extends FramePoint3DReadOnly> comInitialPositionList, List<? extends FramePoint3DReadOnly> comFinalPositionList,
                                List<? extends FrameVector3DReadOnly> comInitialVelocityList, List<? extends FrameVector3DReadOnly> comFinalVelocityList,
                                List<? extends FrameVector3DReadOnly> comInitialAccelerationList, List<? extends FrameVector3DReadOnly> comFinalAccelerationList,
                                int comListCounter)
   {
      tempFramePoint1.setIncludingFrame(comInitialPositionList.get(comListCounter));
      tempFramePoint1.addZ(comHeight);
      tempFramePoint2.setIncludingFrame(comFinalPositionList.get(comListCounter));
      tempFramePoint2.addZ(comHeight);
      comPositionTrajectory.setQuintic(initialTime, finalTime, tempFramePoint1, comInitialVelocityList.get(comListCounter),
                                      comInitialAccelerationList.get(comListCounter), tempFramePoint2, comFinalVelocityList.get(comListCounter),
                                      comFinalAccelerationList.get(comListCounter));
      return comListCounter + 1;
   }

   @Test
   public void testAngularMomentumNormalTransfer()
   {
      setupInputs();
      isInitialTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(true);
      copTrajectoryGenerator.computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), RobotSide.LEFT, null);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      icpTrajectoryGenerator.initializeForTransferFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(),
                                                           copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.computeTrajectoryStartingFromTransfer(copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                                   icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());
      List<? extends FramePoint3DReadOnly> comInitialPositionList = comTrajectoryGenerator.getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> comFinalPositionList = comTrajectoryGenerator.getCoMPositionDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredFinalList();
      List<CoPPointsInFoot> copWaypointList = copTrajectoryGenerator.getWaypoints();

      angularMomentumGenerator.addCopAndComSetpointsToPlan(copTrajectoryGenerator.getWaypoints(), comInitialPositionList, comFinalPositionList,
                                                           comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList,
                                                           copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), isStanding.getValue());
      angularMomentumGenerator.initializeForDoubleSupport(0.0, isStanding.getBooleanValue());

      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();
      checkAngularMomentumTrajectories(0, 0, comInitialPositionList, comFinalPositionList, comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList,
                                       comFinalAccelerationList, copWaypointList, swingAngularMomentumTrajectories, transferAngularMomentumTrajectories);
   }

   private void checkAngularMomentumTrajectories(int comListCounter, int stepIndex, List<? extends FramePoint3DReadOnly> comInitialPositionList,
                                                 List<? extends FramePoint3DReadOnly> comFinalPositionList,
                                                 List<? extends FrameVector3DReadOnly> comInitialVelocityList,
                                                 List<? extends FrameVector3DReadOnly> comFinalVelocityList,
                                                 List<? extends FrameVector3DReadOnly> comInitialAccelerationList,
                                                 List<? extends FrameVector3DReadOnly> comFinalAccelerationList,
                                                 List<CoPPointsInFoot> copWaypointList,
                                                 List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories,
                                                 List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories)
   {
      for (; stepIndex < numberOfFootstepsToConsider.getIntegerValue(); stepIndex++)
      {
         comListCounter = checkTransferAngularMomentumTrajectories(comListCounter, stepIndex, comInitialPositionList, comFinalPositionList,
                                                                   comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList,
                                                                   comFinalAccelerationList, copWaypointList, transferAngularMomentumTrajectories);

         comListCounter = checkSwingAngularMomentumTrajectories(comListCounter, stepIndex, comInitialPositionList, comFinalPositionList, comInitialVelocityList,
                                                                comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, copWaypointList,
                                                                swingAngularMomentumTrajectories);
      }
      AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(numberOfFootstepsToConsider.getIntegerValue());
      assertTrue("Transfer trajectory " + numberOfFootstepsToConsider.getIntegerValue() + " has " + transferAngularMomentumTrajectory.getNumberOfSegments()
            + " segments, needed 1.\n" + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 1);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
      comListCounter = setCoMTrajectory(0.0, transferTime * testParameters.getTransferSplitFraction(), comInitialPositionList, comFinalPositionList,
                                        comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeDoubleSupportFootTrajectories(copWaypointList, numberOfFootstepsToConsider.getIntegerValue());
      computeAngularMomentumTrajectory();
      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + transferAngularMomentumTrajectory.getSegment(0).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, transferAngularMomentumTrajectory.getSegment(0), Epsilons.ONE_TEN_THOUSANDTH));
   }

   private int checkSwingAngularMomentumTrajectories(int comListCounter, int stepIndex,
                                                     List<? extends FramePoint3DReadOnly> comInitialPositionList,
                                                     List<? extends FramePoint3DReadOnly> comFinalPositionList,
                                                     List<? extends FrameVector3DReadOnly> comInitialVelocityList,
                                                     List<? extends FrameVector3DReadOnly> comFinalVelocityList,
                                                     List<? extends FrameVector3DReadOnly> comInitialAccelerationList,
                                                     List<? extends FrameVector3DReadOnly> comFinalAccelerationList, List<CoPPointsInFoot> copWaypointList,
                                                     List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories)
   {
      AngularMomentumTrajectory swingAngularMomentumTrajectory = swingAngularMomentumTrajectories.get(stepIndex);
      assertTrue("Swing trajectory " + stepIndex + " has " + swingAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 3.\n"
            + swingAngularMomentumTrajectory.toString(), swingAngularMomentumTrajectory.getNumberOfSegments() == 3);

      assertTrue(swingAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(swingAngularMomentumTrajectory.getSegment(0).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction()
            * testParameters.getSwingSplitFraction());
      comListCounter = setCoMTrajectory(0.0, swingTime * testParameters.getSwingSplitFraction() * testParameters.getSwingDurationShiftFraction(),
                                        comInitialPositionList, comFinalPositionList, comInitialVelocityList, comFinalVelocityList,
                                        comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeSingleSupportFootTrajectories(copWaypointList, stepIndex);
      computeAngularMomentumTrajectory();
      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + swingAngularMomentumTrajectory.getSegment(0).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, swingAngularMomentumTrajectory.getSegment(0), Epsilons.ONE_TEN_THOUSANDTH));

      assertTrue(swingAngularMomentumTrajectory.getSegment(1).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction()
            * testParameters.getSwingSplitFraction());
      assertTrue(swingAngularMomentumTrajectory.getSegment(1).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction());
      comListCounter = setCoMTrajectory(swingTime * testParameters.getSwingSplitFraction() * testParameters.getSwingDurationShiftFraction(),
                                        swingTime * testParameters.getSwingDurationShiftFraction(), comInitialPositionList, comFinalPositionList,
                                        comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeSingleSupportFootTrajectories(copWaypointList, stepIndex);
      computeAngularMomentumTrajectory();
      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + swingAngularMomentumTrajectory.getSegment(1).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, swingAngularMomentumTrajectory.getSegment(1), Epsilons.ONE_TEN_THOUSANDTH));

      assertTrue(swingAngularMomentumTrajectory.getSegment(2).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction());
      assertTrue(swingAngularMomentumTrajectory.getSegment(2).getFinalTime() == swingTime);
      comListCounter = setCoMTrajectory(swingTime * testParameters.getSwingDurationShiftFraction(), swingTime, comInitialPositionList, comFinalPositionList,
                                        comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeSingleSupportFootTrajectories(copWaypointList, stepIndex);
      computeAngularMomentumTrajectory();
      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + swingAngularMomentumTrajectory.getSegment(2).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, swingAngularMomentumTrajectory.getSegment(2), Epsilons.ONE_TEN_THOUSANDTH));
      return comListCounter;
   }

   private int checkTransferAngularMomentumTrajectories(int comListCounter, int stepIndex,
                                                        List<? extends FramePoint3DReadOnly> comInitialPositionList,
                                                        List<? extends FramePoint3DReadOnly> comFinalPositionList,
                                                        List<? extends FrameVector3DReadOnly> comInitialVelocityList,
                                                        List<? extends FrameVector3DReadOnly> comFinalVelocityList,
                                                        List<? extends FrameVector3DReadOnly> comInitialAccelerationList,
                                                        List<? extends FrameVector3DReadOnly> comFinalAccelerationList, List<CoPPointsInFoot> copWaypointList,
                                                        List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories)
   {
      AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(stepIndex);
      assertTrue("Transfer trajectory " + stepIndex + " has " + transferAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 2.\n"
            + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 2);

      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
      comListCounter = setCoMTrajectory(0.0, transferTime * testParameters.getTransferSplitFraction(), comInitialPositionList, comFinalPositionList,
                                        comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeDoubleSupportFootTrajectories(copWaypointList, stepIndex);
      computeAngularMomentumTrajectory();
      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + transferAngularMomentumTrajectory.getSegment(0).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, transferAngularMomentumTrajectory.getSegment(0), Epsilons.ONE_TEN_THOUSANDTH));

      assertTrue(transferAngularMomentumTrajectory.getSegment(1).getInitialTime() == testParameters.getTransferSplitFraction() * transferTime);
      assertTrue(transferAngularMomentumTrajectory.getSegment(1).getFinalTime() == transferTime);
      comListCounter = setCoMTrajectory(transferTime * testParameters.getTransferSplitFraction(), transferTime, comInitialPositionList, comFinalPositionList,
                                        comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, comListCounter);
      computeDoubleSupportFootTrajectories(copWaypointList, stepIndex);
      computeAngularMomentumTrajectory();

      assertTrue(angularMomentumTrajectory.toString() + "\n\n" + transferAngularMomentumTrajectory.getSegment(1).toString(),
                 TrajectoryMathTools.epsilonEquals(angularMomentumTrajectory, transferAngularMomentumTrajectory.getSegment(1), Epsilons.ONE_TEN_THOUSANDTH));
      return comListCounter;
   }

   @Test
   public void testAngularMomentumSwing()
   {
      setupInputs();
      isInitialTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(false);
      copTrajectoryGenerator.initializeForSwing();
      copTrajectoryGenerator.computeReferenceCoPsStartingFromSingleSupport(RobotSide.LEFT);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      icpTrajectoryGenerator.initializeForSwingFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.computeTrajectoryStartingFromSingleSupport(copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                                        icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());

      List<? extends FramePoint3DReadOnly> comInitialPositionList = comTrajectoryGenerator.getCoMPositionDesiredInitialList();
      List<? extends FramePoint3DReadOnly> comFinalPositionList = comTrajectoryGenerator.getCoMPositionDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalVelocityList = comTrajectoryGenerator.getCoMVelocityDesiredFinalList();
      List<? extends FrameVector3DReadOnly> comInitialAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredInitialList();
      List<? extends FrameVector3DReadOnly> comFinalAccelerationList = comTrajectoryGenerator.getCoMAccelerationDesiredFinalList();
      List<CoPPointsInFoot> copWaypointList = copTrajectoryGenerator.getWaypoints();

      angularMomentumGenerator.addCopAndComSetpointsToPlan(copTrajectoryGenerator.getWaypoints(), comInitialPositionList, comFinalPositionList,
                                                           comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList,
                                                           copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();
      angularMomentumGenerator.initializeForSingleSupport(0.0);

      int stepIndex = 0, comListCounter = 0;
      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();

      comListCounter = checkSwingAngularMomentumTrajectories(comListCounter, stepIndex, comInitialPositionList, comFinalPositionList, comInitialVelocityList,
                                                             comFinalVelocityList, comInitialAccelerationList, comFinalAccelerationList, copWaypointList,
                                                             swingAngularMomentumTrajectories);

      checkAngularMomentumTrajectories(comListCounter, stepIndex + 1, comInitialPositionList, comFinalPositionList, comInitialVelocityList, comFinalVelocityList, comInitialAccelerationList,
                                       comFinalAccelerationList, copWaypointList, swingAngularMomentumTrajectories, transferAngularMomentumTrajectories);
   }

   private void setupInputs()
   {
      clear();
      RobotSide side = RobotSide.LEFT;
      for (int i = 0; i < testParameters.getNumberOfFootstepsToConsider(); i++)
      {
         // Update the current location based on walking direction
         tempFrameVector.setIncludingFrame(walkingDirectionUnitVector);
         tempFrameVector.scale(stepLength);
         currentLocation.add(tempFrameVector);

         // Create new footstep based on updated CoM location
         Footstep newFootstep = new Footstep();
         newFootstep.setRobotSide(side);
         getFootLocationFromCoMLocation(tempFramePoint1, side, currentLocation, walkingDirectionUnitVector, stepLength, stepWidth);

         // Set calculated pose to footstep
         newFootstep.setPose(tempFramePoint1, robotOrientation);
         upcomingFootstepsData.add(new FootstepData(newFootstep, new FootstepTiming(swingTime, transferTime)));
         side = side.getOppositeSide();
      }

      numberOfUpcomingFootsteps.set(upcomingFootstepsData.size());
   }
   
   public static void main(String args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(FootstepAngularMomentumPredictor.class, FootstepAngularMomentumPredictorTest.class);
   }
}
