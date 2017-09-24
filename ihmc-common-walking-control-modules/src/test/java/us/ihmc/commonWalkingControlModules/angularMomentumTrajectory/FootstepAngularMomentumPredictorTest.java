package us.ihmc.commonWalkingControlModules.angularMomentumTrajectory;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.AngularMomentumTrajectory;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.ReferenceICPTrajectoryGenerator;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameOrientation;
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
   private final String testName = "TesAngularMomentum";

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SmoothCMPPlannerParameters testParameters = new SmoothCMPPlannerParameters()
   {
      public boolean planWithAngularMomentum()
      {
         return true;
      };
   };

   // Variables for testing
   private final YoVariableRegistry testRegistry = new YoVariableRegistry("AngularMomentumTestRegistry");
   private final YoDouble omega = new YoDouble("AngularMomentumTestOmega", testRegistry); // Taking the for Atlas
   private final FramePoint3D currentLocation = new FramePoint3D();
   private final FrameVector3D walkingDirectionUnitVector = new FrameVector3D();
   private final FrameOrientation robotOrientation = new FrameOrientation();
   private final QuaternionBasedTransform rightTransform = new QuaternionBasedTransform(new Quaternion(0.0, 0.0, Math.sin(-Math.PI / 4.0),
                                                                                                       Math.cos(-Math.PI / 4.0)),
                                                                                        new FramePoint3D());
   private final QuaternionBasedTransform leftTransform = new QuaternionBasedTransform(new Quaternion(0.0, 0.0, Math.sin(Math.PI / 4.0),
                                                                                                      Math.cos(Math.PI / 4.0)),
                                                                                       new FrameVector3D());
   private final List<YoDouble> swingDurations = new ArrayList<YoDouble>();
   private final List<YoDouble> transferDurations = new ArrayList<YoDouble>();
   private final List<YoDouble> swingSplitFractions = new ArrayList<YoDouble>();
   private final List<YoDouble> transferSplitFractions = new ArrayList<YoDouble>();
   private final List<YoDouble> swingShiftFractions = new ArrayList<YoDouble>();

   // Some variables for setters, getters and intermediate computations
   private final FramePoint3D tempFramePoint = new FramePoint3D();
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

   @Before
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
         getFootLocationFromCoMLocation(tempFramePoint, side, currentLocation, walkingDirectionUnitVector, stepLength, stepWidth);
         foot.setPose(tempFramePoint, robotOrientation);
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
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleFrames, midFeetFrame, soleFrames, testRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      copTrajectoryGenerator = new ReferenceCoPTrajectoryGenerator(testName + "CoPGenerator", testParameters.getNumberOfCoPWayPointsPerFoot(),
                                                                   testParameters.getNumberOfFootstepsToConsider(), bipedSupportPolygons, contactableFeet,
                                                                   numberOfFootstepsToConsider, swingDurations, transferDurations, swingSplitFractions,
                                                                   swingShiftFractions, transferSplitFractions, testRegistry);
      icpTrajectoryGenerator = new ReferenceICPTrajectoryGenerator(testName, omega, numberOfFootstepsToConsider, isStanding, isInitialTransfer, isDoubleSupport,
                                                                   false, testRegistry);
      comTrajectoryGenerator = new ReferenceCoMTrajectoryGenerator(testName, omega, numberOfFootstepsToConsider, isStanding, isInitialTransfer, isDoubleSupport,
                                                                   testRegistry);
      angularMomentumGenerator = new FootstepAngularMomentumPredictor(testName, omega, testRegistry);
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
      contactableFeet.clear();
      soleFrames.clear();
      ankleFrames.clear();
      copTrajectoryGenerator.clear();
      icpTrajectoryGenerator.reset();
      comTrajectoryGenerator.reset();
      angularMomentumGenerator.clear();
   }

   @After
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
      copTrajectoryGenerator.computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), null);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      copTrajectoryGenerator.initializeForTransfer(0.0);
      icpTrajectoryGenerator.initializeForTransferFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(),
                                                           copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.initializeForTransfer(0.0, copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                   icpTrajectoryGenerator.getICPPositionFromCoPDesiredInitialList(),
                                                   icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());

      angularMomentumGenerator.addFootstepCoPsToPlan(copTrajectoryGenerator.getWaypoints(), comTrajectoryGenerator.getCoMPositionDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMPositionDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredFinalList(),
                                                     copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getBooleanValue());
      angularMomentumGenerator.initializeForDoubleSupport(0.0, isStanding.getBooleanValue());

      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(i);
         assertTrue("Transfer trajectory " + i + " has " + transferAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 2.\n"
               + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 2);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getInitialTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getFinalTime() == transferTime);
      }
      AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(numberOfFootstepsToConsider.getIntegerValue());
      assertTrue("Transfer trajectory " + numberOfFootstepsToConsider.getIntegerValue() + " has " + transferAngularMomentumTrajectory.getNumberOfSegments()
            + " segments, needed 1.\n" + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 1);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory swingAngularMomentumTrajectory = swingAngularMomentumTrajectories.get(i);
         assertTrue("Swing trajectory " + i + " has " + swingAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 3.\n"
               + swingAngularMomentumTrajectory.toString(), swingAngularMomentumTrajectory.getNumberOfSegments() == 3);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getFinalTime() == swingTime);
      }
   }

   @Test
   public void testAngularMomentumNormalTransfer()
   {
      setupInputs();
      isInitialTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(true);
      copTrajectoryGenerator.computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), null);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      copTrajectoryGenerator.initializeForTransfer(0.0);
      icpTrajectoryGenerator.initializeForTransferFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(),
                                                           copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.initializeForTransfer(0.0, copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                   icpTrajectoryGenerator.getICPPositionFromCoPDesiredInitialList(),
                                                   icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());

      angularMomentumGenerator.addFootstepCoPsToPlan(copTrajectoryGenerator.getWaypoints(), comTrajectoryGenerator.getCoMPositionDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMPositionDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredFinalList(),
                                                     copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getBooleanValue());
      angularMomentumGenerator.initializeForDoubleSupport(0.0, isStanding.getBooleanValue());

      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(i);
         assertTrue("Transfer trajectory " + i + " has " + transferAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 2.\n"
               + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 2);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getInitialTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getFinalTime() == transferTime);
      }
      AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(numberOfFootstepsToConsider.getIntegerValue());
      assertTrue("Transfer trajectory " + numberOfFootstepsToConsider.getIntegerValue() + " has " + transferAngularMomentumTrajectory.getNumberOfSegments()
            + " segments, needed 1.\n" + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 1);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory swingAngularMomentumTrajectory = swingAngularMomentumTrajectories.get(i);
         assertTrue("Swing trajectory " + i + " has " + swingAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 3.\n"
               + swingAngularMomentumTrajectory.toString(), swingAngularMomentumTrajectory.getNumberOfSegments() == 3);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getFinalTime() == swingTime);
      }
   }

   @Test
   public void testAngularMomentumSwing()
   {
      setupInputs();
      isInitialTransfer.set(false);
      isStanding.set(false);
      isDoubleSupport.set(false);
      copTrajectoryGenerator.computeReferenceCoPsStartingFromSingleSupport(null);
      icpTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());

      copTrajectoryGenerator.initializeForSwing(0.0);
      icpTrajectoryGenerator.initializeForSwingFromCoPs(copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories());
      icpTrajectoryGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      comTrajectoryGenerator.setNumberOfRegisteredSteps(copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      comTrajectoryGenerator.initializeForSwing(0.0, copTrajectoryGenerator.getTransferCoPTrajectories(), copTrajectoryGenerator.getSwingCoPTrajectories(),
                                                icpTrajectoryGenerator.getICPPositionFromCoPDesiredInitialList(),
                                                icpTrajectoryGenerator.getICPPositonFromCoPDesiredFinalList());

      angularMomentumGenerator.addFootstepCoPsToPlan(copTrajectoryGenerator.getWaypoints(), comTrajectoryGenerator.getCoMPositionDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMPositionDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMVelocityDesiredFinalList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredInitialList(),
                                                     comTrajectoryGenerator.getCoMAccelerationDesiredFinalList(),
                                                     copTrajectoryGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();
      angularMomentumGenerator.initializeForSingleSupport(0.0);

      List<? extends AngularMomentumTrajectory> swingAngularMomentumTrajectories = angularMomentumGenerator.getSwingAngularMomentumTrajectories();
      List<? extends AngularMomentumTrajectory> transferAngularMomentumTrajectories = angularMomentumGenerator.getTransferAngularMomentumTrajectories();

      for (int i = 1; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(i);
         assertTrue("Transfer trajectory " + i + " has " + transferAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 2.\n"
               + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 2);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getInitialTime() == testParameters.getTransferSplitFraction() * transferTime);
         assertTrue(transferAngularMomentumTrajectory.getSegment(1).getFinalTime() == transferTime);
      }
      AngularMomentumTrajectory transferAngularMomentumTrajectory = transferAngularMomentumTrajectories.get(numberOfFootstepsToConsider.getIntegerValue());
      assertTrue("Transfer trajectory " + numberOfFootstepsToConsider.getIntegerValue() + " has " + transferAngularMomentumTrajectory.getNumberOfSegments()
            + " segments, needed 1.\n" + transferAngularMomentumTrajectory.toString(), transferAngularMomentumTrajectory.getNumberOfSegments() == 1);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
      assertTrue(transferAngularMomentumTrajectory.getSegment(0).getFinalTime() == testParameters.getTransferSplitFraction() * transferTime);
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         AngularMomentumTrajectory swingAngularMomentumTrajectory = swingAngularMomentumTrajectories.get(i);
         assertTrue("Swing trajectory " + i + " has " + swingAngularMomentumTrajectory.getNumberOfSegments() + " segments, needed 3.\n"
               + swingAngularMomentumTrajectory.toString(), swingAngularMomentumTrajectory.getNumberOfSegments() == 3);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getInitialTime() == 0.0);
         assertTrue(swingAngularMomentumTrajectory.getSegment(0).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction() * testParameters.getSwingSplitFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(1).getFinalTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getInitialTime() == swingTime * testParameters.getSwingDurationShiftFraction());
         assertTrue(swingAngularMomentumTrajectory.getSegment(2).getFinalTime() == swingTime);
      }

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
         getFootLocationFromCoMLocation(tempFramePoint, side, currentLocation, walkingDirectionUnitVector, stepLength, stepWidth);

         // Set calculated pose to footstep
         newFootstep.setPose(tempFramePoint, robotOrientation);
         copTrajectoryGenerator.addFootstepToPlan(newFootstep, new FootstepTiming(swingTime, transferTime));
         side = side.getOppositeSide();
      }
   }
}
