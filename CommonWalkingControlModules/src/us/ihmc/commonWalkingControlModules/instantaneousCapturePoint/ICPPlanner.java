package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.lists.FrameTupleArrayList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class ICPPlanner
{
   private static final boolean VISUALIZE = true;
   private static final double ICP_CORNER_POINT_SIZE = 0.004;
   private static final double ICP_CONSTANT_COP_POINT_SIZE = 0.005;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpPlanner";

   private final BooleanYoVariable atAStop = new BooleanYoVariable(namePrefix + "AtAStop", registry);
   private final BooleanYoVariable comeToStop = new BooleanYoVariable(namePrefix + "ComeToStop", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final BooleanYoVariable hasBeenWokenUp = new BooleanYoVariable(namePrefix + "HasBeenWokenUp", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable isDoneTimeThreshold = new DoubleYoVariable(namePrefix + "isDoneTimeThreshold", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(namePrefix + "DoubleSupportTime", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(namePrefix + "SingleSupportTime", registry);
   private final DoubleYoVariable minSingleSupportDuration = new DoubleYoVariable(namePrefix + "MinSingleSupportTime", registry);
   private final DoubleYoVariable timeCorrectionFactor = new DoubleYoVariable(namePrefix + "TimeCorrectionFactor", registry);
   private final DoubleYoVariable doubleSupportInitialTransferDuration = new DoubleYoVariable(namePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable(namePrefix + "DoubleSupportSplitFractior", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable(namePrefix + "RemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);
   private final IntegerYoVariable footstepsToStop = new IntegerYoVariable(namePrefix + "NumberFootstepsToStop", registry);
   private final YoFramePoint singleSupportInitialDesiredICP = new YoFramePoint(namePrefix + "FinalDesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector singleSupportInitialDesiredICPVelocity = new YoFrameVector(namePrefix + "FinalDesiredCapturePointVelocity", worldFrame, registry);
   private final YoFramePoint nextSingleSupportInitialDesiredICP = new YoFramePoint(namePrefix + "NextSingleSupportInitialCapturePointPosition", worldFrame, registry);
   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);

   private final DoubleYoVariable constantCMPInFromFootCenter = new DoubleYoVariable(namePrefix + "constantCMPInFromFootCenter", registry);
   private final DoubleYoVariable constantCMPForwardFromFootCenter = new DoubleYoVariable(namePrefix + "constantCMPForwardFromFootCenter", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final FramePoint icpInRearFootFrame = new FramePoint(worldFrame);
   private final FrameVector icpVelocityInRearFootFrame = new FrameVector(worldFrame);

   private final FramePoint icpInFrontFootFrame = new FramePoint(worldFrame);
   private final FrameVector icpVelocityInFrontFootFrame = new FrameVector(worldFrame);

   private final FramePoint icpInSingleSupportFootFrame = new FramePoint();
   private final FramePoint cmpInSingleSupportFootFrame = new FramePoint();

   private final ArrayList<YoFramePoint> constantCMPs = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFramePoint> capturePointCornerPoints = new ArrayList<YoFramePoint>();
   private final FrameTupleArrayList<FramePoint> footstepLocations = FrameTupleArrayList.createFramePointArrayList();

   private final ICPPlannerDoubleSupportTrajectoryGenerator doubleSupportCapturePointTrajectory;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final CommonHumanoidReferenceFrames referenceFrames;

   public ICPPlanner(BipedSupportPolygons bipedSupportPolygons, CommonHumanoidReferenceFrames referenceFrames, CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.referenceFrames = referenceFrames;

      atAStop.set(true);
      hasBeenWokenUp.set(false);

      int numberOfCoefficients = capturePointPlannerParameters.getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory();
      doubleSupportCapturePointTrajectory = new ICPPlannerDoubleSupportTrajectoryGenerator(namePrefix + "DoubleSupportTrajectory", numberOfCoefficients, registry);
      doubleSupportInitialTransferDuration.set(capturePointPlannerParameters.getDoubleSupportInitialTransferDuration());
      numberFootstepsToConsider.set(capturePointPlannerParameters.getNumberOfFootstepsToConsider());
      footstepsToStop.set(capturePointPlannerParameters.getNumberOfFootstepsToStop());
      isDoneTimeThreshold.set(capturePointPlannerParameters.getIsDoneTimeThreshold());
      doubleSupportSplitFraction.set(capturePointPlannerParameters.getDoubleSupportSplitFraction());

      constantCMPInFromFootCenter.set(capturePointPlannerParameters.getCapturePointInFromFootCenterDistance());
      constantCMPForwardFromFootCenter.set(capturePointPlannerParameters.getCapturePointForwardFromFootCenterDistance());

      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      minSingleSupportDuration.set(0.3); // TODO Need to be extracted
      timeCorrectionFactor.set(0.5); // TODO Need to be extracted

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoFramePoint constantCopYoFramePoint = new YoFramePoint("icpConstantCoP" + i, worldFrame, registry);
         constantCMPs.add(constantCopYoFramePoint);
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePoint icpCornerPointYoFramePoint = new YoFramePoint("icpCornerPoints" + i, worldFrame, registry);
         capturePointCornerPoints.add(icpCornerPointYoFramePoint);
      }

      parentRegistry.addChild(registry);

      if (VISUALIZE && yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("ICPComputer");
      ArtifactList artifactList = new ArtifactList("ICPPlanner");

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         YoGraphicPosition constantFootCenterCentersOfPressureViz = new YoGraphicPosition("icpConstantCoP" + i, constantCMPs.get(i), ICP_CONSTANT_COP_POINT_SIZE, YoAppearance.Green(), GraphicType.SOLID_BALL);
         yoGraphicsList.add(constantFootCenterCentersOfPressureViz);
         artifactList.add(constantFootCenterCentersOfPressureViz.createArtifact());
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoGraphicPosition icpCornerPointsViz = new YoGraphicPosition("icpCornerPoints" + i, capturePointCornerPoints.get(i), ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpCornerPointsViz);
         artifactList.add(icpCornerPointsViz.createArtifact());
      }

      YoGraphicPosition singleSupportInitialICP = new YoGraphicPosition("singleSupportInitialICP", singleSupportInitialDesiredICP, 0.004, YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition nextSingleSupportInitialICP = new YoGraphicPosition("nextSingleSupportInitialICP", nextSingleSupportInitialDesiredICP, 0.004, YoAppearance.Chocolate(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(singleSupportInitialICP);
      yoGraphicsList.add(nextSingleSupportInitialICP);
      artifactList.add(singleSupportInitialICP.createArtifact());
      artifactList.add(nextSingleSupportInitialICP.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearPlan()
   {
      footstepLocations.clear();
   }

   public void addFootstepToPlan(Footstep footstep)
   {
      if (footstep == null)
         return;

      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      FramePoint footstepLocation = footstepLocations.add();
      footstepLocation.setToZero(footstep.getSoleReferenceFrame());

      if (predictedContactPoints != null)
      {
         int numberOfContactPoints = predictedContactPoints.size();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            Point2d contactPoint = predictedContactPoints.get(i);
            footstepLocation.setX(footstepLocation.getX() + contactPoint.getX() / (double) numberOfContactPoints);
            footstepLocation.setY(footstepLocation.getY() + contactPoint.getY() / (double) numberOfContactPoints);
         }
      }

      footstepLocation.setX(footstepLocation.getX() + constantCMPForwardFromFootCenter.getDoubleValue());
      footstepLocation.setY(footstepLocation.getY() + footstep.getRobotSide().negateIfLeftSide(constantCMPInFromFootCenter.getDoubleValue()));
      footstepLocation.changeFrame(worldFrame);
   }

   public void setDesiredCapturePointState(YoFramePoint2d currentDesiredCapturePointPosition, YoFrameVector2d currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.setXY(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.setXY(currentDesiredCapturePointVelocity);
   }

   public void initializeDoubleSupport(double initialTime, RobotSide transferToSide)
   {
      this.transferToSide.set(transferToSide);
      this.supportSide.set(null);

      if (transferToSide == null) transferToSide = RobotSide.LEFT;
      RobotSide transferFromSide = transferToSide.getOppositeSide();

      addSupportFootLocation(transferToSide);
      addSupportFootLocation(transferFromSide);

      isDoubleSupport.set(true);

      comeToStop.set(footstepLocations.size() <= footstepsToStop.getIntegerValue());
      this.initialTime.set(initialTime);

      computeConstantCMPs(this.footstepLocations);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      if (comeToStop.getBooleanValue())
      {
         singleSupportInitialDesiredICP.set(constantCMPs.get(0));
         singleSupportInitialDesiredICPVelocity.set(0.0, 0.0, 0.0);
      }
      else
      {
         double reducedDoubleSupportTime = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), reducedDoubleSupportTime, capturePointCornerPoints.get(1), constantCMPs.get(1), singleSupportInitialDesiredICP);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportInitialDesiredICP, constantCMPs.get(1), singleSupportInitialDesiredICPVelocity);
      }

      DoubleYoVariable doubleSupportTimeToUse = atAStop.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;

      desiredCapturePointPosition.getFrameTupleIncludingFrame(icpInRearFootFrame);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(icpVelocityInRearFootFrame);
      singleSupportInitialDesiredICP.getFrameTupleIncludingFrame(icpInFrontFootFrame);
      singleSupportInitialDesiredICPVelocity.getFrameTupleIncludingFrame(icpVelocityInFrontFootFrame);

      icpInRearFootFrame.changeFrame(referenceFrames.getSoleFrame(transferFromSide));
      icpVelocityInRearFootFrame.changeFrame(referenceFrames.getSoleFrame(transferFromSide));
      icpInFrontFootFrame.changeFrame(referenceFrames.getSoleFrame(transferToSide));
      icpVelocityInFrontFootFrame.changeFrame(referenceFrames.getSoleFrame(transferToSide));

      doubleSupportCapturePointTrajectory.setTrajectoryTime(doubleSupportTimeToUse.getDoubleValue());
      doubleSupportCapturePointTrajectory.setInitialConditions(icpInRearFootFrame, icpVelocityInRearFootFrame);
      doubleSupportCapturePointTrajectory.setFinalConditions(icpInFrontFootFrame, icpVelocityInFrontFootFrame);
      doubleSupportCapturePointTrajectory.initialize();
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.transferToSide.set(null);
      this.supportSide.set(supportSide);

      atAStop.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);

      addSupportFootLocation(supportSide);

      comeToStop.set(footstepLocations.size() <= footstepsToStop.getIntegerValue());

      computeConstantCMPs(this.footstepLocations);
      computeCapturePointCornerPoints(doubleSupportDuration.getDoubleValue() + singleSupportDuration.getDoubleValue());

      double reducedDoubleSupportTime = doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue();
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), reducedDoubleSupportTime, capturePointCornerPoints.get(0), constantCMPs.get(0), singleSupportInitialDesiredICP);
      CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), reducedDoubleSupportTime, capturePointCornerPoints.get(1), constantCMPs.get(1), nextSingleSupportInitialDesiredICP);

      singleSupportInitialDesiredICP.getFrameTupleIncludingFrame(icpInSingleSupportFootFrame);
      constantCMPs.get(0).getFrameTupleIncludingFrame(cmpInSingleSupportFootFrame);

      icpInSingleSupportFootFrame.changeFrame(referenceFrames.getSoleFrame(supportSide));
      cmpInSingleSupportFootFrame.changeFrame(referenceFrames.getSoleFrame(supportSide));
   }

   public void updatePlanForSingleSupportDisturbances(double time, FramePoint actualCapturePointPosition)
   {
      initializeSingleSupport(initialTime.getDoubleValue(), supportSide.getEnumValue());

      YoFramePoint constantCMP = constantCMPs.get(0);
      double actualDistanceDueToDisturbance = constantCMP.distance(actualCapturePointPosition);
      double expectedDistanceAccordingToPlan = constantCMP.distance(singleSupportInitialDesiredICP);

      double correctedTimeInCurrentState = Math.log(actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan) / omega0.getDoubleValue();

      computeTimeInCurrentState(time);
      double deltaTimeToBeAccounted = correctedTimeInCurrentState - timeInCurrentState.getDoubleValue();

      if (!Double.isNaN(deltaTimeToBeAccounted))
      {
         deltaTimeToBeAccounted *= timeCorrectionFactor.getDoubleValue();
         deltaTimeToBeAccounted = MathTools.clipToMinMax(deltaTimeToBeAccounted, 0.0, Math.max(0.0, computeAndReturnTimeRemaining(time) - minSingleSupportDuration.getDoubleValue()));
         
         initialTime.sub(deltaTimeToBeAccounted);
      }
   }

   private void addSupportFootLocation(RobotSide supportSide)
   {
      FramePoint footLocation;

      if (footstepLocations.isEmpty())
         footLocation = footstepLocations.add();
      else
         footLocation = footstepLocations.insertAtIndex(0);

      FrameConvexPolygon2d supportFootPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      footLocation.setXYIncludingFrame(supportFootPolygon.getCentroid());
      footLocation.setX(footLocation.getX() + constantCMPForwardFromFootCenter.getDoubleValue());
      footLocation.setY(footLocation.getY() + supportSide.negateIfLeftSide(constantCMPInFromFootCenter.getDoubleValue()));
      footLocation.setZ(0.0); // Otherwise we end up with the ankle height
      footLocation.changeFrame(worldFrame);
   }

   protected void computeConstantCMPs(FrameTupleArrayList<FramePoint> footstepList)
   {
      comeToStop.set(footstepList.size() <= footstepsToStop.getIntegerValue());

      int indexOfLastFootstepToConsider = Math.min(footstepList.size(), numberFootstepsToConsider.getIntegerValue()) - 1;

      CapturePointTools.computeConstantCMPs(constantCMPs, footstepList, 0, indexOfLastFootstepToConsider, atAStop.getBooleanValue(), comeToStop.getBooleanValue());
   }

   protected void computeCapturePointCornerPoints(double steppingDuration)
   {
      CapturePointTools.computeDesiredCornerPoints(capturePointCornerPoints, constantCMPs, false, steppingDuration, omega0.getDoubleValue());
   }

   protected void computeDesiredCapturePoint(double time)
   {
      if (isDoubleSupport.getBooleanValue())
      {
         doubleSupportCapturePointTrajectory.compute(time);
         doubleSupportCapturePointTrajectory.packLinearData(desiredCapturePointPosition, desiredCapturePointVelocity, desiredCapturePointAcceleration);
         singleSupportInitialDesiredICP.setAndMatchFrame(icpInFrontFootFrame);
         singleSupportInitialDesiredICPVelocity.setAndMatchFrame(icpVelocityInFrontFootFrame);
      }
      else
      {
         singleSupportInitialDesiredICP.setAndMatchFrame(icpInSingleSupportFootFrame);
         constantCMPs.get(0).setAndMatchFrame(cmpInSingleSupportFootFrame);
         time = MathTools.clipToMinMax(time, 0.0, singleSupportDuration.getDoubleValue());
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), time, singleSupportInitialDesiredICP, constantCMPs.get(0), desiredCapturePointPosition);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), time, singleSupportInitialDesiredICP, constantCMPs.get(0), desiredCapturePointVelocity);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), time, singleSupportInitialDesiredICP, constantCMPs.get(0), desiredCapturePointAcceleration);
      }
   }

   protected void computeDesiredCentroidalMomentumPivot()
   {
      if (isDoubleSupport.getBooleanValue())
         CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePointPosition, desiredCapturePointVelocity, omega0.getDoubleValue(), desiredCentroidalMomentumPivotPosition);
      else
         desiredCentroidalMomentumPivotPosition.set(constantCMPs.get(0));
   }

   public void packDesiredCapturePointPositionVelocityAndAcceleration(FramePoint desiredCapturePointPositionToPack,
         FrameVector desiredCapturePointVelocityToPack, FrameVector desiredCapturePointAccelerationToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
      desiredCapturePointAcceleration.getFrameTupleIncludingFrame(desiredCapturePointAccelerationToPack);
   }

   public void packDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   protected void packDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack, double time)
   {
      computeTimeInCurrentState(time);

      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPositionToPack.set(desiredCapturePointPosition);
      desiredCapturePointVelocityToPack.set(desiredCapturePointVelocity);
   }

   public void packDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      computeDesiredCentroidalMomentumPivot();

      desiredCentroidalMomentumPivotPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   protected void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   public double computeAndReturnTimeRemaining(double time)
   {
      computeTimeInCurrentState(time);

      DoubleYoVariable stateDuration;

      if (isDoubleSupport.getBooleanValue())
         stateDuration = atAStop.getBooleanValue() ? doubleSupportInitialTransferDuration : doubleSupportDuration;
      else
         stateDuration = singleSupportDuration;

      remainingTime.set(stateDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      return remainingTime.getDoubleValue();
   }

   public void setInitialDoubleSupportTime(double time)
   {
      doubleSupportInitialTransferDuration.set(time);
   }

   public void setDoubleSupportTime(double time)
   {
      doubleSupportDuration.set(time);
   }

   public void setSingleSupportTime(double time)
   {
      singleSupportDuration.set(time);
   }

   public void setDoubleSupportSplitFraction(double doubleSupportSplitFraction)
   {
      this.doubleSupportSplitFraction.set(doubleSupportSplitFraction);
   }

   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   public void getSingleSupportInitialCapturePointPosition(FramePoint capturePointPositionToPack)
   {
      singleSupportInitialDesiredICP.getFrameTupleIncludingFrame(capturePointPositionToPack);
   }

   public ArrayList<YoFramePoint> getCapturePointCornerPoints()
   {
      return capturePointCornerPoints;
   }

   public ArrayList<YoFramePoint> getConstantCentroidalMomentumPivots()
   {
      return constantCMPs;
   }

   public double computeAndReturnTimeInCurrentState(double time)
   {
      computeTimeInCurrentState(time);
      return timeInCurrentState.getDoubleValue();
   }

   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      capturePointCornerPoints.get(1).getFrameTupleIncludingFrame(finalDesiredCapturePointPositionToPack);
   }

   public void getConstantCentroidalMomentumPivotPosition(FramePoint constantCentroidalMomentumPositionToPack)
   {
      constantCMPs.get(0).getFrameTupleIncludingFrame(constantCentroidalMomentumPositionToPack);
   }

   public void reset(double time)
   {
      atAStop.set(true);

      clearPlan();
      initializeDoubleSupport(time, null);
   }

   public boolean getHasBeenWokenUp()
   {
      return hasBeenWokenUp.getBooleanValue();
   }

   public void wakeUp()
   {
      hasBeenWokenUp.set(true);
   }

   public boolean isDone(double time)
   {
      double timeRemaining = computeAndReturnTimeRemaining(time);
      return (timeRemaining <= isDoneTimeThreshold.getDoubleValue());
   }
}
