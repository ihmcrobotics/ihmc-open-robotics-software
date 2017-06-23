package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CoPPolynomialTrajectoryPlannerInterface;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCoPTrajectoryCalculator implements CoPPolynomialTrajectoryPlannerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double COP_POINT_SIZE = 0.005;

   private static final int maxNumberOfPointsInFoot = 4;
   private static final int maxNumberOfFootstepsToConsider = 4;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   private final YoBoolean isDoneWalking;
   private final List<YoDouble> maxCoPOffsets = new ArrayList<>();
   private final List<YoDouble> minCoPOffsets = new ArrayList<>();

   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingDurationShiftFractions;

   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferSplitFractions;

   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   private CoPTrajectory activeTrajectory;
   private double initialTime;

   private final SideDependentList<List<YoFrameVector2d>> copUserOffsets = new SideDependentList<>();

   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble stepLengthToCoPOffsetFactor;

   private final YoInteger numberOfUpcomingFootsteps;
   private final YoInteger numberOfPointsPerFoot;
   private final YoInteger numberOfFootstepsToConsider;

   private final YoEnum<CoPSplineType> orderOfSplineInterpolation;

   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<FootstepData>(10, FootstepData.class);

   private final FramePoint tmpCoP = new FramePoint();
   private final FramePoint firstCoP = new FramePoint();
   private final FramePoint secondCoP = new FramePoint();

   private final FramePoint2d tmpCoP2d = new FramePoint2d();
   private final FramePoint2d previousCoP2d = new FramePoint2d();
   private final FramePoint2d firstHeelCoPForSingleSupport = new FramePoint2d();

   private final FramePoint desiredCoP = new FramePoint();
   private final FrameVector desiredCoPVelocity = new FrameVector();

   private final List<CoPPointsInFoot> copLocationWaypoints = new ArrayList<>();

   private final FrameConvexPolygon2d tempSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempSupportPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private final FramePoint2d centroidOfUpcomingFootstep = new FramePoint2d();
   private final FramePoint2d centroidOfCurrentFootstep = new FramePoint2d();
   private final FramePoint2d centroidOfFootstepToConsider = new FramePoint2d();

   private final FramePoint tempFramePoint = new FramePoint();

   private final FramePoint currentCoPPosition = new FramePoint();
   private final FrameVector currentCoPVelocity = new FrameVector();
   private final FrameVector currentCoPAcceleration = new FrameVector();
   private final FrameVector finalCoPVelocity = new FrameVector();

   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final FrameConvexPolygon2d predictedSupportPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   private final YoDouble percentageChickenSupport;

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCoPTrajectoryCalculator(String namePrefix, SmoothCMPPlannerParameters plannerParameters,
                                           BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                           YoInteger numberOfFootstepsToConsider, List<YoDouble> swingDurations, List<YoDouble> transferDurations,
                                           List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions,
                                           List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;

      firstHeelCoPForSingleSupport.setToNaN();

      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      stepLengthToCoPOffsetFactor = new YoDouble(namePrefix + "StepLengthToCMPOffsetFactor", registry);

      int numberOfPointsPerFoot = plannerParameters.getNumberOfWayPointsPerFoot();
      double[] maxCoPOffsets = plannerParameters.getMaxCoPForwardOffsetsFootFrame();
      double[] minCoPOffsets = plannerParameters.getMinCoPForwardOffsetsFootFrame();

      for (int waypointIndex = 0; waypointIndex < numberOfPointsPerFoot; waypointIndex++)
      {
         YoDouble maxCoPOffset = new YoDouble("maxCoPForwardOffset" + waypointIndex, registry);
         YoDouble minCoPOffset = new YoDouble("minCoPForwardOffset" + waypointIndex, registry);
         maxCoPOffset.set(maxCoPOffsets[waypointIndex]);
         minCoPOffset.set(minCoPOffsets[waypointIndex]);
         this.maxCoPOffsets.add(maxCoPOffset);
         this.minCoPOffsets.add(minCoPOffset);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         supportFootPolygonsInSoleZUpFrames.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         List<YoFrameVector2d> copUserOffsets = new ArrayList<>();

         for (int i = 0; i < numberOfPointsPerFoot; i++)
         {
            YoFrameVector2d copUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "CoPConstantOffset" + i, null, registry);
            copUserOffsets.add(copUserOffset);
         }

         this.copUserOffsets.put(robotSide, copUserOffsets);
      }

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, transferDurations.get(i), transferSplitFractions.get(i), registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, swingDurations.get(i), swingSplitFractions.get(i), swingDurationShiftFractions.get(i), registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      int index = numberOfFootstepsToConsider.getIntegerValue();
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, index, transferDurations.get(index), transferSplitFractions.get(index), registry);
      transferCoPTrajectories.add(transferCoPTrajectory);


      this.numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfUpcomingFootsteps.set(0);

      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.numberOfPointsPerFoot.set(numberOfPointsPerFoot);

      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.orderOfSplineInterpolation.set(plannerParameters.getOrderOfCoPInterpolation());

      ReferenceFrame midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT)};
      for (int i = 0 ; i < maxNumberOfFootstepsToConsider; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i,  maxNumberOfPointsInFoot, framesToRegister, registry));
      }

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      percentageChickenSupport.set(0.5);

      initializeParameters(plannerParameters);
      clear();

      parentRegistry.addChild(registry);
   }

   public void updateListeners()
   {
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).notifyVariableChangedListeners();
   }

   public void initializeParameters(SmoothCMPPlannerParameters parameters)
   {
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());
      stepLengthToCoPOffsetFactor.set(parameters.getStepLengthToCoPOffsetFactor());

      List<Vector2D> copOffsets = parameters.getCoPOffsetsFootFrame();

      for (int waypointNumber = 0; waypointNumber < copOffsets.size(); waypointNumber++)
         setSymmetricCoPConstantOffsets(waypointNumber, copOffsets.get(waypointNumber));
   }

   public void setSymmetricCoPConstantOffsets(int waypointNumber, Vector2D heelOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copUserOffsets.get(robotSide).get(waypointNumber);
         copUserOffset.setX(heelOffset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(heelOffset.getY()));
      }
   }

   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int footIndex = 0; footIndex < copLocationWaypoints.size(); footIndex++)
      {
         for (int waypointIndex = 0; waypointIndex < maxNumberOfPointsInFoot; waypointIndex++)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(footIndex + "Foot CoP Waypoint" + waypointIndex,
                                                             copLocationWaypoints.get(footIndex).getWaypointInWorldFrameReadOnly(waypointIndex), COP_POINT_SIZE,
                                                             YoAppearance.Green(), GraphicType.BALL);
            yoGraphicsList.add(copViz);
            artifactList.add(copViz.createArtifact());
         }
      }
   }

   public void clear()
   {
      upcomingFootstepsData.clear();
      numberOfUpcomingFootsteps.set(0);
      currentCoPPosition.setToNaN();
      currentCoPVelocity.setToNaN();

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         copLocationWaypoints.get(i).reset();
         transferCoPTrajectories.get(i).reset();
         swingCoPTrajectories.get(i).reset();
      }
   }

   /**
    * Add footstep location to planned
    * @param footstep
    */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null && timing != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
            upcomingFootstepsData.add().set(footstep, timing);
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }

      numberOfUpcomingFootsteps.increment();
   }

   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getIntegerValue();
   }

   public void update(double currentTime)
   {
      double timeInState = currentTime - initialTime;

      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredCoP, desiredCoPVelocity);
   }

   public void getDesiredCenterOfPressure(FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
   {
      desiredCoPToPack.setIncludingFrame(desiredCoP);
      desiredCoPVelocityToPack.setIncludingFrame(desiredCoPVelocity);
   }

   public void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack, YoFrameVector desiredCoPVelocityToPack)
   {
      desiredCoPToPack.set(desiredCoP);
      desiredCoPVelocityToPack.set(desiredCoPVelocity);
   }

   public void initializeForTransfer(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = transferCoPTrajectories.get(0);
   }

   public void initializeForSwing(double currentTime)
   {
      initialTime = currentTime;
      activeTrajectory = swingCoPTrajectories.get(0);
   }

   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      int numberOfUpcomingFootsteps = upcomingFootstepsData.size();
      this.numberOfUpcomingFootsteps.set(numberOfUpcomingFootsteps);
      int footIndex = 0;
      int stepIndex = 0;
      boolean noUpcomingFootsteps = numberOfUpcomingFootsteps == 0;
      isDoneWalking.set(noUpcomingFootsteps);
      ReferenceFrame transferToSoleFrame = soleZUpFrames.get(transferToSide);
      ReferenceFrame transferFromSoleFrame = soleZUpFrames.get(transferFromSide);

      if (atAStop || noUpcomingFootsteps)
      {
         FrameConvexPolygon2d footA = supportFootPolygonsInSoleZUpFrames.get(transferFromSide);
         FrameConvexPolygon2d footB = supportFootPolygonsInSoleZUpFrames.get(transferFromSide.getOppositeSide());
         computeFinalCoPBetweenSupportFeet(footIndex, footA, footB);
         footIndex++;
         stepIndex++;

         if (noUpcomingFootsteps)
         {
            setRemainingCoPsToDuplicateLastComputedCoP(0);
            return;
         }
      }
      else
      {
         copLocationWaypoints.get(footIndex).setToNaN(0);

         if (numberOfPointsPerFoot.getIntegerValue() > 2)
         {
            //todo make this the toe
            boolean isUpcomingFootstepLast = noUpcomingFootsteps;
            computeBallCoPForSupportFoot(tmpCoP, transferFromSide, supportFootPolygonsInSoleZUpFrames.get(transferToSide).getCentroid(), isUpcomingFootstepLast);
            tmpCoP.changeFrame(transferFromSoleFrame);
            copLocationWaypoints.get(footIndex).setIncludingFrame(2, tmpCoP);
         }
         else if (numberOfPointsPerFoot.getIntegerValue() > 1)
         {
            boolean isUpcomingFootstepLast = noUpcomingFootsteps;
            computeBallCoPForSupportFoot(tmpCoP, transferFromSide, supportFootPolygonsInSoleZUpFrames.get(transferToSide).getCentroid(), isUpcomingFootstepLast);
            tmpCoP.changeFrame(transferFromSoleFrame);
            copLocationWaypoints.get(footIndex).setIncludingFrame(1, tmpCoP);
         }
         else
         {
            computeHeelCoPForSupportFoot(tmpCoP, transferFromSide, null, null);
            tmpCoP.changeFrame(transferFromSoleFrame);
            copLocationWaypoints.get(footIndex).setIncludingFrame(0, tmpCoP);
         }

         footIndex++;
      }

      // compute the CoP points in the upcoming support foot
      computeHeelCoPForSupportFoot(tmpCoP, transferToSide, supportFootPolygonsInSoleZUpFrames.get(transferFromSide).getCentroid(), copLocationWaypoints.get(footIndex - 1).get(1));
      tmpCoP.changeFrame(transferToSoleFrame);
      copLocationWaypoints.get(footIndex).setIncludingFrame(0, tmpCoP);
      firstHeelCoPForSingleSupport.setByProjectionOntoXYPlaneIncludingFrame(tmpCoP);

      // compute the transfer CoP trajectory
      TransferCoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(0);
      FramePoint previousCoP = copLocationWaypoints.get(0).get(numberOfPointsPerFoot.getIntegerValue() - 1).getFrameTuple();
      FramePoint currentHeelCoP = copLocationWaypoints.get(1).get(0).getFrameTuple();
      previousCoP.changeFrame(worldFrame);
      currentHeelCoP.changeFrame(worldFrame);
      transferCoPTrajectory.compute(orderOfSplineInterpolation.getEnumValue(), previousCoP, currentHeelCoP);

      // compute upcoming support foot CoP locations
      computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootstepsData.get(0).getFootstep());
      boolean isUpcomingFootstepLast = upcomingFootstepsData.size() == 1;

      computeBallCoPForSupportFoot(tmpCoP, transferToSide, centroidOfUpcomingFootstep, isUpcomingFootstepLast);
      tmpCoP.changeFrame(transferToSoleFrame);
      copLocationWaypoints.get(footIndex).setIncludingFrame(1, tmpCoP);

      computeToeCoPForSupportFoot(tmpCoP, transferToSide);
      tmpCoP.changeFrame(transferToSoleFrame);
      copLocationWaypoints.get(footIndex).setIncludingFrame(2, tmpCoP);
      footIndex++;
      stepIndex++;

      // compute the CoP trajectory for the current swing segment
      SwingCoPTrajectory swingCoPTrajectory = swingCoPTrajectories.get(0);
      FramePoint currentBallCoP = copLocationWaypoints.get(1).get(1).getFrameTuple();
      FramePoint currentToeCoP = copLocationWaypoints.get(1).get(2).getFrameTuple();
      currentBallCoP.changeFrame(worldFrame);
      currentToeCoP.changeFrame(worldFrame);

      if (numberOfPointsPerFoot.getIntegerValue() > 2 && !currentToeCoP.containsNaN())
         swingCoPTrajectory.computeThreePointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP, currentToeCoP);
      else if (numberOfPointsPerFoot.getIntegerValue() > 1 && !currentBallCoP.containsNaN())
         swingCoPTrajectory.computeTwoPointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP);
      else
         swingCoPTrajectory.computeOnePointPerFoot(currentHeelCoP);

      // compute remaining steps
      computeReferenceCoPsWithUpcomingFootsteps(transferToSide, numberOfUpcomingFootsteps, footIndex, stepIndex);
      changeFrameOfCoPs(2, worldFrame);
   }

   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      int numberOfUpcomingFootsteps = upcomingFootstepsData.size();
      this.numberOfUpcomingFootsteps.set(numberOfUpcomingFootsteps);
      int constantCoPIndex = 0;
      int stepIndex = 0;
      boolean onlyOneUpcomingFootstep = numberOfUpcomingFootsteps == 1;
      isDoneWalking.set(onlyOneUpcomingFootstep);

      // Compute stance foot CoP waypoints

      // Can happen if this method is the first to be called, so it should pretty much never happen.
      if (firstHeelCoPForSingleSupport.containsNaN())
         computeHeelCoPForSupportFoot(tmpCoP, supportSide, null, null);
      else
         tmpCoP.setXYIncludingFrame(firstHeelCoPForSingleSupport);

      ReferenceFrame supportSoleFrame = soleZUpFrames.get(supportSide);
      tmpCoP.changeFrame(supportSoleFrame);
      copLocationWaypoints.get(constantCoPIndex).setIncludingFrame(0, tmpCoP);
      computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootstepsData.get(0).getFootstep());
      boolean isUpcomingFootstepLast = upcomingFootstepsData.size() == 1;

      computeBallCoPForSupportFoot(tmpCoP, supportSide, centroidOfUpcomingFootstep, isUpcomingFootstepLast);
      tmpCoP.changeFrame(supportSoleFrame);
      copLocationWaypoints.get(constantCoPIndex).setIncludingFrame(1, tmpCoP);

      computeToeCoPForSupportFoot(tmpCoP, supportSide);
      tmpCoP.changeFrame(supportSoleFrame);
      copLocationWaypoints.get(constantCoPIndex).setIncludingFrame(2, tmpCoP);

      // Compute swing CoP trajectory
      FramePoint currentHeelCoP = copLocationWaypoints.get(0).get(0).getFrameTuple();
      FramePoint currentBallCoP = copLocationWaypoints.get(0).get(1).getFrameTuple();
      FramePoint currentToeCoP = copLocationWaypoints.get(0).get(2).getFrameTuple();
      currentHeelCoP.changeFrame(worldFrame);
      currentBallCoP.changeFrame(worldFrame);
      currentToeCoP.changeFrame(worldFrame);

      SwingCoPTrajectory swingCoPTrajectory = swingCoPTrajectories.get(0);
      if (numberOfPointsPerFoot.getIntegerValue() > 2 && !currentToeCoP.containsNaN())
         swingCoPTrajectory.computeThreePointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP, currentToeCoP);
      else if (numberOfPointsPerFoot.getIntegerValue() > 1 && !currentBallCoP.containsNaN())
         swingCoPTrajectory.computeTwoPointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP);
      else
         swingCoPTrajectory.computeOnePointPerFoot(currentHeelCoP);

      constantCoPIndex++;
      stepIndex++;

      if (onlyOneUpcomingFootstep)
      {
         predictedSupportPolygon.clear(upcomingFootstepsData.get(0).getSoleReferenceFrame());
         addPredictedContactPointsToPolygon(upcomingFootstepsData.get(0).getFootstep(), predictedSupportPolygon);
         predictedSupportPolygon.update();
         computeFinalCoPBetweenSupportFeet(constantCoPIndex, supportFootPolygonsInSoleZUpFrames.get(supportSide), predictedSupportPolygon);
         setRemainingCoPsToDuplicateLastComputedCoP(constantCoPIndex);

         FramePoint previousCoP = copLocationWaypoints.get(constantCoPIndex - 1).get(numberOfPointsPerFoot.getIntegerValue() - 1).getFrameTuple();
         FramePoint heelCoP = copLocationWaypoints.get(constantCoPIndex).get(0).getFrameTuple();
         previousCoP.changeFrame(worldFrame);
         currentHeelCoP.changeFrame(worldFrame);

         TransferCoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(1);
         transferCoPTrajectory.compute(orderOfSplineInterpolation.getEnumValue(), previousCoP, heelCoP);

         return;
      }

      computeReferenceCoPsWithUpcomingFootsteps(supportSide, numberOfUpcomingFootsteps, constantCoPIndex, stepIndex);
      changeFrameOfCoPs(1, worldFrame);

      // set current trajectory
      activeTrajectory = swingCoPTrajectory;
   }

   private void computeReferenceCoPsWithUpcomingFootsteps(RobotSide firstSupportSide, int numberOfUpcomingFootsteps, int footIndex, int stepIndex)
   {
      FramePoint2d centroidInSoleFrameOfPreviousSupportFoot = supportFootPolygonsInSoleZUpFrames.get(firstSupportSide).getCentroid();

      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         Footstep currentFootstep = upcomingFootstepsData.get(i).getFootstep();
         computeFootstepCentroid(centroidOfCurrentFootstep, currentFootstep);

         FramePoint2d centroidOfNextFootstep = null;
         int indexOfUpcomingFootstep = i + 1;
         if (i < upcomingFootstepsData.size() - 1)
         {
            computeFootstepCentroid(centroidOfUpcomingFootstep, upcomingFootstepsData.get(indexOfUpcomingFootstep).getFootstep());
            centroidOfNextFootstep = centroidOfUpcomingFootstep;
         }

         boolean isUpcomingFootstepLast = (indexOfUpcomingFootstep >= Math.min(upcomingFootstepsData.size(), numberOfFootstepsToConsider.getIntegerValue()));
         if (isUpcomingFootstepLast)
         {
            // fixme this is predicting the final heel point as something incorrect
            predictedSupportPolygon.clear(currentFootstep.getSoleReferenceFrame());
            addPredictedContactPointsToPolygon(currentFootstep, predictedSupportPolygon);
            predictedSupportPolygon.update();
            computeFinalCoPBetweenSupportFeet(footIndex, supportFootPolygonsInSoleZUpFrames.get(firstSupportSide), predictedSupportPolygon);

            FramePoint previousCoP = copLocationWaypoints.get(footIndex - 1).get(numberOfPointsPerFoot.getIntegerValue() - 1).getFrameTuple();
            FramePoint currentHeelCoP = copLocationWaypoints.get(footIndex).get(0).getFrameTuple();
            previousCoP.changeFrame(worldFrame);
            currentHeelCoP.changeFrame(worldFrame);

            TransferCoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
            transferCoPTrajectory.compute(orderOfSplineInterpolation.getEnumValue(), previousCoP, currentHeelCoP);
         }
         else
         {
            // compute CoP waypoints
            computeToeCoPForFootstep(tmpCoP, currentFootstep);
            tmpCoP.changeFrame(soleZUpFrames.get(firstSupportSide));
            copLocationWaypoints.get(footIndex).setIncludingFrame(2, tmpCoP);

            computeBallCoPForFootstep(tmpCoP, currentFootstep, centroidOfNextFootstep, isUpcomingFootstepLast);
            tmpCoP.changeFrame(soleZUpFrames.get(firstSupportSide));
            copLocationWaypoints.get(footIndex).setIncludingFrame(1, tmpCoP);

            YoFramePoint previousLastCoP = copLocationWaypoints.get(footIndex - 1).get(numberOfPointsPerFoot.getIntegerValue() - 1);
            computeHeelCoPForFootstep(tmpCoP, currentFootstep, centroidInSoleFrameOfPreviousSupportFoot, previousLastCoP);
            tmpCoP.changeFrame(soleZUpFrames.get(firstSupportSide));
            copLocationWaypoints.get(footIndex).setIncludingFrame(0, tmpCoP);

            // compute CoP trajectories
            FramePoint previousCoP = copLocationWaypoints.get(footIndex - 1).get(numberOfPointsPerFoot.getIntegerValue() - 1).getFrameTuple();
            FramePoint currentHeelCoP = copLocationWaypoints.get(footIndex).get(0).getFrameTuple();
            FramePoint currentBallCoP = copLocationWaypoints.get(footIndex).get(1).getFrameTuple();
            FramePoint currentToeCoP = copLocationWaypoints.get(footIndex).get(2).getFrameTuple();
            previousCoP.changeFrame(worldFrame);
            currentHeelCoP.changeFrame(worldFrame);
            currentBallCoP.changeFrame(worldFrame);
            currentToeCoP.changeFrame(worldFrame);

            TransferCoPTrajectory transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
            transferCoPTrajectory.compute(orderOfSplineInterpolation.getEnumValue(), previousCoP, currentHeelCoP);

            SwingCoPTrajectory swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
            if (numberOfPointsPerFoot.getIntegerValue() > 2 && !currentToeCoP.containsNaN())
               swingCoPTrajectory.computeThreePointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP, currentToeCoP);
            else if (numberOfPointsPerFoot.getIntegerValue() > 1 && !currentBallCoP.containsNaN())
               swingCoPTrajectory.computeTwoPointsPerFoot(orderOfSplineInterpolation.getEnumValue(), currentHeelCoP, currentBallCoP);
            else
               swingCoPTrajectory.computeOnePointPerFoot(currentHeelCoP);
         }

         footIndex++;
         stepIndex++;
         centroidInSoleFrameOfPreviousSupportFoot = centroidOfCurrentFootstep;

         if (footIndex >= copLocationWaypoints.size())
            break;
      }
      setRemainingCoPsToDuplicateLastComputedCoP(footIndex - 1);
   }

   /**
    * Remove first footstep in the upcoming footstep queue from planner
    */
   public void removeFootstepQueueFront()
   {
      removeFootstep(0);
      numberOfUpcomingFootsteps.decrement();
   }

   /**
    * Removes the specified number of footsteps from the queue front
    * @param numberOfFootstepsToRemove number of steps to remove
    */

   public void removeFootstepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
      numberOfUpcomingFootsteps.decrement();
   }

   /**
    * Removes specified footstep from upcoming footstep queue
    * @param index
    */
   public void removeFootstep(int index)
   {
      upcomingFootstepsData.remove(index);
      numberOfUpcomingFootsteps.decrement();
   }

   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   public void setSafeDistanceFromSupportEdges(double distance)
   {
      safeDistanceFromCoPToSupportEdges.set(distance);
   }


   private void setRemainingCoPsToDuplicateLastComputedCoP(int lastComputedCoPIndex)
   {
      for (int footIndex = lastComputedCoPIndex + 1; footIndex < copLocationWaypoints.size(); footIndex++)
      {
         for (int waypointIndex = 0; waypointIndex < numberOfPointsPerFoot.getIntegerValue(); waypointIndex++)
            copLocationWaypoints.get(footIndex).setIncludingFrame(waypointIndex, copLocationWaypoints.get(lastComputedCoPIndex).get(waypointIndex));
      }
   }

   private void changeFrameOfCoPs(int fromIndex, ReferenceFrame desiredFrame)
   {
      for (int i = fromIndex; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).changeFrame(desiredFrame);
   }

   private void computeFootstepCentroid(FramePoint2d centroidToPack, Footstep footstep)
   {
      predictedSupportPolygon.clear(footstep.getSoleReferenceFrame());
      addPredictedContactPointsToPolygon(footstep, predictedSupportPolygon);
      predictedSupportPolygon.update();
      predictedSupportPolygon.getCentroid(centroidToPack);
   }

   private void addPredictedContactPointsToPolygon(Footstep footstep, FrameConvexPolygon2d convexPolygonToExtend)
   {
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();

      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         int numberOfContactPoints = predictedContactPoints.size();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            tempFramePoint.setXYIncludingFrame(footstep.getSoleReferenceFrame(), predictedContactPoints.get(i));
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
      else
      {
         ConvexPolygon2D defaultPolygon = defaultFootPolygons.get(footstep.getRobotSide());
         for (int i = 0; i < defaultPolygon.getNumberOfVertices(); i++)
         {
            tempFramePoint.setXYIncludingFrame(footstep.getSoleReferenceFrame(), defaultPolygon.getVertex(i));
            convexPolygonToExtend.addVertexByProjectionOntoXYPlane(tempFramePoint);
         }
      }
   }

   private FrameConvexPolygon2d getFootSupportPolygon(RobotSide side)
   {
      return supportFootPolygonsInSoleZUpFrames.get(side);
   }

   private FramePoint2d getFootSupportPolygonCentroid(RobotSide side)
   {
      return getFootSupportPolygon(side).getCentroid();
   }


   public List<CoPPointsInFoot> getWaypoints()
   {
      convertCoPWayPointsToWorldFrame();
      return copLocationWaypoints;
   }

   private void computeHeelCoPForSupportFoot(FramePoint copToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot,
                                              YoFramePoint previousLastCoP)
   {
      ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
      tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      tempSupportPolygon.changeFrame(soleFrame);

      computeHeelCoP(copToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfPreviousSupportFoot, previousLastCoP);
   }

   private void computeHeelCoPForFootstep(FramePoint heelCoPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfPreviousSupportFoot,
                                           YoFramePoint previousLastCoP)
   {
      ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      RobotSide robotSide = footstep.getRobotSide();

      if (predictedContactPoints != null)
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
      else
         tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

      computeHeelCoP(heelCoPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfPreviousSupportFoot, previousLastCoP);
   }

   private void computeHeelCoP(FramePoint heelCoPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon,
                                FramePoint2d centroidInSoleFrameOfPreviousSupportFoot, YoFramePoint previousLastCoP)
   {
      if (numberOfPointsPerFoot.getIntegerValue() > 1)
      {
         if (centroidInSoleFrameOfPreviousSupportFoot != null)
            centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfPreviousSupportFoot);
         else
            centroidOfFootstepToConsider.setToZero(soleFrame);
         centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

         if (previousLastCoP != null)
         {
            previousLastCoP.getFrameTuple2dIncludingFrame(previousCoP2d);
            previousCoP2d.changeFrameAndProjectToXYPlane(soleFrame);
            // Choose the laziest option
            if (Math.abs(previousCoP2d.getX()) < Math.abs(centroidOfFootstepToConsider.getX()))
               centroidOfFootstepToConsider.set(previousCoP2d);
         }

         constrainCoPAccordingToSupportPolygonAndUserOffsets(tmpCoP2d, footSupportPolygon, centroidOfFootstepToConsider, copUserOffsets.get(robotSide).get(0),
                                                             minCoPOffsets.get(0).getDoubleValue(), maxCoPOffsets.get(0).getDoubleValue());
      }
      else
      {
         tmpCoP2d.setIncludingFrame(footSupportPolygon.getCentroid());
         YoFrameVector2d offset = copUserOffsets.get(robotSide).get(0);
         tmpCoP2d.add(offset.getX(), offset.getY());
      }

      heelCoPToPack.setXYIncludingFrame(tmpCoP2d);
      heelCoPToPack.changeFrame(worldFrame);
   }

   private void computeBallCoPForSupportFoot(FramePoint ballCoPToPack, RobotSide robotSide, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot,
                                             boolean isUpcomingFootstepLast)
   {
      if (numberOfPointsPerFoot.getIntegerValue() > 1)
      {
         ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrames.get(robotSide));
         tempSupportPolygon.changeFrame(soleFrame);

         computeBallCoP(ballCoPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         ballCoPToPack.setToNaN(worldFrame);
      }
   }

   private void computeBallCoPForFootstep(FramePoint ballCoPToPack, Footstep footstep, FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot,
                                          boolean isUpcomingFootstepLast)
   {
      if (numberOfPointsPerFoot.getIntegerValue() > 1)
      {
         ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
         List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
         RobotSide robotSide = footstep.getRobotSide();

         if (predictedContactPoints != null)
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
         else
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

         computeBallCoP(ballCoPToPack, robotSide, soleFrame, tempSupportPolygon, centroidInSoleFrameOfUpcomingSupportFoot, isUpcomingFootstepLast);
      }
      else
      {
         ballCoPToPack.setToNaN(worldFrame);
      }

   }

   private void computeBallCoP(FramePoint ballCoPToPack, RobotSide robotSide, ReferenceFrame soleFrame, FrameConvexPolygon2d footSupportPolygon,
                               FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot, boolean isUpcomingFootstepLast)
   {
      if (centroidInSoleFrameOfUpcomingSupportFoot != null)
         centroidOfFootstepToConsider.setIncludingFrame(centroidInSoleFrameOfUpcomingSupportFoot);
      else
         centroidOfFootstepToConsider.setToZero(soleFrame);
      centroidOfFootstepToConsider.changeFrameAndProjectToXYPlane(soleFrame);

      boolean polygonIsAPoint = footSupportPolygon.getArea() == 0.0;
      //boolean putCoPOnToesWalking = false;
      //boolean putCoPOnToesSteppingDown = false;

      /*
      if (!isUpcomingFootstepLast && centroidInSoleFrameOfUpcomingSupportFoot != null && !polygonIsAPoint)
      {
         soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
         soleFrameOrigin.changeFrame(soleFrame);
         soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
         //boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue();
         soleToSoleFrameVector.changeFrame(worldFrame);
         //boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < -footstepHeightThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue();

         if (isSteppingDownEnough)
         {
            putCoPOnToesSteppingDown = isSteppingForwardEnough && isSteppingDownEnough;
         }
         else if (putExitCoPOnToes.getBooleanValue())
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);

            putCoPOnToesWalking = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCoPOnToes.getDoubleValue();
         }
      }
        */


      if (polygonIsAPoint)
      {
         tmpCoP2d.setToZero(footSupportPolygon.getReferenceFrame());
         tmpCoP2d.set(footSupportPolygon.getVertex(0));
      }
      /*
      else if (putCoPOnToesWalking)
      {
         //putExitCoPOnToes(tmpCoP2d, footSupportPolygon, exitCoPUserOffsets.get(robotSide).getY());
      }
      else if (putCoPOnToesSteppingDown)
      {
         //putExitCoPOnToes(tmpCoP2d, footSupportPolygon, 0.0);
      }
      */
      else
      {
         constrainCoPAccordingToSupportPolygonAndUserOffsets(tmpCoP2d, footSupportPolygon, centroidOfFootstepToConsider, copUserOffsets.get(robotSide).get(1),
                                                             minCoPOffsets.get(1).getDoubleValue(), maxCoPOffsets.get(1).getDoubleValue());
      }

      ballCoPToPack.setXYIncludingFrame(tmpCoP2d);
      ballCoPToPack.changeFrame(worldFrame);
   }

   private void computeToeCoPForSupportFoot(FramePoint toeCoPToPack, RobotSide robotSide)
   {
      if (numberOfPointsPerFoot.getIntegerValue() > 2)
      {
         ReferenceFrame soleFrame = soleZUpFrames.get(robotSide);
         tempSupportPolygon.setIncludingFrameAndUpdate(supportFootPolygonsInSoleZUpFrames.get(robotSide));
         tempSupportPolygon.changeFrame(soleFrame);

         computeToeCoP(toeCoPToPack);
      }
      else
      {
         toeCoPToPack.setToNaN(worldFrame);
      }
   }

   private void computeToeCoPForFootstep(FramePoint toeCoPToPack, Footstep footstep)
   {
      if (numberOfPointsPerFoot.getIntegerValue() > 1)
      {
         ReferenceFrame soleFrame = footstep.getSoleReferenceFrame();
         List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
         RobotSide robotSide = footstep.getRobotSide();

         if (predictedContactPoints != null)
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, predictedContactPoints);
         else
            tempSupportPolygon.setIncludingFrameAndUpdate(soleFrame, defaultFootPolygons.get(robotSide));

         computeToeCoP(toeCoPToPack);
      }
      else
      {
         toeCoPToPack.setToNaN(worldFrame);
      }

   }

   private void computeToeCoP(FramePoint toeCoPToPack)
   {
      toeCoPToPack.setToNaN(worldFrame);
   }

   private void constrainCoPAccordingToSupportPolygonAndUserOffsets(FramePoint2d copToPack, FrameConvexPolygon2d footSupportPolygon,
                                                                    FramePoint2d centroidOfFootstepToConsider, YoFrameVector2d copOffset,
                                                                    double minForwardCoPOffset, double maxForwardCoPOffset)
   {
      // First constrain the computed CoP to the given min/max along the x-axis.
      FramePoint2d footSupportCentroid = footSupportPolygon.getCentroid();
      double copXOffsetFromCentroid = stepLengthToCoPOffsetFactor.getDoubleValue() * (centroidOfFootstepToConsider.getX() - footSupportCentroid.getX()) + copOffset.getX();
      copXOffsetFromCentroid = MathTools.clamp(copXOffsetFromCentroid, minForwardCoPOffset, maxForwardCoPOffset);

      copToPack.setIncludingFrame(footSupportCentroid);
      copToPack.add(copXOffsetFromCentroid, copOffset.getY());

      // Then constrain the computed CoP to be inside a safe support region
      tempSupportPolygonForShrinking.setIncludingFrameAndUpdate(footSupportPolygon);
      convexPolygonShrinker.shrinkConstantDistanceInto(tempSupportPolygonForShrinking, safeDistanceFromCoPToSupportEdges.getDoubleValue(), footSupportPolygon);

      footSupportPolygon.orthogonalProjection(copToPack);
   }

   private final FramePoint2d tempCentroid = new FramePoint2d();
   private final FramePoint tempCentroid3d = new FramePoint();
   private final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d upcomingSupport = new FrameConvexPolygon2d();
   private void computeFinalCoPBetweenSupportFeet(int footIndex, FrameConvexPolygon2d footA, FrameConvexPolygon2d footB)
   {
      footA.getCentroid(tempCentroid);
      firstCoP.setXYIncludingFrame(tempCentroid);
      firstCoP.changeFrame(worldFrame);

      footB.getCentroid(tempCentroid);
      secondCoP.setXYIncludingFrame(tempCentroid);
      secondCoP.changeFrame(worldFrame);

      upcomingSupport.clear(worldFrame);
      tempFootPolygon.setIncludingFrame(footA);
      tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      upcomingSupport.addVertices(tempFootPolygon);
      tempFootPolygon.setIncludingFrame(footB);
      tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      upcomingSupport.addVertices(tempFootPolygon);
      upcomingSupport.update();

      copLocationWaypoints.get(footIndex).switchCurrentReferenceFrame(0, worldFrame);

      upcomingSupport.getCentroid(tempCentroid);
      tempCentroid3d.setXYIncludingFrame(tempCentroid);

      double chicken = MathTools.clamp(percentageChickenSupport.getDoubleValue(), 0.0, 1.0);
      tmpCoP.setToZero(worldFrame);
      if (chicken <= 0.5)
         copLocationWaypoints.get(footIndex).get(0).interpolate(firstCoP, tempCentroid3d, chicken * 2.0);
      else
         copLocationWaypoints.get(footIndex).get(0).interpolate(tempCentroid3d, secondCoP, (chicken - 0.5) * 2.0);
   }

   private void convertCoPWayPointsToWorldFrame()
   {
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).changeFrame(worldFrame);
   }

   @Override
   public void setInitialCoPPosition(FramePoint initialCoPPosition)
   {
      currentCoPPosition.setIncludingFrame(initialCoPPosition);
   }

   @Override
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition)
   {
      currentCoPPosition.setToZero(initialCoPPosition.getReferenceFrame());
      currentCoPPosition.setXY(initialCoPPosition);
   }

   @Override
   public void setInitialCoPVelocity(FrameVector initialCoPVelocity)
   {
      currentCoPVelocity.setIncludingFrame(initialCoPVelocity);
   }

   @Override
   public void setInitialCoPVelocity(FrameVector2d initialCoPVelocity)
   {
      currentCoPVelocity.setToZero(initialCoPVelocity.getReferenceFrame());
      currentCoPVelocity.setXY(initialCoPVelocity);
   }


   @Override
   public void setInitialCoPAcceleration(FrameVector initialCoPAcceleration)
   {
      currentCoPAcceleration.setIncludingFrame(initialCoPAcceleration);
   }

   @Override
   public void setInitialCoPAcceleration(FrameVector2d initialCoPAccel)
   {
      currentCoPAcceleration.setToZero(initialCoPAccel.getReferenceFrame());
      currentCoPAcceleration.setXY(initialCoPAccel);
   }

   @Override
   public void setFinalCoPVelocity(FrameVector finalCoPVelocity)
   {
      this.finalCoPVelocity.setIncludingFrame(finalCoPVelocity);
   }

   @Override
   public void setFinalCoPVelocity(FrameVector2d finalCoPVelocity)
   {
      this.finalCoPVelocity.setToZero(finalCoPVelocity.getReferenceFrame());
      this.finalCoPVelocity.setXY(finalCoPVelocity);
   }
}
