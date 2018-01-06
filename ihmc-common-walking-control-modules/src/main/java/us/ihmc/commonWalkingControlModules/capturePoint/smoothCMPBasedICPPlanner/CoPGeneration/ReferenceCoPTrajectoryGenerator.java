package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.CoPPointPlanningParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

//TODO 1) add isDoneWalking functionality

public class ReferenceCoPTrajectoryGenerator implements ReferenceCoPTrajectoryGeneratorInterface
{
   private static final int maxNumberOfCoPWaypoints = 20;
   private final boolean debug;

   public enum UseSplitFractionFor
   {
      POSITION, TIME
   };

   private final String fullPrefix;
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double COP_POINT_SIZE = 0.005;

   private static final int numberOfSwingSegments = 3;
   private static final int numberOfTransferSegments = 2;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // Waypoint planning parameters
   private double defaultSwingTime;
   private double defaultTransferTime;

   private final EnumMap<CoPPointName, CoPPointPlanningParameters> copPointParametersMap = new EnumMap<>(CoPPointName.class);

   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble safeDistanceFromCoPToSupportEdgesWhenSteppingDown;
   private final YoDouble footstepHeightThresholdToPutExitCoPOnToesSteppingDown;
   private final YoDouble footstepLengthThresholdToPutExitCoPOnToesSteppingDown;
   private final YoDouble footstepLengthThresholdToPutExitCoPOnToes;
   private final YoDouble exitCoPForwardSafetyMarginOnToes;
   private final YoDouble percentageChickenSupport;
   private CoPPointName entryCoPName;
   private CoPPointName exitCoPName;
   private CoPPointName endCoPName;
   private CoPPointName[] transferCoPPointList;
   private CoPPointName[] swingCoPPointList;
   private final YoDouble additionalTimeForFinalTransfer;

   // State variables 
   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberOfPointsPerFoot;
   private final YoInteger numberOfUpcomingFootsteps;
   private final YoInteger numberFootstepsToConsider;

   private final List<YoDouble> swingDurations;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> touchdownDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingDurationShiftFractions;
   private final List<YoDouble> transferSplitFractions;
   private final List<UseSplitFractionFor> useTransferSplitFractionFor;

   private final YoEnum<CoPSplineType> orderOfSplineInterpolation;

   private final YoBoolean isDoneWalking;
   private final YoBoolean holdDesiredState;
   private final YoBoolean putExitCoPOnToes;
   private final YoBoolean planIsAvailable;

   // Output variables
   private final List<CoPPointsInFoot> copLocationWaypoints = new ArrayList<>();
   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   // Runtime variables
   private final FramePoint3D desiredCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();
   private final FramePoint3D heldCoPPosition = new FramePoint3D();

   private int plannedFootstepIndex = -1;
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FramePoint3D tempDoubleSupportPolygonCentroid = new FramePoint3D();
   private FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootPredictedFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private final ConvexPolygon2D polygonReference = new ConvexPolygon2D();
   private final FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint2D tempFramePoint2d = new FramePoint2D();
   private final FramePoint3D tempPointForCoPCalculation = new FramePoint3D();

   // Planner level overrides (the planner knows better!)
   private static final int maxNumberOfFootstepsToConsider = 4;

   // Input data
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<FootstepData>(maxNumberOfFootstepsToConsider,
                                                                                                               FootstepData.class);

   // Visualization 
   private final List<YoFramePoint> copWaypointsViz = new ArrayList<>(maxNumberOfCoPWaypoints);

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCoPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> touchdownDurations, List<YoDouble> swingSplitFractions,
                                          List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet, numberFootstepsToConsider, swingDurations, transferDurations,
           touchdownDurations, swingSplitFractions, swingDurationShiftFractions, transferSplitFractions, false, parentRegistry);

   }

   public ReferenceCoPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> touchdownDurations, List<YoDouble> swingSplitFractions,
                                          List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions, boolean debug, YoVariableRegistry parentRegistry)
   {
      this.numberFootstepsToConsider = numberFootstepsToConsider;
      this.fullPrefix = namePrefix + "CoPTrajectoryGenerator";
      this.debug = debug;
      additionalTimeForFinalTransfer = new YoDouble(fullPrefix + "AdditionalTimeForFinalTransfer", registry);
      safeDistanceFromCoPToSupportEdges = new YoDouble(fullPrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(fullPrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      footstepHeightThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(fullPrefix + "FootstepHeightThresholdToPutExitCoPOnToesSteppingDown",
                                                                           parentRegistry);
      footstepLengthThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(fullPrefix + "FootstepLengthThresholdToPutExitCoPOnToesSteppingDown",
                                                                           parentRegistry);
      footstepLengthThresholdToPutExitCoPOnToes = new YoDouble(fullPrefix + "FootstepLengthThresholdToPutExitCoPOnToes", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(fullPrefix + "ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageChickenSupport = new YoDouble(fullPrefix + "PercentageChickenSupport", registry);
      numberOfUpcomingFootsteps = new YoInteger(fullPrefix + "NumberOfUpcomingFootsteps", registry);

      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.touchdownDurations = touchdownDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;
      this.useTransferSplitFractionFor = new ArrayList<>(transferSplitFractions.size());
      for (int i = 0; i < transferSplitFractions.size(); i++)
         useTransferSplitFractionFor.add(UseSplitFractionFor.TIME);

      this.numberOfPointsPerFoot = new YoInteger(fullPrefix + "NumberOfPointsPerFootstep", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(fullPrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.isDoneWalking = new YoBoolean(fullPrefix + "IsDoneWalking", registry);
      this.holdDesiredState = new YoBoolean(fullPrefix + "HoldDesiredState", parentRegistry);
      this.putExitCoPOnToes = new YoBoolean(fullPrefix + "PutExitCoPOnToes", parentRegistry);
      this.planIsAvailable = new YoBoolean(fullPrefix + "CoPPlanAvailable", parentRegistry);

      for (CoPPointName pointName : CoPPointName.values)
      {
         CoPPointPlanningParameters copParameters = new CoPPointPlanningParameters(pointName);

         YoDouble maxCoPOffset = new YoDouble(fullPrefix + "maxCoPForwardOffset" + pointName.toString(), registry);
         YoDouble minCoPOffset = new YoDouble(fullPrefix + "minCoPForwardOffset" + pointName.toString(), registry);

         copParameters.setCoPOffsetBounds(minCoPOffset, maxCoPOffset);
         copPointParametersMap.put(pointName, copParameters);

         for (RobotSide robotSide : RobotSide.values)
         {
            String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
            YoFrameVector2d copUserOffset = new YoFrameVector2d(fullPrefix + sidePrefix + "CoPConstantOffset" + pointName.toString(), null, registry);
            copParameters.setCoPOffsets(robotSide, copUserOffset);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         supportFootPolygonsInSoleZUpFrames.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));
      }

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(orderOfSplineInterpolation.getEnumValue(), numberOfTransferSegments);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(orderOfSplineInterpolation.getEnumValue(), numberOfSwingSegments);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(orderOfSplineInterpolation.getEnumValue(), numberOfTransferSegments);
      transferCoPTrajectories.add(transferCoPTrajectory);

      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};

      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(fullPrefix, i, framesToRegister, registry));
      }
      copLocationWaypoints.add(new CoPPointsInFoot(fullPrefix, maxNumberOfFootstepsToConsider + 1, framesToRegister, registry));
      copLocationWaypoints.add(new CoPPointsInFoot(fullPrefix, maxNumberOfFootstepsToConsider + 2, framesToRegister, registry));

      parentRegistry.addChild(registry);
      clear();
   }

   public void setDefaultPhaseTimes(double defaultSwingTime, double defaultTransferTime)
   {
      this.defaultSwingTime = defaultSwingTime;
      this.defaultTransferTime = defaultTransferTime;
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters parameters)
   {
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());
      numberOfPointsPerFoot.set(parameters.getNumberOfCoPWayPointsPerFoot());
      orderOfSplineInterpolation.set(parameters.getOrderOfCoPInterpolation());
      percentageChickenSupport.set(0.5);

      EnumMap<CoPPointName, CoPSupportPolygonNames> copSupportPolygon = parameters.getSupportPolygonNames();
      EnumMap<CoPPointName, Boolean> isConstrainedToSupportPolygonFlags = parameters.getIsConstrainedToSupportPolygonFlags();
      EnumMap<CoPPointName, Boolean> isConstrainedToMinMaxFlags = parameters.getIsConstrainedToMinMaxFlags();
      EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactors = parameters.getStepLengthToCoPOffsetFactors();
      EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetReferencePolygons = parameters.getStepLengthCoPOffsetPolygons();

      for (CoPPointName copPointName : CoPPointName.values)
      {
         CoPPointPlanningParameters copPointParameters = copPointParametersMap.get(copPointName);
         if (copSupportPolygon.containsKey(copPointName))
            copPointParameters.setSupportPolygonName(copSupportPolygon.get(copPointName));
         if (isConstrainedToSupportPolygonFlags.containsKey(copPointName))
            copPointParameters.setIsConstrainedToSupportPolygon(isConstrainedToSupportPolygonFlags.get(copPointName));
         if (isConstrainedToMinMaxFlags.containsKey(copPointName))
            copPointParameters.setIsConstrainedToMinMax(isConstrainedToMinMaxFlags.get(copPointName));
         if (stepLengthToCoPOffsetFactors.containsKey(copPointName))
            copPointParameters.setStepLengthToCoPOffsetFactor(stepLengthToCoPOffsetFactors.get(copPointName));
         if (stepLengthOffsetReferencePolygons.containsKey(copPointName))
            copPointParameters.setStepLengthOffsetPolygon(stepLengthOffsetReferencePolygons.get(copPointName));
      }

      this.entryCoPName = parameters.getEntryCoPName();
      this.exitCoPName = parameters.getExitCoPName();
      this.endCoPName = parameters.getEndCoPName();

      this.transferCoPPointList = parameters.getTransferCoPPointsToPlan();
      this.swingCoPPointList = parameters.getSwingCoPPointsToPlan();

      this.footstepHeightThresholdToPutExitCoPOnToesSteppingDown.set(parameters.getStepHeightThresholdForExitCoPOnToesWhenSteppingDown());
      this.footstepLengthThresholdToPutExitCoPOnToesSteppingDown.set(parameters.getStepLengthThresholdForExitCoPOnToesWhenSteppingDown());
      this.safeDistanceFromCoPToSupportEdgesWhenSteppingDown.set(parameters.getCoPSafeDistanceAwayFromToesWhenSteppingDown());

      this.footstepLengthThresholdToPutExitCoPOnToes.set(parameters.getStepLengthThresholdForExitCoPOnToes());
      this.putExitCoPOnToes.set(parameters.putExitCoPOnToes());
      this.exitCoPForwardSafetyMarginOnToes.set(parameters.getExitCoPForwardSafetyMarginOnToes());

      EnumMap<CoPPointName, Vector2D> copOffsets = parameters.getCoPOffsetsInFootFrame();
      EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds = parameters.getCoPForwardOffsetBoundsInFoot();
      CoPPointName[] transferCoPNames = parameters.getTransferCoPPointsToPlan();
      CoPPointName[] swingCoPNames = parameters.getSwingCoPPointsToPlan();
      for (CoPPointName transferCoPName : transferCoPNames)
      {
         setSymmetricCoPConstantOffsets(transferCoPName, copOffsets.get(transferCoPName));

         Vector2D bounds = copForwardOffsetBounds.get(transferCoPName);
         copPointParametersMap.get(transferCoPName).getMinCoPOffset().set(bounds.getX());
         copPointParametersMap.get(transferCoPName).getMaxCoPOffset().set(bounds.getY());
      }
      for (CoPPointName swingCoPName : swingCoPNames)
      {
         setSymmetricCoPConstantOffsets(swingCoPName, copOffsets.get(swingCoPName));

         Vector2D bounds = copForwardOffsetBounds.get(swingCoPName);
         copPointParametersMap.get(swingCoPName).getMinCoPOffset().set(bounds.getX());
         copPointParametersMap.get(swingCoPName).getMaxCoPOffset().set(bounds.getY());
      }
   }

   public void holdPosition(FramePoint3D desiredCoPPositionToHold)
   {
      holdDesiredState.set(true);
      heldCoPPosition.setIncludingFrame(desiredCoPPositionToHold);
   }

   public void holdPosition()
   {
      holdPosition(desiredCoPPosition);
   }

   public void clearHeldPosition()
   {
      holdDesiredState.set(false);
      heldCoPPosition.setToNaN(worldFrame);
   }

   @Override
   public void setSymmetricCoPConstantOffsets(CoPPointName copPointName, Vector2D offset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copPointParametersMap.get(copPointName).getCoPOffsets(robotSide);
         copUserOffset.setX(offset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(offset.getY()));
      }
   }

   @Override
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      if (debug)
      {
         for (int footIndex = 0; footIndex < copLocationWaypoints.size(); footIndex++)
         {
            CoPPointsInFoot copPointsInFoot = copLocationWaypoints.get(footIndex);
            copPointsInFoot.setupVisualizers(yoGraphicsList, artifactList, COP_POINT_SIZE);
         }
      }
      for (int waypointIndex = 0; waypointIndex < maxNumberOfCoPWaypoints; waypointIndex++)
      {
         YoFramePoint yoCoPWaypoint = new YoFramePoint("CoPWaypointAfterAdjustment" + waypointIndex, worldFrame, registry);
         YoGraphicPosition copWaypointViz = new YoGraphicPosition("AdjustedCoPWaypointViz" + waypointIndex, yoCoPWaypoint, COP_POINT_SIZE,
                                                                  YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
         yoCoPWaypoint.setToNaN();
         yoGraphicsList.add(copWaypointViz);
         artifactList.add(copWaypointViz.createArtifact());
         copWaypointsViz.add(yoCoPWaypoint);
      }
   }

   @Override
   public void updateListeners()
   {
      if (debug)
      {
         for (int i = 0; i < copLocationWaypoints.size(); i++)
            copLocationWaypoints.get(i).notifyVariableChangedListeners();
      }

      updateAdjustedCoPViz();
   }

   private void updateAdjustedCoPViz()
   {
      int transferTrajectoryIndex = 0;
      int swingTrajectoryIndex = 0;
      int waypointIndex = 0;
      int additionalTransferIndex = 0;
      for (int i = 0; waypointIndex < copWaypointsViz.size() && i < copLocationWaypoints.size() && !copLocationWaypoints.get(i).getCoPPointList().isEmpty(); i++)
      {
         CoPPointsInFoot copPointsInFoot = copLocationWaypoints.get(i);
         List<CoPPointName> copPointNames = copPointsInFoot.getCoPPointList();
         int transferEndIndex = CoPPlanningTools.getCoPPointIndex(copPointNames, entryCoPName);
         if (transferEndIndex == -1)
         {
            transferEndIndex = copPointNames.size();
            if (copPointNames.get(transferEndIndex - 1) == CoPPointName.FINAL_COP)
               transferEndIndex--;
         }
         CoPTrajectory transferTrajectory = transferCoPTrajectories.get(transferTrajectoryIndex);
         int j = 0;
         for (j = 0; j + additionalTransferIndex < transferTrajectory.getNumberOfSegments(); j++)
         {
            transferTrajectory.getSegment(j + additionalTransferIndex).getFramePositionInitial(tempFramePoint1);
            copWaypointsViz.get(waypointIndex).set(tempFramePoint1);
            copWaypointsViz.get(waypointIndex++).notifyVariableChangedListeners();
         }
         additionalTransferIndex = j;
         if (transferEndIndex == copPointNames.size())
            continue;
         transferTrajectoryIndex++;

         CoPTrajectory swingTrajectory = swingCoPTrajectories.get(swingTrajectoryIndex);
         for (j = 0; j < swingTrajectory.getNumberOfSegments(); j++)
         {
            swingTrajectory.getSegment(j).getFramePositionInitial(tempFramePoint1);
            copWaypointsViz.get(waypointIndex).set(tempFramePoint1);
            copWaypointsViz.get(waypointIndex++).notifyVariableChangedListeners();
         }
         additionalTransferIndex = 0;
         swingTrajectoryIndex++;
      }
   }

   @Override
   public void clear()
   {
      upcomingFootstepsData.clear();
      numberOfUpcomingFootsteps.set(0);
      desiredCoPPosition.setToNaN();
      desiredCoPVelocity.setToNaN();
      desiredCoPAcceleration.setToNaN();
      clearPlan();
   }

   public void clearPlan()
   {
      planIsAvailable.set(false);
      plannedFootstepIndex = -1;
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).reset();

      for (int i = 0; i < transferCoPTrajectories.size(); i++)
         transferCoPTrajectories.get(i).reset();
      for (int i = 0; i < swingCoPTrajectories.size(); i++)
         swingCoPTrajectories.get(i).reset();
   }

   @Override
   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getIntegerValue();
   }

   public Footstep getFootstep(int footstepIndex)
   {
      return upcomingFootstepsData.get(footstepIndex).getFootstep();
   }

   @Override
   public void initializeForTransfer(double currentTime)
   {
      this.initialTime = currentTime;
      this.activeTrajectory = transferCoPTrajectories.get(0);
   }

   @Override
   public void initializeForSwing(double currentTime)
   {
      clearHeldPosition();
      this.initialTime = currentTime;
      this.activeTrajectory = swingCoPTrajectories.get(0);
   }

   @Override
   public void update(double currentTime)
   {
      if (!planIsAvailable.getBooleanValue())
      {
         int footstepIndex = 0;
         initializeFootPolygons(RobotSide.LEFT, footstepIndex);
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, swingFootInitialPolygon, supportFootPolygon, worldFrame);
         desiredCoPPosition.setIncludingFrame(tempDoubleSupportPolygonCentroid);
         desiredCoPVelocity.setToZero(worldFrame);
         desiredCoPAcceleration.setToZero(worldFrame);
      }
      else if (activeTrajectory != null)
      {
         activeTrajectory.update(currentTime - this.initialTime, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
      }
      else
         throw new RuntimeException("CoP Planner: Plan available but no active trajectory initialized");
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint3D desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack)
   {
      getDesiredCenterOfPressure(desiredCoPToPack);
      desiredCoPVelocityToPack.setIncludingFrame(desiredCoPVelocity);
   }

   @Override
   public void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack)
   {
      desiredCoPToPack.set(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(YoFramePoint desiredCoPToPack, YoFrameVector desiredCoPVelocityToPack)
   {
      getDesiredCenterOfPressure(desiredCoPToPack);
      desiredCoPVelocityToPack.set(desiredCoPVelocity);
   }

   @Override
   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      int footstepIndex = 0;
      int copLocationIndex = 1;
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(footstepIndex);
      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop && numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         isDoneWalking.set(true);

         if (holdDesiredState.getBooleanValue())
         {
            tempPointForCoPCalculation.setIncludingFrame(heldCoPPosition);
            clearHeldPosition();
         }
         else
         {
            initializeFootPolygons(transferToSide, footstepIndex);
            getDoubleSupportPolygonCentroid(tempPointForCoPCalculation, supportFootPolygon, swingFootInitialPolygon, worldFrame);
         }
         copLocationWaypoint.addAndSetIncludingFrame(CoPPointName.START_COP, 0.0, tempPointForCoPCalculation);

         // set swing parameters for angular momentum estimation
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSupportFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSwingFootLocation(tempFramePoint1);

         // this is the last step
         computeCoPPointsForFinalTransfer(copLocationIndex, transferToSide, true, footstepIndex);
      }
      else if (atAStop)
      { // plan the whole thing, starting from rest
         isDoneWalking.set(false);
         updateFootPolygons(null, footstepIndex);

         // compute initial waypoint
         if (holdDesiredState.getBooleanValue())
         {
            tempPointForCoPCalculation.setIncludingFrame(heldCoPPosition);
            clearHeldPosition();
         }
         else
         {
            computeMidFeetPointWithChickenSupportForInitialTransfer(tempPointForCoPCalculation);
         }
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoint.addAndSetIncludingFrame(CoPPointName.START_COP, 0.0, tempPointForCoPCalculation);

         // set swing parameters for angular momentum estimation
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

         // compute all the upcoming footsteps
         computeCoPPointsForUpcomingFootsteps(copLocationIndex, footstepIndex);
      }
      else if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         // Put first CoP at the exitCoP of the swing foot if not starting from rest
         clearHeldPosition();
         isDoneWalking.set(true);
         initializeFootPolygons(transferToSide.getOppositeSide(), footstepIndex);

         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(exitCoPName), transferToSide.getOppositeSide(), footstepIndex);
         copLocationWaypoint.addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);

         // set swing parameters for angular momentum estimation
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSupportFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSwingFootLocation(tempFramePoint1);

         // compute all the upcoming footsteps
         computeCoPPointsForFinalTransfer(copLocationIndex, transferToSide, true, footstepIndex);
      }
      else
      {
         clearHeldPosition();
         isDoneWalking.set(false);
         updateFootPolygons(null, footstepIndex);

         // compute starting cop
         computeCoPPointLocationForPreviousPlan(tempPointForCoPCalculation, copPointParametersMap.get(exitCoPName),
                                                upcomingFootstepsData.get(footstepIndex).getSwingSide(), footstepIndex);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoint.addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);

         // set swing parameters for angular momentum estimation
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

         // compute all the upcoming footsteps
         computeCoPPointsForUpcomingFootsteps(copLocationIndex, footstepIndex);
      }

      // generate the cop trajectory
      generateCoPTrajectoriesFromWayPoints();
      planIsAvailable.set(true);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      clearHeldPosition();
      int footstepIndex = 0;
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(footstepIndex);
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         isDoneWalking.set(true);
         return;
      }
      else
      {
         updateFootPolygons(null, footstepIndex);
         isDoneWalking.set(false);

         // compute cop waypoint location
         computeCoPPointLocationForPreviousPlan(tempPointForCoPCalculation, copPointParametersMap.get(exitCoPName), upcomingFootstepData.getSwingSide(),
                                                footstepIndex);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoint.addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);

         // set swing foot parameters for angular momentum estimation
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

         int copLocationIndex = footstepIndex + 1;
         if (upcomingFootstepData.getSwingTime() == Double.POSITIVE_INFINITY)
         { // We're in flamingo support, so only do the current one
            upcomingFootstepData.setSwingTime(defaultSwingTime);
            computeCoPPointsForFlamingoStance(copLocationIndex, footstepIndex);
         }
         else
         { // Compute all the upcoming waypoints
            computeCoPPointsForUpcomingFootsteps(copLocationIndex, footstepIndex);
         }
      }

      // generate the cop trajectory
      generateCoPTrajectoriesFromWayPoints();
      planIsAvailable.set(true);
   }

   private void computeMidFeetPointWithChickenSupportForInitialTransfer(FramePoint3D framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, swingFootInitialPolygon, supportFootPolygon);
   }

   private void computeMidFeetPointWithChickenSupportForFinalTransfer(FramePoint3D framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, supportFootPolygon, swingFootPredictedFinalPolygon);
   }

   /**
    * Assumes foot polygon and double support polygon has been updated
    * @param framePointToPack
    * @param transferToSide
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint3D framePointToPack, FrameConvexPolygon2d supportFootPolygon,
                                                      FrameConvexPolygon2d swingFootPolygon)
   {
      computeMidFeetPointByPositionFraction(framePointToPack, supportFootPolygon, swingFootPolygon, percentageChickenSupport.getDoubleValue(),
                                            supportFootPolygon.getReferenceFrame());
   }

   private void computeMidFeetPointByPositionFraction(FramePoint3D framePointToPack, FrameConvexPolygon2d footPolygonA, FrameConvexPolygon2d footPolygonB,
                                                      double fraction, ReferenceFrame referenceFrameToConvertTo)
   {
      fraction = MathTools.clamp(fraction, 0.0, 1.0);
      FrameConvexPolygon2d framePolygonReference;
      if (fraction < 0.5)
      {
         fraction *= 2.0;
         framePolygonReference = footPolygonA;
      }
      else
      {
         fraction = (fraction - 0.5) * 2.0;
         framePolygonReference = footPolygonB;
      }
      getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, footPolygonA, footPolygonB, referenceFrameToConvertTo);
      convertToFramePointRetainingZ(tempFramePoint1, framePolygonReference.getCentroid(), referenceFrameToConvertTo);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
      framePointToPack.interpolate(this.tempDoubleSupportPolygonCentroid, tempFramePoint1, fraction);
   }

   private void computeCoPPointLocationForPreviousPlan(FramePoint3D framePointToPack, CoPPointPlanningParameters copPointParameters, RobotSide swingSide,
                                                       int footstepIndex)
   {
      CoPPointName copPointName = copPointParameters.getCopPointName();

      if (copPointName == exitCoPName && setInitialExitCoPUnderSpecialCases(framePointToPack, swingSide))
         return;
      setInitialCoPPointToPolygonOrigin(framePointToPack, copPointParameters.getSupportPolygonName(), copPointName, footstepIndex);
      YoFrameVector2d copOffset = copPointParameters.getCoPOffsets(swingSide);
      double copXOffset = copOffset.getX()
            + getInitialStepLengthToCoPOffset(copPointParameters.getStepLengthOffsetPolygon(), copPointParameters.getStepLengthToCoPOffsetFactor());

      if (copPointParameters.getIsConstrainedToMinMax())
         copXOffset = MathTools.clamp(copXOffset, copPointParameters.getMinCoPOffset().getDoubleValue(), copPointParameters.getMaxCoPOffset().getDoubleValue());
      framePointToPack.add(copXOffset, copOffset.getY(), 0.0);
      if (copPointParameters.getIsConstrainedToSupportPolygon())
         constrainInitialCoPPointToSupportPolygon(framePointToPack, copPointParameters.getSupportPolygonName());
      framePointToPack.changeFrame(worldFrame);
   }

   private double getInitialStepLengthToCoPOffset(CoPSupportPolygonNames stepLengthOffsetPolygon, double stepLengthToCoPOffsetFactor)
   {
      switch (stepLengthOffsetPolygon)
      {
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain given the " + stepLengthOffsetPolygon);
      case FINAL_SWING_POLYGON:
         supportFootPolygon.getFrameVertex(supportFootPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, swingFootInitialPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(swingFootInitialPolygon, tempFramePoint1, stepLengthToCoPOffsetFactor);
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, swingFootInitialPolygon, supportFootPolygon,
                                         swingFootInitialPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(swingFootInitialPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
      case SUPPORT_FOOT_POLYGON:
      case NULL:
      default:
         return 0.0;
      }
   }

   private void constrainInitialCoPPointToSupportPolygon(FramePoint3D copPointToPlan, CoPSupportPolygonNames copSupportPolygon)
   {
      switch (copSupportPolygon)
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         constrainToPolygon(copPointToPlan, supportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         //TODO
         throw new RuntimeException("Not implemented yet");
         //getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, swingFootInitialPolygon, supportFootPolygon, );
         //constrainToPolygon(copPointToPlan, tempDoubleSupportPolygonCentroid, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         //return;
      case SUPPORT_FOOT_POLYGON:
      default:
         constrainToPolygon(copPointToPlan, swingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
      }
   }

   private void setInitialCoPPointToPolygonOrigin(FramePoint3D copPointToPlan, CoPSupportPolygonNames copSupportPolygon, CoPPointName copPointName,
                                                  int footstepIndex)
   {
      switch (copSupportPolygon)
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to compute initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         convertToFramePointRetainingZ(copPointToPlan, supportFootPolygon.getCentroid(), swingFootInitialPolygon.getReferenceFrame());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointByPositionFraction(copPointToPlan, swingFootInitialPolygon, supportFootPolygon,
                                                  transferSplitFractions.get(footstepIndex).getDoubleValue(), swingFootInitialPolygon.getReferenceFrame());
         else
            getDoubleSupportPolygonCentroid(copPointToPlan, swingFootInitialPolygon, supportFootPolygon, swingFootInitialPolygon.getReferenceFrame());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         convertToFramePointRetainingZ(copPointToPlan, swingFootInitialPolygon.getCentroid(), swingFootInitialPolygon.getReferenceFrame());
      }
   }

   private void computeCoPPointsForUpcomingFootsteps(int copLocationIndex, int footstepIndex)
   {
      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getIntegerValue());
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         updateFootPolygons(upcomingFootstepData.getSupportSide(), footstepIndex);
         computeCoPPointsForFootstep(copLocationIndex, footstepIndex);
         footstepIndex++;
         copLocationIndex++;
      }

      boolean isLastTransfer = numberOfUpcomingFootsteps < numberFootstepsToConsider.getIntegerValue();
      computeCoPPointsForFinalTransfer(copLocationIndex, upcomingFootstepsData.get(footstepIndex - 1).getSupportSide(), isLastTransfer, footstepIndex);
   }

   private void computeCoPPointsForFinalTransfer(int copLocationIndex, RobotSide finalSupportSide, boolean isLastTransfer, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationIndex);

      convertToFramePointRetainingZ(tempFramePoint1, swingFootPredictedFinalPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      swingFootInitialPolygon.setIncludingFrame(supportFootPolygon);
      supportFootPolygon.setIncludingFrame(swingFootPredictedFinalPolygon);

      int loopEnd = CoPPlanningTools.getCoPPointIndex(transferCoPPointList, endCoPName);
      for (int i = 0; i <= loopEnd; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(transferCoPPointList[i]), finalSupportSide, footstepIndex);
         double segmentDuration = getTransferSegmentTimes(i, footstepIndex);
         copLocationWaypoint.addAndSetIncludingFrame(transferCoPPointList[i], segmentDuration, tempPointForCoPCalculation);
      }

      if (isLastTransfer)
      {
         swingFootPredictedFinalPolygon.setIncludingFrame(swingFootInitialPolygon);
         computeMidFeetPointWithChickenSupportForFinalTransfer(tempPointForCoPCalculation);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         double segmentDuration = getTransferSegmentTimes(loopEnd + 1, footstepIndex) + additionalTimeForFinalTransfer.getDoubleValue();
         copLocationWaypoint.addAndSetIncludingFrame(CoPPointName.FINAL_COP, segmentDuration, tempPointForCoPCalculation);
      }
   }

   private static void convertToFramePointRetainingZ(FramePoint3D framePointToPack, FramePoint2D framePoint2dToCopy, ReferenceFrame referenceFrameToConvertTo)
   {
      framePointToPack.setIncludingFrame(framePoint2dToCopy, 0.0);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the current footstep
    */
   private void computeCoPPointsForFootstep(int copLocationIndex, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationIndex);

      convertToFramePointRetainingZ(tempFramePoint1, swingFootPredictedFinalPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      computeCoPPointsForFootstepTransfer(copLocationIndex, footstepIndex);
      computeCoPPointsForFootstepSwing(copLocationIndex, footstepIndex);
   }

   private void computeCoPPointsForFlamingoStance(int copLocationIndex, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationIndex);

      convertToFramePointRetainingZ(tempFramePoint1, swingFootPredictedFinalPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      // Change this call by making a new function here in case some modifications are needed
      computeCoPPointsForFootstepTransfer(copLocationIndex, footstepIndex);
      computeCoPPointsForFlamingoSingleSupport(copLocationIndex, footstepIndex);
   }

   private void computeCoPPointsForFlamingoSingleSupport(int copLocationsIndex, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationsIndex);
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      int i = 0;
      for (i = 0; i < swingCoPPointList.length - 1; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[i]), upcomingFootstepData.getSupportSide(), footstepIndex);
         copLocationWaypoint.addAndSetIncludingFrame(swingCoPPointList[i], getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
      computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(CoPPointName.FLAMINGO_STANCE_FINAL_COP),
                              upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint.addAndSetIncludingFrame(CoPPointName.FLAMINGO_STANCE_FINAL_COP, getSwingSegmentTimes(i++, footstepIndex), tempPointForCoPCalculation);
      copLocationWaypoint.addAndSetIncludingFrame(CoPPointName.FLAMINGO_STANCE_FINAL_COP, getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
   }

   private void computeCoPPointsForFootstepTransfer(int copLocationsIndex, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationsIndex);
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      for (int i = 0; i < transferCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(transferCoPPointList[i]), upcomingFootstepData.getSupportSide(),
                                 footstepIndex);
         copLocationWaypoint.addAndSetIncludingFrame(transferCoPPointList[i], getTransferSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
   }

   private double getTransferSegmentTimes(int segmentIndex, int footstepIndex)
   {
      double transferTime = transferDurations.get(footstepIndex).getDoubleValue();

      if(footstepIndex > 0 && touchdownDurations.size() > 0)
      {
         int previousSwingTouchdownIndex = footstepIndex - 1;
         double touchdownDuration = touchdownDurations.get(previousSwingTouchdownIndex).getDoubleValue();
         if (Double.isFinite(touchdownDuration) && touchdownDuration > 0.0)
         {
            transferTime -= touchdownDuration;
         }
      }
      
      if (transferTime <= 0.0 || !Double.isFinite(transferTime))
      {
         transferTime = defaultTransferTime;
         if (debug) PrintTools.warn("Using a default transfer time because it is currently invalid.");
      }

      double splitFraction = transferSplitFractions.get(footstepIndex).getDoubleValue();
      if (Double.isNaN(splitFraction))
      {
         splitFraction = 0.5;
         if (debug) PrintTools.warn("Using a transfer split fraction of 0.5 because it is currently unset.");
      }

      if (useTransferSplitFractionFor.get(footstepIndex) == UseSplitFractionFor.TIME)
      {
         switch (segmentIndex)
         {
         case 0:
            return transferTime * splitFraction;
         case 1:
            return transferTime * (1.0 - splitFraction);
         default:
            throw new RuntimeException("For some reason we didn't just use a array that summed to one");
         }
      }
      else
      {
         return transferTime * 0.5;
      }
   }

   private void computeCoPPointsForFootstepSwing(int copLocationsIndex, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(copLocationsIndex);
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      for (int i = 0; i < swingCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[i]), upcomingFootstepData.getSupportSide(),
                                 footstepIndex);
         copLocationWaypoint.addAndSetIncludingFrame(swingCoPPointList[i], getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
      computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[swingCoPPointList.length - 1]),
                              upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint.addAndSetIncludingFrame(swingCoPPointList[swingCoPPointList.length - 1],
                                                  getSwingSegmentTimes(swingCoPPointList.length, footstepIndex), tempPointForCoPCalculation);
   }

   private double getSwingSegmentTimes(int segmentIndex, int footstepIndex)
   {
      double swingTime = swingDurations.get(footstepIndex).getDoubleValue();
      double touchdownDuration = touchdownDurations.get(footstepIndex).getDoubleValue();
      if (swingTime <= 0.0 || !Double.isFinite(swingTime))
      {
         swingTime = defaultSwingTime;
      }
      
      if (Double.isFinite(touchdownDuration) && touchdownDuration > 0.0)
      {
         swingTime += touchdownDuration;
      }
      
      switch (segmentIndex)
      {
      case 0:
         return swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue() * swingSplitFractions.get(footstepIndex).getDoubleValue();
      case 1:
         return swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue() * (1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue());
      case 2:
         return swingTime * (1.0 - swingDurationShiftFractions.get(footstepIndex).getDoubleValue());
      default:
         throw new RuntimeException("For some reason we didn't just use a array that summed to one here as well");
      }
   }

   private void computeCoPPointLocation(FramePoint3D copPointToPlan, CoPPointPlanningParameters copPointParameters, RobotSide supportSide, int footstepIndex)
   {
      CoPPointName copPointName = copPointParameters.getCopPointName();
      if (copPointName == exitCoPName && setExitCoPUnderSpecialCases(copPointToPlan, supportSide))
         return;
      setCoPPointInPolygon(copPointToPlan, copPointParameters.getSupportPolygonName(), copPointName, footstepIndex);

      YoFrameVector2d copOffset = copPointParameters.getCoPOffsets(supportSide);
      double copXOffset = copOffset.getX()
            + getStepLengthToCoPOffset(copPointParameters.getStepLengthOffsetPolygon(), copPointParameters.getStepLengthToCoPOffsetFactor());
      if (copPointParameters.getIsConstrainedToMinMax())
         copXOffset = MathTools.clamp(copXOffset, copPointParameters.getMinCoPOffset().getDoubleValue(), copPointParameters.getMaxCoPOffset().getDoubleValue());
      copPointToPlan.add(copXOffset, copOffset.getY(), 0.0);

      if (copPointParameters.getIsConstrainedToSupportPolygon())
         constrainCoPPointToSupportPolygon(copPointToPlan, copPointParameters.getSupportPolygonName());
      copPointToPlan.changeFrame(worldFrame);
   }

   private boolean setInitialExitCoPUnderSpecialCases(FramePoint3D framePointToPack, RobotSide supportSide)
   {
      return setExitCoPUnderSpecialCases(framePointToPack, swingFootInitialPolygon, supportFootPolygon, supportSide);
   }

   private boolean setExitCoPUnderSpecialCases(FramePoint3D framePointToPack, RobotSide supportSide)
   {
      return setExitCoPUnderSpecialCases(framePointToPack, supportFootPolygon, swingFootPredictedFinalPolygon, supportSide);
   }

   private boolean setExitCoPUnderSpecialCases(FramePoint3D framePointToPack, FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon,
                                               RobotSide supportSide)
   {
      convertToFramePointRetainingZ(tempFramePoint1, swingFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
      convertToFramePointRetainingZ(tempFramePoint2, supportFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
      double supportToSwingStepLength = tempFramePoint1.getX() - tempFramePoint2.getX();
      double supportToSwingStepHeight = tempFramePoint1.getZ() - tempFramePoint2.getZ();
      if (supportFootPolygon.getArea() == 0.0)
      {
         framePointToPack.setToZero(supportFootPolygon.getReferenceFrame());
         framePointToPack.set(supportFootPolygon.getVertex(0));
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else if (putExitCoPOnToes.getBooleanValue()
            && MathTools.isGreaterThanWithPrecision(supportToSwingStepLength, footstepLengthThresholdToPutExitCoPOnToes.getDoubleValue(),
                                                    Epsilons.ONE_HUNDREDTH))
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
         framePointToPack.add(supportFootPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue(),
                              copPointParametersMap.get(exitCoPName).getCoPOffsets(supportSide).getY(), 0.0);
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else if (MathTools.isGreaterThanWithPrecision(-supportToSwingStepHeight, footstepHeightThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue(),
                                                    Epsilons.ONE_HUNDREDTH)
            && MathTools.isGreaterThanWithPrecision(supportToSwingStepLength, footstepLengthThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue(),
                                                    Epsilons.ONE_HUNDREDTH))
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
         framePointToPack.add(supportFootPolygon.getMaxX(), 0.0, 0.0);
         constrainToPolygon(framePointToPack, supportFootPolygon, safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getDoubleValue());
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else
         return false;
   }

   private double getStepLengthToCoPOffset(CoPSupportPolygonNames stepLengthOffsetPolygon, double stepLengthToCoPOffsetFactor)
   {
      switch (stepLengthOffsetPolygon)
      {
      //TODO should all the calculations be done with respect to the support polygon
      case INITIAL_SWING_POLYGON:
         swingFootInitialPolygon.getFrameVertex(swingFootInitialPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempFramePoint1, stepLengthToCoPOffsetFactor);
      case FINAL_SWING_POLYGON:
         swingFootPredictedFinalPolygon.getFrameVertex(swingFootPredictedFinalPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempFramePoint1, stepLengthToCoPOffsetFactor);
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootInitialPolygon, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootPredictedFinalPolygon,
                                         supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
      default:
         return 0.0;
      }
   }

   private void setCoPPointInPolygon(FramePoint3D copPointToPlan, CoPSupportPolygonNames copSupportPolygon, CoPPointName copPointName, int footstepIndex)
   {
      switch (copSupportPolygon)
      {
      //TODO check if if makes sense that all frame points are stored in the support foot polygon have been stored in support polygon frame
      case INITIAL_SWING_POLYGON:
         convertToFramePointRetainingZ(copPointToPlan, swingFootInitialPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
         return;
      case FINAL_SWING_POLYGON:
         convertToFramePointRetainingZ(copPointToPlan, swingFootPredictedFinalPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP && useTransferSplitFractionFor.get(footstepIndex) == UseSplitFractionFor.POSITION)
            computeMidFeetPointByPositionFraction(copPointToPlan, swingFootInitialPolygon, supportFootPolygon,
                                                  transferSplitFractions.get(footstepIndex).getDoubleValue(), supportFootPolygon.getReferenceFrame());
         else
            getDoubleSupportPolygonCentroid(copPointToPlan, supportFootPolygon, swingFootInitialPolygon, supportFootPolygon.getReferenceFrame());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointByPositionFraction(copPointToPlan, supportFootPolygon, swingFootPredictedFinalPolygon,
                                                  transferSplitFractions.get(footstepIndex).getDoubleValue(), supportFootPolygon.getReferenceFrame());
         else
            getDoubleSupportPolygonCentroid(copPointToPlan, supportFootPolygon, swingFootPredictedFinalPolygon, supportFootPolygon.getReferenceFrame());
         return;
      case NULL:
         throw new RuntimeException("No frame defined for CoP point:" + copPointName.toString());
      default:
         convertToFramePointRetainingZ(copPointToPlan, supportFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
         return;
      }
   }

   private static double getStepLengthBasedOffset(FrameConvexPolygon2d supportPolygon, FramePoint3D referencePoint, double stepLengthToCoPOffsetFactor)
   {
      return stepLengthToCoPOffsetFactor * (referencePoint.getX() - supportPolygon.getMaxX());
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by projection
    * @param copPointToConstrain
    * @param supportFoot
    * @param safeDistanceFromSupportPolygonEdges
    */
   private void constrainToPolygon(FramePoint3D copPointToConstrain, FrameConvexPolygon2d constraintPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      tempFramePoint2d.setIncludingFrame(copPointToConstrain);
      tempPolygon.orthogonalProjection(tempFramePoint2d);
      copPointToConstrain.setIncludingFrame(tempFramePoint2d, 0.0);
   }

   /**
    * @param copPointToConstrain
    * @param copPointName
    */
   private void constrainCoPPointToSupportPolygon(FramePoint3D copPointToConstrain, CoPSupportPolygonNames copSupportPolygon)
   {
      switch (copSupportPolygon)
      {
      case INITIAL_SWING_POLYGON:
         constrainToPolygon(copPointToConstrain, swingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_SWING_POLYGON:
         constrainToPolygon(copPointToConstrain, swingFootPredictedFinalPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         //TODO 
         throw new RuntimeException("Constraining to initial double support polygon not yet implemented");
         //getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootInitialPolygon, supportFootPolygon.getReferenceFrame());
         //constrainToPolygon(copPointToConstrain, tempDoubleSupportPolygonCentroid, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         //return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         //TODO
         throw new RuntimeException("Constraining to initial double support polygon not yet implemented");
         //getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootPredictedFinalPolygon, supportFootPolygon.getReferenceFrame());
         //constrainToPolygon(copPointToConstrain, tempDoubleSupportPolygonCentroid, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         //return;
      case NULL:
         throw new RuntimeException("Invalid constraining frame defined for this point");
      default:
         constrainToPolygon(copPointToConstrain, supportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         break;
      }
   }

   /**
    * Updates the variable {@code currentDoubleSupportPolygon} from the specified swing and support polygons 
    */
   private void getDoubleSupportPolygonCentroid(FramePoint3D framePointToPack, FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon,
                                                ReferenceFrame referenceFrameToStoreResultIn)
   {
      tempFramePoint1.setIncludingFrame(swingFootPolygon.getCentroid(), 0.0);
      tempFramePoint1.changeFrame(referenceFrameToStoreResultIn);
      tempFramePoint2.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
      tempFramePoint2.changeFrame(referenceFrameToStoreResultIn);
      framePointToPack.changeFrame(referenceFrameToStoreResultIn);
      framePointToPack.interpolate(tempFramePoint1, tempFramePoint2, 0.5);
   }

   /**
    * Updates the swing and support foot polygons based on footstepIndex
    * <p> Has no memory of the previous state so should be used carefully </p>
    */
   private void updateFootPolygons(RobotSide supportSide, int footstepIndex)
   {
      if (!(upcomingFootstepsData.size() == 0) && footstepIndex >= upcomingFootstepsData.size())
         throw new RuntimeException("CoP planner: Attempting to plan for footstep index, " + footstepIndex + " with only " + upcomingFootstepsData.size()
               + " upcoming footsteps");
      else if (footstepIndex == plannedFootstepIndex)
         return;

      switch (footstepIndex)
      {
      case 0:
         initializeFootPolygons(supportSide, footstepIndex);
         break;
      default:

         if (upcomingFootstepsData.get(footstepIndex).getSwingSide() == upcomingFootstepsData.get(footstepIndex - 1).getSupportSide()) // the normal way 
         {
            swingFootInitialPolygon.setIncludingFrame(supportFootPolygon);
            supportFootPolygon.setIncludingFrame(swingFootPredictedFinalPolygon);
         }
         else
         {
            swingFootInitialPolygon.setIncludingFrame(swingFootPredictedFinalPolygon);
         }
         setFootPolygonFromFootstep(swingFootPredictedFinalPolygon, footstepIndex);
      }
      plannedFootstepIndex = footstepIndex;
   }

   /**
    * Initialize the swing and support foot polygons based on footstep index 
    */
   private void initializeFootPolygons(RobotSide defaultSupportSide, int footstepIndex)
   {
      if (upcomingFootstepsData.size() == 0)
      {
         setFootPolygonFromCurrentState(swingFootInitialPolygon, defaultSupportSide.getOppositeSide());
         swingFootPredictedFinalPolygon.setIncludingFrame(swingFootInitialPolygon);
         setFootPolygonFromCurrentState(supportFootPolygon, defaultSupportSide);
         return;
      }

      FootstepData currentFootstepData;
      if (footstepIndex >= upcomingFootstepsData.size())
         throw new RuntimeException("CoP Planner attempting to generate trajectories for unplanned footsteps");
      else
         currentFootstepData = upcomingFootstepsData.get(footstepIndex);

      switch (footstepIndex)
      {
      case 0:
         setFootPolygonFromCurrentState(swingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromCurrentState(supportFootPolygon, currentFootstepData.getSupportSide());
         setFootPolygonFromFootstep(swingFootPredictedFinalPolygon, footstepIndex);
         break;
      case 1:
         setFootPolygonFromCurrentState(swingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromFootstep(supportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(swingFootPredictedFinalPolygon, footstepIndex);
         break;
      default:
         setFootPolygonFromFootstep(swingFootInitialPolygon, footstepIndex - 2);
         setFootPolygonFromFootstep(supportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(swingFootPredictedFinalPolygon, footstepIndex);
      }
   }

   private void setFootPolygonFromFootstep(FrameConvexPolygon2d framePolygonToPack, int footstepIndex)
   {
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      Footstep upcomingFootstep = upcomingFootstepData.getFootstep();

      framePolygonToPack.clear(upcomingFootstep.getSoleReferenceFrame());

      if (footstepIndex < upcomingFootstepsData.size() && upcomingFootstep != null && upcomingFootstep.getPredictedContactPoints() != null
            && upcomingFootstep.getPredictedContactPoints().size() > 0)
      {
         polygonReference.clear();
         polygonReference.addVertices(upcomingFootstep.getPredictedContactPoints(), upcomingFootstep.getPredictedContactPoints().size());
         polygonReference.update();
         framePolygonToPack.addVertices(polygonReference);
      }
      else
         framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepData.getSwingSide()));
      framePolygonToPack.update();
   }

   private void setFootPolygonFromCurrentState(FrameConvexPolygon2d framePolygonToPack, RobotSide robotSide)
   {
      if (!supportFootPolygonsInSoleZUpFrames.get(robotSide).isEmpty() && ! (supportFootPolygonsInSoleZUpFrames.get(robotSide).getNumberOfVertices() < 3))
         framePolygonToPack.setIncludingFrame(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      else
      {
         framePolygonToPack.clear(soleZUpFrames.get(robotSide));
         framePolygonToPack.addVertices(defaultFootPolygons.get(robotSide));
      }
      framePolygonToPack.update();
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep != null && timing != null)
      {
         if (!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
         {
            upcomingFootstepsData.add().set(footstep, timing);
            numberOfUpcomingFootsteps.increment();
         }
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
   }

   public void removeFootstepQueueFront()
   {
      removeFootstep(0);
   }

   public void removeFootstepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
   }

   public void removeFootstep(int index)
   {
      upcomingFootstepsData.remove(index);
      numberOfUpcomingFootsteps.decrement();
      clearPlan();
   }

   @Override
   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   @Override
   public void setSafeDistanceFromSupportEdges(double distance)
   {
      safeDistanceFromCoPToSupportEdges.set(distance);
   }

   @Override
   public List<CoPPointsInFoot> getWaypoints()
   {
      return copLocationWaypoints;
   }

   /**
    * Picks up the last CoP location in case there is a plan. Else returns the held CoP and in case that is not available, it returns the mid feet point 
    */
   public void getFinalCoPLocation(FramePoint3D framePointToPack)
   {
      if (planIsAvailable.getBooleanValue())
         getDesiredCenterOfPressure(framePointToPack);
      else if (holdDesiredState.getBooleanValue())
         framePointToPack.setIncludingFrame(heldCoPPosition);
      else
      {
         // The selection of swing and support is arbitrary here. 
         setFootPolygonFromCurrentState(swingFootInitialPolygon, RobotSide.LEFT);
         setFootPolygonFromCurrentState(supportFootPolygon, RobotSide.RIGHT);
         getDoubleSupportPolygonCentroid(framePointToPack, supportFootPolygon, swingFootInitialPolygon, worldFrame);
      }
      framePointToPack.changeFrame(worldFrame);
   }

   private void generateCoPTrajectoriesFromWayPoints()
   {
      //It is always guaranteed that the initial state will be transfer the way this code is written. This is needed for the angular momentum approximation to work
      tempFramePoint1.setToNaN(worldFrame);
      WalkingTrajectoryType trajectoryType = WalkingTrajectoryType.TRANSFER;
      double timeInState = 0.0;
      int transferTrajectoryIndex = -1;
      int swingTrajectoryIndex = -1;

      for (int waypointIndex = 0; waypointIndex < copLocationWaypoints.size()
            && !copLocationWaypoints.get(waypointIndex).getCoPPointList().isEmpty(); waypointIndex++)
      {
         CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.get(waypointIndex);
         List<CoPPointName> copList = copLocationWaypoint.getCoPPointList();

         for (int segmentIndex = 0; segmentIndex < copList.size(); segmentIndex++)
         {
            CoPTrajectoryPoint currentPoint = copLocationWaypoint.get(segmentIndex);
            if (!tempFramePoint1.containsNaN())
            {
               if (trajectoryType == WalkingTrajectoryType.SWING)
               {
                  swingCoPTrajectories.get(swingTrajectoryIndex).setNextSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1,
                                                                                currentPoint.getPosition().getFrameTuple());
               }
               else
               {
                  transferCoPTrajectories.get(transferTrajectoryIndex).setNextSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1,
                                                                                      currentPoint.getPosition().getFrameTuple());
               }
            }
            else
            {
               transferTrajectoryIndex++;
               currentPoint.getPosition(tempFramePoint1);
               continue;
            }
            currentPoint.getPosition(tempFramePoint1);

            if (copList.get(segmentIndex) == entryCoPName)
            {
               trajectoryType = WalkingTrajectoryType.SWING;
               timeInState = 0.0;
               swingTrajectoryIndex++;
            }
            else if (copList.get(segmentIndex) == exitCoPName && copList.size() == segmentIndex + 1)
            {
               trajectoryType = WalkingTrajectoryType.TRANSFER;
               timeInState = 0.0;
               transferTrajectoryIndex++;
            }
            else
               timeInState += currentPoint.getTime();
         }
      }
   }

   @Override
   public List<? extends CoPTrajectory> getTransferCoPTrajectories()
   {
      return transferCoPTrajectories;
   }

   @Override
   public List<? extends CoPTrajectory> getSwingCoPTrajectories()
   {
      return swingCoPTrajectories;
   }

   public boolean isOnExitCoP()
   {
      return (activeTrajectory.getTrajectoryType() == WalkingTrajectoryType.SWING
            && activeTrajectory.getCurrentSegmentIndex() == activeTrajectory.getNumberOfSegments() - 1);
   }

   public double getCurrentStateFinalTime()
   {
      if (activeTrajectory.getNumberOfSegments() == 0)
         return 0.0;
      else
         return activeTrajectory.getNodeTimes()[activeTrajectory.getNumberOfSegments()];
   }

   public boolean getIsPlanAvailable()
   {
      return planIsAvailable.getBooleanValue();
   }

   public void getDoubleSupportPolygonCentroid(YoFramePoint copPositionToPack)
   {
      setFootPolygonFromCurrentState(supportFootPolygon, RobotSide.LEFT);
      setFootPolygonFromCurrentState(swingFootInitialPolygon, RobotSide.RIGHT);
      computeMidFeetPointWithChickenSupport(tempFramePoint1, supportFootPolygon, swingFootInitialPolygon);
      tempFramePoint1.changeFrame(copPositionToPack.getReferenceFrame());
      copPositionToPack.set(tempFramePoint1);
   }
}