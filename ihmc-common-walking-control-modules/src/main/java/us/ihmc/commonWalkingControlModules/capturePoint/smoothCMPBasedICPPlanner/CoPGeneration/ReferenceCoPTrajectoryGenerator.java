package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.function.Supplier;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CoPPointPlanningParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools.Bound;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCoPTrajectoryGenerator implements ReferenceCoPTrajectoryGeneratorInterface
{
   private static final int maxNumberOfCoPWaypoints = 20;
   private final boolean debug;

   public enum UseSplitFractionFor
   {
      POSITION, TIME
   }

   private final String fullPrefix;
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   static final double COP_POINT_SIZE = 0.005;

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
   private final SideDependentList<FrameConvexPolygon2D> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2DReadOnly> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberOfPointsPerFoot;
   private final YoInteger numberFootstepsToConsider;
   private final IntegerProvider numberOfUpcomingFootsteps;

   private final YoBoolean goingToPerformInitialDSSmoothingAdjustment;
   private final YoBoolean goingToPerformInitialSSSmoothingAdjustment;

   private final YoDouble maxContinuityAdjustmentSegmentDurationDS;
   private final YoDouble maxContinuityAdjustmentSegmentDurationSS;

   private final List<YoDouble> swingDurations;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> touchdownDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingDurationShiftFractions;
   private final List<YoDouble> transferSplitFractions;
   private final List<UseSplitFractionFor> useTransferSplitFractionFor;
   private final List<FootstepData> upcomingFootstepsData;

   private final YoEnum<CoPSplineType> orderOfSplineInterpolation;

   private final YoBoolean isDoneWalking;
   private final YoBoolean holdDesiredState;
   private final YoBoolean putExitCoPOnToes;
   private final YoBoolean putExitCoPOnToesWhenSteppingDown;
   private final YoBoolean planIsAvailable;

   // Output variables
   private final RecyclingArrayList<CoPPointsInFoot> copLocationWaypoints;
   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   private final ReferenceFrame[] framesToRegister;

   // Runtime variables
   private final FramePoint3D desiredCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();
   private final FramePoint3D heldCoPPosition = new FramePoint3D();
   private final PoseReferenceFrame footFrameAtStartOfSwing = new PoseReferenceFrame("footFrameAtStartOfSwing", worldFrame);
   private final ConvexPolygon2D footPolygonAtStartOfSwing = new ConvexPolygon2D();

   private CoPTrajectory activeTrajectory;
   private FramePoint3D tempDoubleSupportPolygonCentroid = new FramePoint3D();
   private final FrameConvexPolygon2D tempPolygonA = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempPolygonB = new FrameConvexPolygon2D();

   private final RecyclingArrayList<FrameConvexPolygon2D> transferringFromPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2D> transferringToPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);
   private final RecyclingArrayList<FrameConvexPolygon2D> upcomingPolygon = new RecyclingArrayList<>(FrameConvexPolygon2D::new);

   // Temp variables for computation only
   private final ConvexPolygon2D polygonReference = new ConvexPolygon2D();
   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint3D tempFramePoint1 = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final FramePoint2D tempFramePoint2d = new FramePoint2D();
   private final FramePoint3D tempPointForCoPCalculation = new FramePoint3D();
   private final FramePoint3D previousCoPLocation = new FramePoint3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   // Visualization
   private final List<YoFramePoint3D> copWaypointsViz = new ArrayList<>(maxNumberOfCoPWaypoints);

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to
    * pass planning parameters
    *
    * @param namePrefix
    */
   public ReferenceCoPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> touchdownDurations,
                                          List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions,
                                          IntegerProvider numberOfUpcomingFootsteps, List<FootstepData> upcomingFootstepsData,
                                          YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet, numberFootstepsToConsider, swingDurations, transferDurations,
           touchdownDurations, swingSplitFractions, swingDurationShiftFractions, transferSplitFractions, false, numberOfUpcomingFootsteps,
           upcomingFootstepsData, parentRegistry);

   }

   public ReferenceCoPTrajectoryGenerator(String namePrefix, int maxNumberOfFootstepsToConsider, BipedSupportPolygons bipedSupportPolygons,
                                          SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoInteger numberFootstepsToConsider,
                                          List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> touchdownDurations,
                                          List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions,
                                          boolean debug, IntegerProvider numberOfUpcomingFootsteps, List<FootstepData> upcomingFootstepsData,
                                          YoVariableRegistry parentRegistry)
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

      maxContinuityAdjustmentSegmentDurationDS = new YoDouble(namePrefix + "MaxContinuityAdjustmentSegmentDurationDS", registry);
      maxContinuityAdjustmentSegmentDurationSS = new YoDouble(namePrefix + "MaxContinuityAdjustmentSegmentDurationSS", registry);
      maxContinuityAdjustmentSegmentDurationDS.set(0.2);
      maxContinuityAdjustmentSegmentDurationSS.set(0.15);

      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.touchdownDurations = touchdownDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;
      this.useTransferSplitFractionFor = new ArrayList<>(transferSplitFractions.size());
      this.numberOfUpcomingFootsteps = numberOfUpcomingFootsteps;
      this.upcomingFootstepsData = upcomingFootstepsData;
      for (int i = 0; i < transferSplitFractions.size(); i++)
         useTransferSplitFractionFor.add(UseSplitFractionFor.TIME);

      this.numberOfPointsPerFoot = new YoInteger(fullPrefix + "NumberOfPointsPerFootstep", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(fullPrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.isDoneWalking = new YoBoolean(fullPrefix + "IsDoneWalking", registry);
      this.holdDesiredState = new YoBoolean(fullPrefix + "HoldDesiredState", parentRegistry);
      this.putExitCoPOnToes = new YoBoolean(fullPrefix + "PutExitCoPOnToes", parentRegistry);
      this.putExitCoPOnToesWhenSteppingDown = new YoBoolean(fullPrefix + "PutExitCoPOnToesWhenSteppingDown", parentRegistry);
      this.planIsAvailable = new YoBoolean(fullPrefix + "CoPPlanAvailable", parentRegistry);
      this.goingToPerformInitialDSSmoothingAdjustment = new YoBoolean("GoingToPerformInitialDSSmoothingAdjustment", parentRegistry);
      this.goingToPerformInitialSSSmoothingAdjustment = new YoBoolean("GoingToPerformInitialSSSmoothingAdjustment", parentRegistry);

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
            YoFrameVector2D copUserOffset = new YoFrameVector2D(fullPrefix + sidePrefix + "CoPConstantOffset" + pointName.toString(), null, registry);
            copParameters.setCoPOffsets(robotSide, copUserOffset);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(contactableFeet.get(robotSide).getContactPoints2d()));
         defaultFootPolygons.put(robotSide, defaultFootPolygon);

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
      framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};

      copLocationWaypoints = new RecyclingArrayList<>(maxNumberOfFootstepsToConsider + 2, new CoPPointsInFootSupplier());
      copLocationWaypoints.clear();
      copPointsConstructed = true;

      parentRegistry.addChild(registry);
      clear();
   }

   private int copSegmentNumber = 0;
   private boolean copPointsConstructed = false;

   private class CoPPointsInFootSupplier implements Supplier<CoPPointsInFoot>
   {

      public CoPPointsInFoot get()
      {
         CoPPointsInFoot pointsInFoot;
         if (!copPointsConstructed)
            pointsInFoot = new CoPPointsInFoot(fullPrefix, copSegmentNumber, framesToRegister, registry);
         else
            pointsInFoot = new CoPPointsInFoot(fullPrefix, copSegmentNumber, framesToRegister, null);

         copSegmentNumber++;
         return pointsInFoot;
      }
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
      this.putExitCoPOnToesWhenSteppingDown.set(parameters.useExitCoPOnToesForSteppingDown());
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
         YoFramePoint3D yoCoPWaypoint = new YoFramePoint3D("CoPWaypointAfterAdjustment" + waypointIndex, worldFrame, registry);
         YoGraphicPosition copWaypointViz = new YoGraphicPosition("AdjustedCoPWaypointViz" + waypointIndex, yoCoPWaypoint, COP_POINT_SIZE, YoAppearance.Green(),
                                                                  YoGraphicPosition.GraphicType.BALL);
         yoCoPWaypoint.setToNaN();
         yoGraphicsList.add(copWaypointViz);
         artifactList.add(copWaypointViz.createArtifact());
         copWaypointsViz.add(yoCoPWaypoint);
      }
   }

   public void setGoingToPerformDSSmoothingAdjustment(boolean goingToPerformInitialDSSmoothingAdjustment)
   {
      this.goingToPerformInitialDSSmoothingAdjustment.set(goingToPerformInitialDSSmoothingAdjustment);
   }

   public void setGoingToPerformSSSmoothingAdjustment(boolean goingToPerformInitialSSSmoothingAdjustment)
   {
      this.goingToPerformInitialSSSmoothingAdjustment.set(goingToPerformInitialSSSmoothingAdjustment);
   }

   public void holdPosition(FramePoint3DReadOnly desiredCoPPositionToHold)
   {
      holdDesiredState.set(true);
      heldCoPPosition.setIncludingFrame(desiredCoPPositionToHold);
   }

   private void clearHeldPosition()
   {
      holdDesiredState.set(false);
      heldCoPPosition.setToNaN(worldFrame);
   }

   @Override
   public void setSymmetricCoPConstantOffsets(CoPPointName copPointName, Vector2DReadOnly offset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2D copUserOffset = copPointParametersMap.get(copPointName).getCoPOffsets(robotSide);
         copUserOffset.setX(offset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(offset.getY()));
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
      for (int i = 0;
           waypointIndex < copWaypointsViz.size() && i < copLocationWaypoints.size() && !copLocationWaypoints.get(i).getCoPPointList().isEmpty(); i++)
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
         int j;
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
      desiredCoPPosition.setToNaN();
      desiredCoPVelocity.setToNaN();
      desiredCoPAcceleration.setToNaN();
      clearPlan();
   }

   public void clearPlan()
   {
      planIsAvailable.set(false);
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).reset();
      copLocationWaypoints.clear();

      for (int i = 0; i < transferCoPTrajectories.size(); i++)
         transferCoPTrajectories.get(i).reset();
      for (int i = 0; i < swingCoPTrajectories.size(); i++)
         swingCoPTrajectories.get(i).reset();
   }

   @Override
   public int getNumberOfFootstepsRegistered()
   {
      return numberOfUpcomingFootsteps.getValue();
   }

   public Footstep getFootstep(int footstepIndex)
   {
      return upcomingFootstepsData.get(footstepIndex).getFootstep();
   }

   @Override
   public void update(double time)
   {
      if (!planIsAvailable.getBooleanValue())
      {
         setFootPolygonFromCurrentState(tempPolygonA, RobotSide.LEFT);
         setFootPolygonFromCurrentState(tempPolygonB, RobotSide.RIGHT);

         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, tempPolygonA, tempPolygonB, worldFrame);
         desiredCoPPosition.setIncludingFrame(tempDoubleSupportPolygonCentroid);
         desiredCoPVelocity.setToZero(worldFrame);
         desiredCoPAcceleration.setToZero(worldFrame);
      }
      else if (activeTrajectory != null)
      {
         activeTrajectory.update(time, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
      }
      else
         throw new RuntimeException("CoP Planner: Plan available but no active trajectory initialized");
   }

   @Override
   public void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack)
   {
      desiredCoPToPack.set(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(FixedFramePoint3DBasics desiredCoPToPack, FixedFrameVector3DBasics desiredCoPVelocityToPack)
   {
      getDesiredCenterOfPressure(desiredCoPToPack);
      desiredCoPVelocityToPack.set(desiredCoPVelocity);
   }

   /**
    * Remember this in case the plan is cleared but the planner was doing chicken support. In that
    * case the ICP should be offset towards the correct foot.
    */
   private RobotSide lastTransferToSide = RobotSide.LEFT;

   @Override
   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide, RobotSide previousTransferToSide)
   {
      copLocationWaypoints.clear();
      boolean transferringToSameSideAsStartingFrom = previousTransferToSide != null && previousTransferToSide.equals(transferToSide);
      lastTransferToSide = previousTransferToSide == null ? transferToSide.getOppositeSide() : previousTransferToSide;
      initializeAllFootPolygons(transferToSide, transferringToSameSideAsStartingFrom, false);

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());
      RobotSide finalSupportSide = numberOfUpcomingFootsteps > 0 ? upcomingFootstepsData.get(numberOfUpcomingFootsteps - 1).getSupportSide() : transferToSide;
      boolean planIncludesFinalTransfer = numberOfUpcomingFootsteps < numberFootstepsToConsider.getIntegerValue();

      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop)
      { // this guy is starting from standing
         if (numberOfUpcomingFootsteps == 0)
            isDoneWalking.set(true); // not walking
         else
            isDoneWalking.set(false); // start walking

         if (holdDesiredState.getBooleanValue())
         {
            previousCoPLocation.setIncludingFrame(heldCoPPosition);
            previousCoPLocation.changeFrame(worldFrame);
            clearHeldPosition();
         }
         else if (numberOfUpcomingFootsteps == 0)
         { // just standing there
            getDoubleSupportPolygonCentroid(previousCoPLocation, transferringToPolygon.getFirst(), transferringFromPolygon.getFirst(), worldFrame);
         }
         else
         { // starting moving
            computeMidFeetPointWithChickenSupportForInitialTransfer(previousCoPLocation);
         }
         copLocationWaypoint.addWaypoint(CoPPointName.START_COP, 0.0, previousCoPLocation);
      }
      else
      { // starting in motion
         clearHeldPosition();
         if (numberOfUpcomingFootsteps == 0)
            isDoneWalking.set(true); // last step
         else
            isDoneWalking.set(false);

         // Put first CoP at the exitCoP of the swing foot when starting in motion
         computeCoPPointLocationForPreviousPlan(previousCoPLocation, copPointParametersMap.get(exitCoPName), transferToSide.getOppositeSide(),
                                                transferringToSameSideAsStartingFrom);
         copLocationWaypoint.addWaypoint(exitCoPName, 0.0, previousCoPLocation);
      }

      // set swing parameters for angular momentum estimation
      convertToFramePointRetainingZ(tempFramePoint1, transferringToPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, transferringFromPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      // compute all the upcoming footsteps
      for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
         computeCoPPointsForFootstep(footstepIndex);
      computeCoPPointsForFinalTransfer(finalSupportSide, planIncludesFinalTransfer, numberOfUpcomingFootsteps);

      // generate the cop trajectory
      generateCoPTrajectoriesFromWayPoints();
      planIsAvailable.set(true);

      // set active trajectory to be first active trajectory
      this.activeTrajectory = transferCoPTrajectories.get(0);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      copLocationWaypoints.clear();
      clearHeldPosition();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(0);

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      if (numberOfUpcomingFootsteps == 0)
      {
         isDoneWalking.set(true);
         return;
      }

      initializeAllFootPolygons(null, false, true);
      isDoneWalking.set(false);
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      // compute cop waypoint location
      computeCoPPointLocationForPreviousPlan(previousCoPLocation, copPointParametersMap.get(exitCoPName), supportSide.getOppositeSide(), false);
      copLocationWaypoint.addWaypoint(exitCoPName, 0.0, previousCoPLocation);

      // set swing foot parameters for angular momentum estimation
      convertToFramePointRetainingZ(tempFramePoint1, transferringToPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, transferringFromPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      if (upcomingFootstepData.getSwingTime() == Double.POSITIVE_INFINITY)
      { // We're in flamingo support, so only do the current one
         computeCoPPointsForFlamingoStance();
      }
      else
      { // Compute all the upcoming waypoints
         boolean isPlanEnding = numberOfUpcomingFootsteps < numberFootstepsToConsider.getIntegerValue();

         for (int footstepIndex = 0; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
            computeCoPPointsForFootstep(footstepIndex);
         computeCoPPointsForFinalTransfer(upcomingFootstepsData.get(numberOfUpcomingFootsteps - 1).getSupportSide(), isPlanEnding, numberOfUpcomingFootsteps);
      }

      // generate the cop trajectory
      generateCoPTrajectoriesFromWayPoints();
      planIsAvailable.set(true);

      // set active trajectory to be first swing trajectory
      this.activeTrajectory = swingCoPTrajectories.get(0);
   }

   private void computeMidFeetPointWithChickenSupportForInitialTransfer(FramePoint3D framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, transferringFromPolygon.getFirst(), transferringToPolygon.getFirst());
   }

   private void computeMidFeetPointWithChickenSupportForFinalTransfer(FramePoint3D framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, transferringFromPolygon.getLast(), transferringToPolygon.getLast());
   }

   /**
    * Assumes foot polygon and double support polygon has been updated
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint3D framePointToPack, FrameConvexPolygon2D supportFootPolygon,
                                                      FrameConvexPolygon2D swingFootPolygon)
   {
      computeMidFeetPointByPositionFraction(framePointToPack, supportFootPolygon, swingFootPolygon, percentageChickenSupport.getDoubleValue(),
                                            supportFootPolygon.getReferenceFrame());
   }

   // do not use these temporary variables anywhere except this method to avoid modifying them in unwanted places.
   private final FramePoint3D fractionTempMidPoint = new FramePoint3D();
   private final FramePoint3D fractionTempPoint1 = new FramePoint3D();
   private final FramePoint3D fractionTempPoint2 = new FramePoint3D();

   private void computeMidFeetPointByPositionFraction(FramePoint3D framePointToPack, FrameConvexPolygon2DReadOnly footPolygonA,
                                                      FrameConvexPolygon2DReadOnly footPolygonB, double fraction, ReferenceFrame referenceFrameToConvertTo)
   {
      getDoubleSupportPolygonCentroid(fractionTempMidPoint, footPolygonA, footPolygonB, referenceFrameToConvertTo);

      fractionTempPoint1.setIncludingFrame(footPolygonA.getCentroid(), 0.0);
      fractionTempPoint1.changeFrame(referenceFrameToConvertTo);

      fractionTempPoint2.setIncludingFrame(footPolygonB.getCentroid(), 0.0);
      fractionTempPoint2.changeFrame(referenceFrameToConvertTo);

      framePointToPack.setToZero(referenceFrameToConvertTo);

      fraction = MathTools.clamp(fraction, 0.0, 1.0);
      if (fraction < 0.5)
      {
         framePointToPack.interpolate(fractionTempPoint1, fractionTempMidPoint, 2.0 * fraction);
      }
      else
      {
         framePointToPack.interpolate(fractionTempMidPoint, fractionTempPoint2, 2.0 * (fraction - 0.5));
      }
      framePointToPack.changeFrame(worldFrame);
   }

   private void computeCoPPointLocationForPreviousPlan(FramePoint3D exitCoPFromLastPlanToPack, CoPPointPlanningParameters copPointParameters,
                                                       RobotSide swingSide, boolean transferringToSameSideAsStartingFrom)
   {
      CoPPointName copPointName = copPointParameters.getCopPointName();
      RobotSide previousSupportSide = transferringToSameSideAsStartingFrom ? swingSide.getOppositeSide() : swingSide;

      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo = transferringToPolygon.getFirst();
      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom = transferringFromPolygon.getFirst();

      // checks if the previous CoP goes to a special location. If so, use this guy, and complete the function
      if (copPointName == exitCoPName && setInitialExitCoPUnderSpecialCases(exitCoPFromLastPlanToPack, previousSupportSide, footPolygonOfSideTransferringTo,
                                                                            footPolygonOfSideTransferringFrom))
      {
         return;
      }

      // get the base CoP location, which the origin of the side that the robot is transferring from
      setInitialCoPPointToPolygonOrigin(exitCoPFromLastPlanToPack, copPointParameters.getSupportPolygonName(), copPointName, footPolygonOfSideTransferringTo,
                                        footPolygonOfSideTransferringFrom);

      // add the offset, which is the sum of the static offset value, and a ratio of factor of the current step length
      FrameVector2DReadOnly copOffset = copPointParameters.getCoPOffsets(previousSupportSide);
      double copXOffset = copOffset.getX() + getInitialStepLengthToCoPOffset(copPointParameters.getStepLengthOffsetPolygon(),
                                                                             copPointParameters.getStepLengthToCoPOffsetFactor(),
                                                                             footPolygonOfSideTransferringTo, footPolygonOfSideTransferringFrom);

      // clamp the offset value
      if (copPointParameters.getIsConstrainedToMinMax())
         copXOffset = MathTools.clamp(copXOffset, copPointParameters.getMinCoPOffset().getDoubleValue(), copPointParameters.getMaxCoPOffset().getDoubleValue());

      // add the offset to the origin point
      exitCoPFromLastPlanToPack.add(copXOffset, copOffset.getY(), 0.0);

      if (copPointParameters.getIsConstrainedToSupportPolygon())
         constrainInitialCoPPointToSupportPolygon(exitCoPFromLastPlanToPack, copPointParameters.getSupportPolygonName(), footPolygonOfSideTransferringTo,
                                                  footPolygonOfSideTransferringFrom);

      exitCoPFromLastPlanToPack.changeFrame(worldFrame);
   }

   private double getInitialStepLengthToCoPOffset(CoPSupportPolygonNames stepLengthOffsetPolygon, double stepLengthToCoPOffsetFactor,
                                                  FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo,
                                                  FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom)
   {
      switch (stepLengthOffsetPolygon)
      {
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain given the " + stepLengthOffsetPolygon);
      case FINAL_SWING_POLYGON:
         tempFramePoint2d.setIncludingFrame(footPolygonOfSideTransferringTo.getVertex(
               EuclidGeometryPolygonTools.findVertexIndex(footPolygonOfSideTransferringTo, true, Bound.MAX, Bound.MAX)));
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, footPolygonOfSideTransferringFrom.getReferenceFrame());
         return getStepLengthBasedOffset(footPolygonOfSideTransferringFrom, tempFramePoint1, stepLengthToCoPOffsetFactor);
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, footPolygonOfSideTransferringFrom, footPolygonOfSideTransferringTo,
                                         footPolygonOfSideTransferringFrom.getReferenceFrame());
         return getStepLengthBasedOffset(footPolygonOfSideTransferringFrom, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
      case SUPPORT_FOOT_POLYGON:
      case NULL:
      default:
         return 0.0;
      }
   }

   private void constrainInitialCoPPointToSupportPolygon(FramePoint3D copPointToPlan, CoPSupportPolygonNames copSupportPolygon,
                                                         FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo,
                                                         FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom)
   {
      switch (copSupportPolygon)
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         constrainToPolygon(copPointToPlan, footPolygonOfSideTransferringTo, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         //TODO
         throw new RuntimeException("Not implemented yet");
         //getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, swingFootInitialPolygon, supportFootPolygon, );
         //constrainToPolygon(copPointToPlan, tempDoubleSupportPolygonCentroid, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         //return;
      case SUPPORT_FOOT_POLYGON:
      default:
         constrainToPolygon(copPointToPlan, footPolygonOfSideTransferringFrom, safeDistanceFromCoPToSupportEdges.getDoubleValue());
      }
   }

   private void setInitialCoPPointToPolygonOrigin(FramePoint3D copPointToPlan, CoPSupportPolygonNames copSupportPolygon, CoPPointName copPointName,
                                                  FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo,
                                                  FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom)
   {
      switch (copSupportPolygon)
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to compute initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         convertToFramePointRetainingZ(copPointToPlan, footPolygonOfSideTransferringTo.getCentroid(), footPolygonOfSideTransferringFrom.getReferenceFrame());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointByPositionFraction(copPointToPlan, footPolygonOfSideTransferringFrom, footPolygonOfSideTransferringTo,
                                                  transferSplitFractions.get(0).getDoubleValue(), footPolygonOfSideTransferringFrom.getReferenceFrame());
         else
            getDoubleSupportPolygonCentroid(copPointToPlan, footPolygonOfSideTransferringFrom, footPolygonOfSideTransferringTo,
                                            footPolygonOfSideTransferringFrom.getReferenceFrame());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         convertToFramePointRetainingZ(copPointToPlan, footPolygonOfSideTransferringFrom.getCentroid(), footPolygonOfSideTransferringFrom.getReferenceFrame());
      }
   }

   private void computeCoPPointsForFinalTransfer(RobotSide finalSupportSide, boolean isLastTransfer, int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      // this is for angular momentum generation
      convertToFramePointRetainingZ(tempFramePoint1, transferringToPolygon.getLast().getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, transferringFromPolygon.getLast().getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      int loopEnd = CoPPlanningTools.getCoPPointIndex(transferCoPPointList, endCoPName);
      for (int i = 0; i <= loopEnd; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(transferCoPPointList[i]), finalSupportSide, footstepIndex);
         double segmentDuration = getTransferSegmentTimes(i, footstepIndex);
         copLocationWaypoint.addWaypoint(transferCoPPointList[i], segmentDuration, tempPointForCoPCalculation);
      }

      if (isLastTransfer)
      {
         computeMidFeetPointWithChickenSupportForFinalTransfer(tempPointForCoPCalculation);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         double segmentDuration = getTransferSegmentTimes(loopEnd + 1, footstepIndex) + additionalTimeForFinalTransfer.getDoubleValue();
         copLocationWaypoint.addWaypoint(CoPPointName.FINAL_COP, segmentDuration, tempPointForCoPCalculation);
      }
   }

   private static void convertToFramePointRetainingZ(FramePoint3D framePointToPack, FramePoint2DReadOnly framePoint2dToCopy,
                                                     ReferenceFrame referenceFrameToConvertTo)
   {
      framePointToPack.setIncludingFrame(framePoint2dToCopy, 0.0);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the
    * current footstep
    */
   private void computeCoPPointsForFootstep(int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      convertToFramePointRetainingZ(tempFramePoint1, upcomingPolygon.get(footstepIndex).getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, transferringToPolygon.get(footstepIndex).getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      computeCoPPointsForFootstepTransfer(footstepIndex);
      computeCoPPointsForFootstepSwing(footstepIndex);
   }

   private void computeCoPPointsForFlamingoStance()
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.add();

      convertToFramePointRetainingZ(tempFramePoint1, upcomingPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, transferringToPolygon.getFirst().getCentroid(), worldFrame);
      copLocationWaypoint.setSupportFootLocation(tempFramePoint1);

      // Change this call by making a new function here in case some modifications are needed
      computeCoPPointsForFootstepTransfer(0);
      computeCoPPointsForFlamingoSingleSupport();
   }

   private void computeCoPPointsForFlamingoSingleSupport()
   {
      int footstepIndex = 0;
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.getLast();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      int i = 0;
      for (i = 0; i < swingCoPPointList.length - 1; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[i]), upcomingFootstepData.getSupportSide(),
                                 footstepIndex);
         copLocationWaypoint.addWaypoint(swingCoPPointList[i], getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
      computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(CoPPointName.FLAMINGO_STANCE_FINAL_COP),
                              upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint.addWaypoint(CoPPointName.FLAMINGO_STANCE_FINAL_COP, getSwingSegmentTimes(i++, footstepIndex), tempPointForCoPCalculation);
      copLocationWaypoint.addWaypoint(CoPPointName.FLAMINGO_STANCE_FINAL_COP, getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
   }

   private void computeCoPPointsForFootstepTransfer(int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.getLast();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      for (int i = 0; i < transferCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(transferCoPPointList[i]), upcomingFootstepData.getSupportSide(),
                                 footstepIndex);
         copLocationWaypoint.addWaypoint(transferCoPPointList[i], getTransferSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
   }

   private double getTransferSegmentTimes(int segmentIndex, int footstepIndex)
   {
      double transferTime = transferDurations.get(footstepIndex).getDoubleValue();

      if (footstepIndex > 0 && touchdownDurations.size() > 0)
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
         if (debug)
            PrintTools.warn("Using a default transfer time because it is currently invalid.");
      }

      double splitFraction = transferSplitFractions.get(footstepIndex).getDoubleValue();
      if (Double.isNaN(splitFraction))
      {
         splitFraction = 0.5;
         if (debug)
            PrintTools.warn("Using a transfer split fraction of 0.5 because it is currently unset.");
      }

      double segmentDuration;

      if (useTransferSplitFractionFor.get(footstepIndex) == UseSplitFractionFor.TIME)
      {
         if (segmentIndex == 0)
         {
            segmentDuration = transferTime * splitFraction;
         }
         else if (segmentIndex == 1)
         {
            segmentDuration = transferTime * (1.0 - splitFraction);
         }
         else
         {
            throw new RuntimeException("For some reason we didn't just use a array that summed to one");
         }
      }
      else
      {
         segmentDuration = transferTime * 0.5;
      }

      // modifying durations if it's the first step, and the robot is going to perform a continuity adjustment
      if (footstepIndex == 0 && goingToPerformInitialDSSmoothingAdjustment.getBooleanValue())
      {
         if (segmentIndex == 0)
         {
            segmentDuration = Math.min(segmentDuration, maxContinuityAdjustmentSegmentDurationDS.getDoubleValue());
         }
         else if (segmentIndex == 1)
         {
            double initialSegmentDuration = transferTime - segmentDuration;
            double initialSegmentDurationMoved = Math.max(initialSegmentDuration - maxContinuityAdjustmentSegmentDurationDS.getDoubleValue(), 0.0);
            segmentDuration += initialSegmentDurationMoved;
         }
         else
         {
            throw new RuntimeException("Shouldn't be here....");
         }
      }

      return segmentDuration;
   }

   public void initializeForSwing()
   {
      RobotSide swingSide = upcomingFootstepsData.get(0).getSwingSide();
      if (!supportFootPolygonsInSoleZUpFrames.get(swingSide).isEmpty() && !(supportFootPolygonsInSoleZUpFrames.get(swingSide).getNumberOfVertices() < 3))
         footPolygonAtStartOfSwing.set(supportFootPolygonsInSoleZUpFrames.get(swingSide));
      else
         footPolygonAtStartOfSwing.set(defaultFootPolygons.get(swingSide));

      ReferenceFrame swingFootFrame = soleZUpFrames.get(swingSide);
      swingFootFrame.getTransformToDesiredFrame(tempTransform, worldFrame);
      footFrameAtStartOfSwing.setPoseAndUpdate(tempTransform);
   }

   private void computeCoPPointsForFootstepSwing(int footstepIndex)
   {
      CoPPointsInFoot copLocationWaypoint = copLocationWaypoints.getLast();
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);

      for (int i = 0; i < swingCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[i]), upcomingFootstepData.getSupportSide(),
                                 footstepIndex);
         copLocationWaypoint.addWaypoint(swingCoPPointList[i], getSwingSegmentTimes(i, footstepIndex), tempPointForCoPCalculation);
      }
      computeCoPPointLocation(tempPointForCoPCalculation, copPointParametersMap.get(swingCoPPointList[swingCoPPointList.length - 1]),
                              upcomingFootstepData.getSupportSide(), footstepIndex);
      copLocationWaypoint
            .addWaypoint(swingCoPPointList[swingCoPPointList.length - 1], getSwingSegmentTimes(swingCoPPointList.length, footstepIndex),
                         tempPointForCoPCalculation);
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

      double initialSegmentDuration =
            swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue() * swingSplitFractions.get(footstepIndex).getDoubleValue();
      double segmentDuration;

      switch (segmentIndex)
      {
      case 0:
         segmentDuration = initialSegmentDuration;
         break;
      case 1:
         segmentDuration =
               swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue() * (1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue());
         break;
      case 2:
         segmentDuration = swingTime * (1.0 - swingDurationShiftFractions.get(footstepIndex).getDoubleValue());
         break;
      default:
         throw new RuntimeException("For some reason we didn't just use a array that summed to one here as well");
      }

      if (footstepIndex == 0 && goingToPerformInitialSSSmoothingAdjustment.getBooleanValue())
      {
         if (segmentIndex == 0)
         {
            segmentDuration = Math.min(segmentDuration, maxContinuityAdjustmentSegmentDurationSS.getDoubleValue());
         }
         else if (segmentIndex == 1)
         {
            double initialSegmentDurationMoved = Math.max(initialSegmentDuration - maxContinuityAdjustmentSegmentDurationSS.getDoubleValue(), 0.0);
            segmentDuration += initialSegmentDurationMoved;
         }
      }

      return segmentDuration;
   }

   private void computeCoPPointLocation(FramePoint3D copPointToPlan, CoPPointPlanningParameters copPointParameters, RobotSide supportSide, int footstepIndex)
   {
      CoPPointName copPointName = copPointParameters.getCopPointName();
      if (copPointName == exitCoPName && setExitCoPUnderSpecialCases(copPointToPlan, supportSide, footstepIndex))
         return;
      setCoPPointInPolygon(copPointToPlan, copPointParameters.getSupportPolygonName(), copPointName, footstepIndex);

      YoFrameVector2D copOffset = copPointParameters.getCoPOffsets(supportSide);
      double copXOffset =
            copOffset.getX() + getStepLengthToCoPOffset(copPointParameters.getStepLengthOffsetPolygon(), copPointParameters.getStepLengthToCoPOffsetFactor(),
                                                        footstepIndex);
      if (copPointParameters.getIsConstrainedToMinMax())
         copXOffset = MathTools.clamp(copXOffset, copPointParameters.getMinCoPOffset().getDoubleValue(), copPointParameters.getMaxCoPOffset().getDoubleValue());
      copPointToPlan.add(copXOffset, copOffset.getY(), 0.0);

      if (copPointParameters.getIsConstrainedToSupportPolygon())
         constrainCoPPointToSupportPolygon(copPointToPlan, copPointParameters.getSupportPolygonName(), footstepIndex);
      copPointToPlan.changeFrame(worldFrame);
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setInitialExitCoPUnderSpecialCases(FramePoint3D exitCoPToPack, RobotSide previousSupportSide,
                                                      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringTo,
                                                      FrameConvexPolygon2DReadOnly footPolygonOfSideTransferringFrom)
   {
      return setExitCoPUnderSpecialCases(exitCoPToPack, footPolygonOfSideTransferringFrom, footPolygonOfSideTransferringTo, previousSupportSide);
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPUnderSpecialCases(FramePoint3D exitCoPToPack, RobotSide supportSide, int footstepIndex)
   {
      return setExitCoPUnderSpecialCases(exitCoPToPack, transferringToPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex + 1), supportSide);
   }

   /**
    * Checks for the following conditions to have been the case:
    *  - the support polygon is empty
    *  - the exit CoP goes in the toes
    *  - the exit CoP goes in the toes when stepping down
    *
    * @return true if any of these cases held, at which point the CoP has been placed. False if none of them held, at which it still needs to be computed.
    */
   private boolean setExitCoPUnderSpecialCases(FramePoint3D framePointToPack, FrameConvexPolygon2DReadOnly supportFootPolygon,
                                               FrameConvexPolygon2DReadOnly upcomingSwingFootPolygon, RobotSide supportSide)
   {
      convertToFramePointRetainingZ(tempFramePoint1, upcomingSwingFootPolygon.getCentroid(), supportFootPolygon.getReferenceFrame());
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
      else if (putExitCoPOnToes.getBooleanValue() && MathTools
            .isGreaterThanWithPrecision(supportToSwingStepLength, footstepLengthThresholdToPutExitCoPOnToes.getDoubleValue(), Epsilons.ONE_HUNDREDTH))
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
         framePointToPack.add(supportFootPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue(),
                              copPointParametersMap.get(exitCoPName).getCoPOffsets(supportSide).getY(), 0.0);
         framePointToPack.changeFrame(worldFrame);
         return true;
      }
      else if (putExitCoPOnToesWhenSteppingDown.getBooleanValue() && MathTools
            .isGreaterThanWithPrecision(-supportToSwingStepHeight, footstepHeightThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue(),
                                        Epsilons.ONE_HUNDREDTH) && MathTools
            .isGreaterThanWithPrecision(supportToSwingStepLength, footstepLengthThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue(),
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

   private double getStepLengthToCoPOffset(CoPSupportPolygonNames stepLengthOffsetPolygon, double stepLengthToCoPOffsetFactor, int footstepIndex)
   {
      switch (stepLengthOffsetPolygon)
      {
      //TODO should all the calculations be done with respect to the support polygon
      case INITIAL_SWING_POLYGON:
         tempFramePoint2d.setIncludingFrame(transferringFromPolygon.get(footstepIndex).getVertex(
               EuclidGeometryPolygonTools.findVertexIndex(transferringFromPolygon.get(footstepIndex), true, Bound.MAX, Bound.MAX)));
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempFramePoint1, stepLengthToCoPOffsetFactor);
      case FINAL_SWING_POLYGON:
         tempFramePoint2d.setIncludingFrame(transferringToPolygon.get(footstepIndex + 1).getVertex(
               EuclidGeometryPolygonTools.findVertexIndex(transferringToPolygon.get(footstepIndex + 1), true, Bound.MAX, Bound.MAX)));
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempFramePoint1, stepLengthToCoPOffsetFactor);
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, transferringToPolygon.get(footstepIndex), transferringFromPolygon.get(footstepIndex),
                                         transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, transferringToPolygon.get(footstepIndex),
                                         transferringToPolygon.get(footstepIndex + 1), transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return getStepLengthBasedOffset(transferringToPolygon.get(footstepIndex), tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactor);
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
         convertToFramePointRetainingZ(copPointToPlan, transferringFromPolygon.get(footstepIndex).getCentroid(),
                                       transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return;
      case FINAL_SWING_POLYGON:
         convertToFramePointRetainingZ(copPointToPlan, transferringToPolygon.get(footstepIndex + 1).getCentroid(),
                                       transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP && useTransferSplitFractionFor.get(footstepIndex) == UseSplitFractionFor.POSITION)
         {
            computeMidFeetPointByPositionFraction(copPointToPlan, transferringFromPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex),
                                                  transferSplitFractions.get(footstepIndex).getDoubleValue(),
                                                  transferringToPolygon.get(footstepIndex).getReferenceFrame());
         }
         else
         {
            computeMidFeetPointByPositionFraction(copPointToPlan, transferringFromPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex),
                                                  percentageChickenSupport.getDoubleValue(), transferringToPolygon.get(footstepIndex).getReferenceFrame());
         }
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         if (copPointName == CoPPointName.MIDFEET_COP)
         {
            computeMidFeetPointByPositionFraction(copPointToPlan, transferringToPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex + 1),
                                                  transferSplitFractions.get(footstepIndex).getDoubleValue(),
                                                  transferringToPolygon.get(footstepIndex).getReferenceFrame());
         }
         else
         {
            getDoubleSupportPolygonCentroid(copPointToPlan, transferringToPolygon.get(footstepIndex), transferringToPolygon.get(footstepIndex + 1),
                                            transferringToPolygon.get(footstepIndex).getReferenceFrame());
         }
         return;
      case NULL:
         throw new RuntimeException("No frame defined for CoP point:" + copPointName.toString());
      default:
         convertToFramePointRetainingZ(copPointToPlan, transferringToPolygon.get(footstepIndex).getCentroid(),
                                       transferringToPolygon.get(footstepIndex).getReferenceFrame());
         return;
      }
   }

   private static double getStepLengthBasedOffset(FrameConvexPolygon2DReadOnly supportPolygon, FramePoint3DReadOnly referencePoint,
                                                  double stepLengthToCoPOffsetFactor)
   {
      supportPolygon.checkReferenceFrameMatch(referencePoint);
      return stepLengthToCoPOffsetFactor * (referencePoint.getX() - supportPolygon.getMaxX());
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by
    * projection
    */
   private void constrainToPolygon(FramePoint3D copPointToConstrain, FrameConvexPolygon2DReadOnly constraintPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      tempFramePoint2d.setIncludingFrame(copPointToConstrain);
      tempPolygon.orthogonalProjection(tempFramePoint2d);
      copPointToConstrain.setIncludingFrame(tempFramePoint2d, 0.0);
   }

   private void constrainCoPPointToSupportPolygon(FramePoint3D copPointToConstrain, CoPSupportPolygonNames copSupportPolygon, int footstepIndex)
   {
      switch (copSupportPolygon)
      {
      case INITIAL_SWING_POLYGON:
         constrainToPolygon(copPointToConstrain, transferringFromPolygon.get(footstepIndex), safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_SWING_POLYGON:
         constrainToPolygon(copPointToConstrain, transferringToPolygon.get(footstepIndex + 1), safeDistanceFromCoPToSupportEdges.getDoubleValue());
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
         constrainToPolygon(copPointToConstrain, transferringToPolygon.get(footstepIndex), safeDistanceFromCoPToSupportEdges.getDoubleValue());
         break;
      }
   }

   // do not use these temporary variables anywhere except this method to avoid modifying them in unwanted places.
   private final FramePoint3D doubleSupportCentroidTempPoint1 = new FramePoint3D();
   private final FramePoint3D doubleSupportCentroidTempPoint2 = new FramePoint3D();

   /**
    * Updates the variable {@code currentDoubleSupportPolygon} from the specified swing and support
    * polygons
    */
   private void getDoubleSupportPolygonCentroid(FramePoint3D framePointToPack, FrameConvexPolygon2DReadOnly supportFootPolygon,
                                                FrameConvexPolygon2DReadOnly swingFootPolygon, ReferenceFrame referenceFrameToStoreResultIn)
   {
      doubleSupportCentroidTempPoint1.setIncludingFrame(swingFootPolygon.getCentroid(), 0.0);
      doubleSupportCentroidTempPoint1.changeFrame(referenceFrameToStoreResultIn);
      doubleSupportCentroidTempPoint2.setIncludingFrame(supportFootPolygon.getCentroid(), 0.0);
      doubleSupportCentroidTempPoint2.changeFrame(referenceFrameToStoreResultIn);
      framePointToPack.changeFrame(referenceFrameToStoreResultIn);
      framePointToPack.interpolate(doubleSupportCentroidTempPoint1, doubleSupportCentroidTempPoint2, 0.5);
   }

   private void initializeAllFootPolygons(RobotSide upcomingSupportSide, boolean transferringToSameSideAsStartingFrom, boolean planningFromSwing)
   {
      transferringFromPolygon.clear();
      transferringToPolygon.clear();
      upcomingPolygon.clear();

      // in final transfer, or in stand
      if (upcomingFootstepsData.size() == 0)
      {
         if (transferringToSameSideAsStartingFrom)
         {
            setFootPolygonFromCurrentState(transferringFromPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(upcomingPolygon.add(), upcomingSupportSide);
            setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingSupportSide.getOppositeSide());
         }
         else
         {
            setFootPolygonFromCurrentState(transferringFromPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(upcomingPolygon.add(), upcomingSupportSide.getOppositeSide());
            setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingSupportSide);
         }
         return;
      }

      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getValue());

      RobotSide transferringFromSide = transferringToSameSideAsStartingFrom ? upcomingFootstepsData.get(0).getSupportSide() : upcomingFootstepsData.get(0).getSwingSide();

      if(planningFromSwing)
         setFootPolygonDirectly(transferringFromPolygon.add(), footFrameAtStartOfSwing, footPolygonAtStartOfSwing);
      else
         setFootPolygonFromCurrentState(transferringFromPolygon.add(), transferringFromSide);

      setFootPolygonFromCurrentState(transferringToPolygon.add(), upcomingFootstepsData.get(0).getSupportSide());
      setFootPolygonFromFootstep(upcomingPolygon.add(), 0);

      int footstepIndex = 1;
      for (; footstepIndex < numberOfUpcomingFootsteps; footstepIndex++)
      {
         transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));

         if (upcomingFootstepsData.get(footstepIndex).getSwingSide().equals(upcomingFootstepsData.get(footstepIndex - 1).getSwingSide()))
         { // stepping to the same side
            transferringToPolygon.add().setIncludingFrame(transferringFromPolygon.getLast());
         }
         else
         {
            transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
         }
         setFootPolygonFromFootstep(upcomingPolygon.add(), footstepIndex);
      }

      transferringFromPolygon.add().setIncludingFrame(transferringToPolygon.get(footstepIndex - 1));
      transferringToPolygon.add().setIncludingFrame(upcomingPolygon.get(footstepIndex - 1));
   }

   private void setFootPolygonFromFootstep(FrameConvexPolygon2D framePolygonToPack, int footstepIndex)
   {
      FootstepData upcomingFootstepData = upcomingFootstepsData.get(footstepIndex);
      Footstep upcomingFootstep = upcomingFootstepData.getFootstep();

      framePolygonToPack.clear(upcomingFootstep.getSoleReferenceFrame());

      if (footstepIndex < upcomingFootstepsData.size() && upcomingFootstep != null && upcomingFootstep.getPredictedContactPoints() != null
            && upcomingFootstep.getPredictedContactPoints().size() > 0)
      {
         polygonReference.clear();
         polygonReference.addVertices(Vertex2DSupplier.asVertex2DSupplier(upcomingFootstep.getPredictedContactPoints()));
         polygonReference.update();
         framePolygonToPack.addVertices(polygonReference);
      }
      else
         framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepData.getSwingSide()));
      framePolygonToPack.update();
   }

   private void setFootPolygonFromCurrentState(FrameConvexPolygon2D framePolygonToPack, RobotSide robotSide)
   {
      if (!supportFootPolygonsInSoleZUpFrames.get(robotSide).isEmpty() && !(supportFootPolygonsInSoleZUpFrames.get(robotSide).getNumberOfVertices() < 3))
         framePolygonToPack.setIncludingFrame(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      else
      {
         framePolygonToPack.clear(soleZUpFrames.get(robotSide));
         framePolygonToPack.addVertices(defaultFootPolygons.get(robotSide));
      }
      framePolygonToPack.update();
   }

   private void setFootPolygonDirectly(FrameConvexPolygon2D frameConvexPolygonToPack, ReferenceFrame referenceFrame, Vertex2DSupplier vertexSupplier)
   {
      frameConvexPolygonToPack.clear(referenceFrame);
      frameConvexPolygonToPack.addVertices(vertexSupplier);
      frameConvexPolygonToPack.update();
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
    * Picks up the last CoP location in case there is a plan. Else returns the held CoP and in case
    * that is not available, it returns the mid feet point
    */
   public void getFinalCoPLocation(FramePoint3D framePointToPack)
   {
      if (planIsAvailable.getBooleanValue())
      {
         getDesiredCenterOfPressure(framePointToPack);
      }
      else if (holdDesiredState.getBooleanValue())
      {
         framePointToPack.setIncludingFrame(heldCoPPosition);
      }
      else
      {
         // The selection of swing and support is arbitrary here. 
         setFootPolygonFromCurrentState(tempPolygonA, RobotSide.LEFT);
         setFootPolygonFromCurrentState(tempPolygonB, RobotSide.RIGHT);

         getDoubleSupportPolygonCentroid(framePointToPack, tempPolygonB, tempPolygonA, worldFrame);
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

      for (int waypointIndex = 0;
           waypointIndex < copLocationWaypoints.size() && !copLocationWaypoints.get(waypointIndex).getCoPPointList().isEmpty(); waypointIndex++)
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
                  swingCoPTrajectories.get(swingTrajectoryIndex)
                                      .setNextSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1, currentPoint.getPosition());
               }
               else
               {
                  transferCoPTrajectories.get(transferTrajectoryIndex)
                                         .setNextSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1, currentPoint.getPosition());
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

   public void getDoubleSupportPolygonCentroid(FixedFramePoint3DBasics copPositionToPack)
   {
      setFootPolygonFromCurrentState(tempPolygonA, lastTransferToSide.getOppositeSide());
      setFootPolygonFromCurrentState(tempPolygonB, lastTransferToSide);

      computeMidFeetPointWithChickenSupport(tempFramePoint1, tempPolygonA, tempPolygonB);
      tempFramePoint1.changeFrame(copPositionToPack.getReferenceFrame());
      copPositionToPack.set(tempFramePoint1);
   }
}