package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
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

//TODO 1) add isDoneWalking functionality

public class ReferenceCoPTrajectoryGenerator implements ReferenceCoPTrajectoryGeneratorInterface
{
   public enum UseSplitFractionFor
   {
      POSITION, TIME
   };

   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double COP_POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   // Waypoint planning parameters
   private final SideDependentList<EnumMap<CoPPointName, YoFrameVector2d>> copOffsets = new SideDependentList<>();
   private EnumMap<CoPPointName, CoPSupportPolygonNames> copSupportPolygon;
   private EnumMap<CoPPointName, Boolean> isConstrainedToSupportPolygonFlags;
   private double defaultSwingTime;
   private double defaultTransferTime;

   private EnumMap<CoPPointName, Boolean> isConstrainedToMinMaxFlags;
   private final EnumMap<CoPPointName, YoDouble> maxCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);
   private final EnumMap<CoPPointName, YoDouble> minCoPOffsets = new EnumMap<CoPPointName, YoDouble>(CoPPointName.class);

   private EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactors;
   private EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetReferencePolygons;

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

   private int footstepIndex = 0;
   private int plannedFootstepIndex = -1;
   private int numberOfSwingSegments = 3;
   private int numberOfTransferSegments = 2;
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FramePoint3D tempDoubleSupportPolygonCentroid = new FramePoint3D();
   private FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootPredictedFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private double tempDouble;
   private FootstepData currentFootstepData;
   private FrameConvexPolygon2d framePolygonReference;
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

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCoPTrajectoryGenerator(String namePrefix, int numberOfPointsPerFoot, int maxNumberOfFootstepsToConsider,
                                          BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                          YoInteger numberFootstepsToConsider, List<YoDouble> swingDurations, List<YoDouble> transferDurations,
                                          List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions, List<YoDouble> transferSplitFractions,
                                          YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.numberFootstepsToConsider = numberFootstepsToConsider;

      additionalTimeForFinalTransfer = new YoDouble(namePrefix + "AdditionalTimeForFinalTransfer", registry);
      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", parentRegistry);
      footstepHeightThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepHeightThresholdToPutExitCoPOnToesSteppingDown",
                                                                           parentRegistry);
      footstepLengthThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCoPOnToesSteppingDown",
                                                                           parentRegistry);
      footstepLengthThresholdToPutExitCoPOnToes = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCoPOnToes", parentRegistry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(namePrefix + "ExitCoPForwardSafetyMarginOnToes", parentRegistry);

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);

      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;
      this.useTransferSplitFractionFor = new ArrayList<>(transferSplitFractions.size());
      for (int i = 0; i < transferSplitFractions.size(); i++)
         useTransferSplitFractionFor.add(UseSplitFractionFor.TIME);

      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);
      this.holdDesiredState = new YoBoolean(namePrefix + "HoldDesiredState", parentRegistry);
      this.putExitCoPOnToes = new YoBoolean(namePrefix + "PutExitCoPOnToes", parentRegistry);
      this.planIsAvailable = new YoBoolean(namePrefix + "CoPPlanAvailable", parentRegistry);

      for (CoPPointName pointName : CoPPointName.values)
      {
         YoDouble maxCoPOffset = new YoDouble("maxCoPForwardOffset" + pointName.toString(), registry);
         YoDouble minCoPOffset = new YoDouble("minCoPForwardOffset" + pointName.toString(), registry);

         this.maxCoPOffsets.put(pointName, maxCoPOffset);
         this.minCoPOffsets.put(pointName, minCoPOffset);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(robotSide).getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon.getConvexPolygon2d());

         supportFootPolygonsInSoleZUpFrames.put(robotSide, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(robotSide));

         String sidePrefix = robotSide.getCamelCaseNameForMiddleOfExpression();
         EnumMap<CoPPointName, YoFrameVector2d> copUserOffsets = new EnumMap<>(CoPPointName.class);

         for (CoPPointName pointName : CoPPointName.values)
         {
            YoFrameVector2d copUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "CoPConstantOffset" + pointName.toString(), null, registry);
            copUserOffsets.put(pointName, copUserOffset);
         }

         this.copOffsets.put(robotSide, copUserOffsets);
      }

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(i, orderOfSplineInterpolation.getEnumValue(), this.numberOfTransferSegments);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(i, orderOfSplineInterpolation.getEnumValue(), this.numberOfSwingSegments);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(maxNumberOfFootstepsToConsider, orderOfSplineInterpolation.getEnumValue(),
                                                                              this.numberOfTransferSegments);
      transferCoPTrajectories.add(transferCoPTrajectory);

      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};

      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i, framesToRegister, registry));
      }
      copLocationWaypoints.add(new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 1, framesToRegister, registry));
      copLocationWaypoints.add(new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 2, framesToRegister, registry));

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
      
      this.copSupportPolygon = parameters.getSupportPolygonNames();
      this.isConstrainedToSupportPolygonFlags = parameters.getIsConstrainedToSupportPolygonFlags();
      this.isConstrainedToMinMaxFlags = parameters.getIsConstrainedToMinMaxFlags();
      this.stepLengthToCoPOffsetFactors = parameters.getStepLengthToCoPOffsetFactors();
      this.stepLengthOffsetReferencePolygons = parameters.getStepLengthToCoPOffsetFlags();

      this.copSupportPolygon = parameters.getSupportPolygonNames();
      this.isConstrainedToSupportPolygonFlags = parameters.getIsConstrainedToSupportPolygonFlags();
      this.isConstrainedToMinMaxFlags = parameters.getIsConstrainedToMinMaxFlags();
      this.stepLengthToCoPOffsetFactors = parameters.getStepLengthToCoPOffsetFactors();
      this.stepLengthOffsetReferencePolygons = parameters.getStepLengthToCoPOffsetFlags();

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

      EnumMap<CoPPointName, Vector2D> copOffsets = parameters.getCopOffsetsInFootFrame();
      CoPPointName[] transferCoPNames = parameters.getTransferCoPPointsToPlan();
      CoPPointName[] swingCoPNames = parameters.getSwingCoPPointsToPlan();
      for (int waypointNumber = 0; waypointNumber < transferCoPNames.length; waypointNumber++)
         setSymmetricCoPConstantOffsets(transferCoPNames[waypointNumber], copOffsets.get(transferCoPNames[waypointNumber]));
      for (int waypointNumber = 0; waypointNumber < swingCoPNames.length; waypointNumber++)
         setSymmetricCoPConstantOffsets(swingCoPNames[waypointNumber], copOffsets.get(swingCoPNames[waypointNumber]));

      EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds = parameters.getCoPForwardOffsetBoundsInFoot();
      for (int waypointIndex = 0; waypointIndex < transferCoPNames.length; waypointIndex++)
      {
         Vector2D bounds = copForwardOffsetBounds.get(transferCoPNames[waypointIndex]);
         minCoPOffsets.get(transferCoPNames[waypointIndex]).set(bounds.getX());
         maxCoPOffsets.get(transferCoPNames[waypointIndex]).set(bounds.getY());
      }
      for (int waypointIndex = 0; waypointIndex < swingCoPNames.length; waypointIndex++)
      {
         Vector2D bounds = copForwardOffsetBounds.get(swingCoPNames[waypointIndex]);
         minCoPOffsets.get(swingCoPNames[waypointIndex]).set(bounds.getX());
         maxCoPOffsets.get(swingCoPNames[waypointIndex]).set(bounds.getY());
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
         YoFrameVector2d copUserOffset = copOffsets.get(robotSide).get(copPointName);
         copUserOffset.setX(offset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(offset.getY()));
      }
   }

   @Override
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int footIndex = 0; footIndex < copLocationWaypoints.size(); footIndex++)
      {
         CoPPointsInFoot copPointsInFoot = copLocationWaypoints.get(footIndex);
         copPointsInFoot.setupVisualizers(yoGraphicsList, artifactList, COP_POINT_SIZE);
      }
   }

   @Override
   public void updateListeners()
   {
      for (int i = 0; i < copLocationWaypoints.size(); i++)
         copLocationWaypoints.get(i).notifyVariableChangedListeners();
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
      footstepIndex = 0;
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
         initializeFootPolygons(RobotSide.LEFT);
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
      footstepIndex = 0;
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
            initializeFootPolygons(transferToSide);
            getDoubleSupportPolygonCentroid(tempPointForCoPCalculation, supportFootPolygon, swingFootInitialPolygon, worldFrame);
         }
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(CoPPointName.START_COP, 0.0, tempPointForCoPCalculation);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(tempFramePoint1);
         computeCoPPointsForFinalTransfer(footstepIndex + 1, transferToSide, true);
      }
      else if (atAStop)
      {
         isDoneWalking.set(false);
         updateFootPolygons(null);
         if (holdDesiredState.getBooleanValue())
         {
            tempPointForCoPCalculation.setIncludingFrame(heldCoPPosition);
            clearHeldPosition();
         }
         else
            computeMidFeetPointWithChickenSupportForInitialTransfer(tempPointForCoPCalculation);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(CoPPointName.START_COP, 0.0, tempPointForCoPCalculation);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(tempFramePoint1);
         computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      }
      // Put first CoP at the exitCoP of the swing foot if not starting from rest 
      else if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         //transferToSide = transferToSide.getOppositeSide();
         clearHeldPosition();
         isDoneWalking.set(true);
         initializeFootPolygons(transferToSide.getOppositeSide());
         computeCoPPointLocation(tempPointForCoPCalculation, exitCoPName, transferToSide.getOppositeSide());
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(tempFramePoint1);
         computeCoPPointsForFinalTransfer(footstepIndex + 1, transferToSide, true);
      }
      else
      {
         clearHeldPosition();
         isDoneWalking.set(false);
         updateFootPolygons(null);
         computeCoPPointLocationForPreviousPlan(tempPointForCoPCalculation, exitCoPName, upcomingFootstepsData.get(footstepIndex).getSwingSide());
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(tempFramePoint1);
         computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      }
      generateCoPTrajectoriesFromWayPoints();
      planIsAvailable.set(true);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      clearHeldPosition();
      footstepIndex = 0;
      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         isDoneWalking.set(true);
         return;
      }
      else
      {
         updateFootPolygons(null);
         isDoneWalking.set(false);
         computeCoPPointLocationForPreviousPlan(tempPointForCoPCalculation, exitCoPName, upcomingFootstepsData.get(footstepIndex).getSwingSide());
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempPointForCoPCalculation);
         convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(tempFramePoint1);
         convertToFramePointRetainingZ(tempFramePoint1, swingFootInitialPolygon.getCentroid(), worldFrame);
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(tempFramePoint1);
         if(upcomingFootstepsData.get(footstepIndex).getSwingTime() == Double.POSITIVE_INFINITY)
         {
            upcomingFootstepsData.get(footstepIndex).setSwingTime(defaultSwingTime);
            computeCoPPointsForFootstep(footstepIndex + 1);
         }
         else
         {
            computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
         }
      }
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
      this.tempDouble = MathTools.clamp(fraction, 0.0, 1.0);
      if (this.tempDouble < 0.5)
      {
         this.tempDouble *= 2.0;
         this.framePolygonReference = footPolygonA;
      }
      else
      {
         this.tempDouble = (this.tempDouble - 0.5) * 2.0;
         this.framePolygonReference = footPolygonB;
      }
      getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, footPolygonA, footPolygonB, referenceFrameToConvertTo);
      convertToFramePointRetainingZ(tempFramePoint1, framePolygonReference.getCentroid(), referenceFrameToConvertTo);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
      framePointToPack.interpolate(this.tempDoubleSupportPolygonCentroid, tempFramePoint1, this.tempDouble);
   }

   private void computeCoPPointLocationForPreviousPlan(FramePoint3D framePointToPack, CoPPointName copPointName, RobotSide swingSide)
   {
      if (copPointName == exitCoPName && setInitialExitCoPUnderSpecialCases(framePointToPack, swingSide))
         return;
      setInitialCoPPointToPolygonOrigin(framePointToPack, copPointName);
      this.tempDouble = copOffsets.get(swingSide).get(copPointName).getX() + getInitialStepLengthToCoPOffset(copPointName);

      if (isConstrainedToMinMaxFlags.get(copPointName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());
      framePointToPack.add(this.tempDouble, copOffsets.get(swingSide).get(copPointName).getY(), 0.0);
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainInitialCoPPointToSupportPolygon(framePointToPack, copPointName);
      framePointToPack.changeFrame(worldFrame);
   }

   private double getInitialStepLengthToCoPOffset(CoPPointName copPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(copPointName))
      {
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain " + copPointName.toString() + " using given parameters: "
               + stepLengthOffsetReferencePolygons.get(copPointName).toString());
      case FINAL_SWING_POLYGON:
         supportFootPolygon.getFrameVertex(supportFootPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, swingFootInitialPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(swingFootInitialPolygon, tempFramePoint1, stepLengthToCoPOffsetFactors.get(copPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, swingFootInitialPolygon, supportFootPolygon,
                                         swingFootInitialPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(swingFootInitialPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactors.get(copPointName));
      case SUPPORT_FOOT_POLYGON:
      case NULL:
      default:
         return 0.0;
      }
   }

   private void constrainInitialCoPPointToSupportPolygon(FramePoint3D copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
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

   private void setInitialCoPPointToPolygonOrigin(FramePoint3D copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
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

   private void computeCoPPointsForUpcomingFootsteps(int copLocationIndex)
   {
      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getIntegerValue());
      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         updateFootPolygons(upcomingFootstepsData.get(footstepIndex).getSupportSide());
         computeCoPPointsForFootstep(copLocationIndex);
         footstepIndex++;
         copLocationIndex++;
      }
      computeCoPPointsForFinalTransfer(copLocationIndex, upcomingFootstepsData.get(footstepIndex - 1).getSupportSide(),
                                       numberOfUpcomingFootsteps < numberFootstepsToConsider.getIntegerValue());
   }

   private void computeCoPPointsForFinalTransfer(int copLocationIndex, RobotSide finalSupportSide, boolean isLastTransfer)
   {
      convertToFramePointRetainingZ(tempFramePoint1, swingFootPredictedFinalPolygon.getCentroid(), worldFrame);
      copLocationWaypoints.get(copLocationIndex).setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
      copLocationWaypoints.get(copLocationIndex).setSupportFootLocation(tempFramePoint1);

      swingFootInitialPolygon.setIncludingFrame(supportFootPolygon);
      supportFootPolygon.setIncludingFrame(swingFootPredictedFinalPolygon);
      int loopEnd = CoPPlanningTools.getCoPPointIndex(transferCoPPointList, endCoPName);
      for (int i = 0; i <= loopEnd; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, transferCoPPointList[i], finalSupportSide);
         copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(transferCoPPointList[i], getTransferSegmentTimes(i), tempPointForCoPCalculation);
      }
      if (isLastTransfer)
      {
         swingFootPredictedFinalPolygon.setIncludingFrame(swingFootInitialPolygon);
         computeMidFeetPointWithChickenSupportForFinalTransfer(tempPointForCoPCalculation);
         tempPointForCoPCalculation.changeFrame(worldFrame);
         copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(CoPPointName.FINAL_COP,
                                                                            getTransferSegmentTimes(loopEnd + 1)
                                                                                  + additionalTimeForFinalTransfer.getDoubleValue(),
                                                                            tempPointForCoPCalculation);
      }
   }

   private void convertToFramePointRetainingZ(FramePoint3D framePointToPack, FramePoint2D framePoint2dToCopy, ReferenceFrame referenceFrameToConvertTo)
   {
      framePointToPack.setIncludingFrame(framePoint2dToCopy, 0.0);
      framePointToPack.changeFrame(referenceFrameToConvertTo);
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the current footstep
    */
   private void computeCoPPointsForFootstep(int copLocationIndex)
   {
      convertToFramePointRetainingZ(tempFramePoint1, swingFootPredictedFinalPolygon.getCentroid(), worldFrame);
      copLocationWaypoints.get(copLocationIndex).setSwingFootLocation(tempFramePoint1);
      convertToFramePointRetainingZ(tempFramePoint1, supportFootPolygon.getCentroid(), worldFrame);
      copLocationWaypoints.get(copLocationIndex).setSupportFootLocation(tempFramePoint1);
      computeCoPPointsForFootstepTransfer(copLocationIndex);
      computeCoPPointsForFootstepSwing(copLocationIndex);
   }

   private void computeCoPPointsForFootstepTransfer(int copLocationsIndex)
   {
      for (int i = 0; i < transferCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, transferCoPPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(transferCoPPointList[i], getTransferSegmentTimes(i), tempPointForCoPCalculation);
      }
   }

   private double getTransferSegmentTimes(int segmentIndex)
   {
      double transferTime = transferDurations.get(footstepIndex).getDoubleValue();
      if(transferTime <= 0.0 || !Double.isFinite(transferTime)) 
         transferTime = defaultTransferTime;
      if (useTransferSplitFractionFor.get(footstepIndex) == UseSplitFractionFor.TIME)
      {
         switch (segmentIndex)
         {
         case 0:
            return transferTime * transferSplitFractions.get(footstepIndex).getDoubleValue();
         case 1:
            return transferTime * (1.0 - transferSplitFractions.get(footstepIndex).getDoubleValue());
         default:
            throw new RuntimeException("For some reason we didn't just use a array that summed to one");
         }
      }
      else
         return transferTime * 0.5;
   }

   private void computeCoPPointsForFootstepSwing(int copLocationsIndex)
   {
      for (int i = 0; i < swingCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempPointForCoPCalculation, swingCoPPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(swingCoPPointList[i], getSwingSegmentTimes(i), tempPointForCoPCalculation);
      }
      computeCoPPointLocation(tempPointForCoPCalculation, swingCoPPointList[swingCoPPointList.length - 1],
                              upcomingFootstepsData.get(footstepIndex).getSupportSide());
      copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(swingCoPPointList[swingCoPPointList.length - 1],
                                                                          getSwingSegmentTimes(swingCoPPointList.length), tempPointForCoPCalculation);
   }

   private double getSwingSegmentTimes(int segmentIndex)
   {
      double swingTime = swingDurations.get(footstepIndex).getDoubleValue();
      if(swingTime <= 0.0 || !Double.isFinite(swingTime)) 
         swingTime = defaultSwingTime;
      switch (segmentIndex)
      {
      case 0:
         return swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue()
               * swingSplitFractions.get(footstepIndex).getDoubleValue();
      case 1:
         return swingTime * swingDurationShiftFractions.get(footstepIndex).getDoubleValue()
               * (1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue());
      case 2:
         return swingTime * (1.0 - swingDurationShiftFractions.get(footstepIndex).getDoubleValue());
      default:
         throw new RuntimeException("For some reason we didn't just use a array that summed to one here as well");
      }
   }

   private void computeCoPPointLocation(FramePoint3D copPointToPlan, CoPPointName copPointName, RobotSide supportSide)
   {
      if (copPointName == exitCoPName && setExitCoPUnderSpecialCases(copPointToPlan, supportSide))
         return;
      setCoPPointInPolygon(copPointToPlan, copPointName);
      this.tempDouble = copOffsets.get(supportSide).get(copPointName).getX() + getStepLengthToCoPOffset(copPointName);
      if (isConstrainedToMinMaxFlags.get(copPointName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());
      copPointToPlan.add(this.tempDouble, copOffsets.get(supportSide).get(copPointName).getY(), 0.0);
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainCoPPointToSupportPolygon(copPointToPlan, copPointName);
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
                              copOffsets.get(supportSide).get(exitCoPName).getY(), 0.0);
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

   private double getStepLengthToCoPOffset(CoPPointName coPPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(coPPointName))
      {
      //TODO should all the calculations be done with respect to the support polygon
      case INITIAL_SWING_POLYGON:
         swingFootInitialPolygon.getFrameVertex(swingFootInitialPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempFramePoint1, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_SWING_POLYGON:
         swingFootPredictedFinalPolygon.getFrameVertex(swingFootPredictedFinalPolygon.getMaxXMaxYIndex(), tempFramePoint2d);
         convertToFramePointRetainingZ(tempFramePoint1, tempFramePoint2d, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempFramePoint1, stepLengthToCoPOffsetFactors.get(coPPointName));
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootInitialPolygon, supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         getDoubleSupportPolygonCentroid(tempDoubleSupportPolygonCentroid, supportFootPolygon, swingFootPredictedFinalPolygon,
                                         supportFootPolygon.getReferenceFrame());
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygonCentroid, stepLengthToCoPOffsetFactors.get(coPPointName));
      default:
         return 0.0;
      }
   }

   private void setCoPPointInPolygon(FramePoint3D copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
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

   private double getStepLengthBasedOffset(FrameConvexPolygon2d supportPolygon, FramePoint3D referencePoint, double stepLengthToCoPOffsetFactor)
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
    * 
    * @param copPointToConstrain
    * @param copPointName
    */
   private void constrainCoPPointToSupportPolygon(FramePoint3D copPointToConstrain, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
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
         throw new RuntimeException("Invalid constraining frame defined for " + copPointName.toString());
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
   private void updateFootPolygons(RobotSide supportSide)
   {
      if (!(upcomingFootstepsData.size() == 0) && footstepIndex >= upcomingFootstepsData.size())
         throw new RuntimeException("CoP planner: Attempting to plan for footstep index, " + footstepIndex + " with only " + upcomingFootstepsData.size()
               + " upcoming footsteps");
      else if (footstepIndex == plannedFootstepIndex)
         return;

      switch (footstepIndex)
      {
      case 0:
         initializeFootPolygons(supportSide);
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
   private void initializeFootPolygons(RobotSide defaultSupportSide)
   {
      if (upcomingFootstepsData.size() == 0)
      {
         setFootPolygonFromCurrentState(swingFootInitialPolygon, defaultSupportSide.getOppositeSide());
         swingFootPredictedFinalPolygon.setIncludingFrame(swingFootInitialPolygon);
         setFootPolygonFromCurrentState(supportFootPolygon, defaultSupportSide);
         return;
      }

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
      framePolygonToPack.clear(upcomingFootstepsData.get(footstepIndex).getFootstep().getSoleReferenceFrame());
      if (footstepIndex < upcomingFootstepsData.size() && upcomingFootstepsData.get(footstepIndex).getFootstep() != null
            && upcomingFootstepsData.get(footstepIndex).getFootstep().getPredictedContactPoints() != null
            && upcomingFootstepsData.get(footstepIndex).getFootstep().getPredictedContactPoints().size() > 0)
      {
         polygonReference.clear();
         polygonReference.addVertices(upcomingFootstepsData.get(footstepIndex).getFootstep().getPredictedContactPoints(),
                                      upcomingFootstepsData.get(footstepIndex).getFootstep().getPredictedContactPoints().size());
         polygonReference.update();
         framePolygonToPack.addVertices(polygonReference);
      }
      else
         framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepsData.get(footstepIndex).getSwingSide()));
      framePolygonToPack.update();
   }

   private void setFootPolygonFromCurrentState(FrameConvexPolygon2d framePolygonToPack, RobotSide robotSide)
   {
      if (!supportFootPolygonsInSoleZUpFrames.get(robotSide).isEmpty())
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

   private WalkingTrajectoryType trajectoryType = WalkingTrajectoryType.TRANSFER;
   private double timeInState = 0.0;
   private int transferTrajectoryIndex = -1;
   private int swingTrajectoryIndex = -1;
   private List<CoPPointName> copList;

   private void generateCoPTrajectoriesFromWayPoints()
   {
      //It is always guaranteed that the initial state will be transfer the way this code is written. This is needed for the angular momentum approximation to work
      tempFramePoint1.setToNaN(worldFrame);
      trajectoryType = WalkingTrajectoryType.TRANSFER;
      copList = null;
      timeInState = 0.0;
      transferTrajectoryIndex = -1;
      swingTrajectoryIndex = -1;

      for (int waypointIndex = 0; waypointIndex < copLocationWaypoints.size()
            && !copLocationWaypoints.get(waypointIndex).getCoPPointList().isEmpty(); waypointIndex++)
      {
         copList = copLocationWaypoints.get(waypointIndex).getCoPPointList();
         for (int segmentIndex = 0; segmentIndex < copList.size(); segmentIndex++)
         {
            CoPTrajectoryPoint currentPoint = copLocationWaypoints.get(waypointIndex).get(segmentIndex);
            if (!tempFramePoint1.containsNaN())
            {
               if (trajectoryType == WalkingTrajectoryType.SWING)
               {
                  swingCoPTrajectories.get(swingTrajectoryIndex).setSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1,
                                                                            currentPoint.getPosition().getFrameTuple());
               }
               else
               {
                  transferCoPTrajectories.get(transferTrajectoryIndex).setSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint1,
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
      return activeTrajectory.getNodeTimes()[activeTrajectory.getNumberOfSegments()];
   }
}