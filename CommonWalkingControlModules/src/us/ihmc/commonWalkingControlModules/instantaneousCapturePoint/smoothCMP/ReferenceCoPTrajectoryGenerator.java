package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters.CoPSupportPolygonNames;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
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

//TODO 1) add isDoneWalking functionality

public class ReferenceCoPTrajectoryGenerator implements ReferenceCoPTrajectoryGeneratorInterface
{
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double COP_POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   // Waypoint planning parameters
   private final SideDependentList<EnumMap<CoPPointName, YoFrameVector2d>> copOffsets = new SideDependentList<>();
   private EnumMap<CoPPointName, CoPSupportPolygonNames> copSupportPolygon;
   private EnumMap<CoPPointName, Boolean> isConstrainedToSupportPolygonFlags;

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

   private final YoEnum<CoPSplineType> orderOfSplineInterpolation;

   private final YoBoolean isDoneWalking;
   private final YoBoolean holdDesiredState;
   private final YoBoolean putExitCoPOnToes;

   // Output variables
   private final List<CoPPointsInFoot> copLocationWaypoints = new ArrayList<>();
   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   // Runtime variables
   private FramePoint desiredCoPPosition = new FramePoint();
   private FrameVector desiredCoPVelocity = new FrameVector();
   private FrameVector desiredCoPAcceleration = new FrameVector();
   private FramePoint heldCoPPosition = new FramePoint();

   private int footstepIndex = 0;
   private int plannedFootstepIndex = -1;
   private int numberOfSwingSegments = 10;
   private int numberOfTransferSegments = 10;
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FrameConvexPolygon2d tempDoubleSupportPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d swingFootPredictedFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private double tempDouble;
   private FootstepData currentFootstepData;
   private FrameConvexPolygon2d framePolygonReference;
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private FramePoint tempFramePoint = new FramePoint();
   private FramePoint2d tempFramePoint2d = new FramePoint2d();

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

      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);
      this.holdDesiredState = new YoBoolean(namePrefix + "HoldDesiredState", parentRegistry);
      this.putExitCoPOnToes = new YoBoolean(namePrefix + "PutExitCoPOnToes", parentRegistry);

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
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(),
                                                                                 this.numberOfTransferSegments, registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(), this.numberOfSwingSegments,
                                                                        registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, maxNumberOfFootstepsToConsider,
                                                                              orderOfSplineInterpolation.getEnumValue(), this.numberOfTransferSegments,
                                                                              registry);
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
      for (int waypointNumber = 0; waypointNumber < parameters.getCoPPointsToPlan().length; waypointNumber++)
         setSymmetricCoPConstantOffsets(parameters.getCoPPointsToPlan()[waypointNumber], copOffsets.get(parameters.getCoPPointsToPlan()[waypointNumber]));

      EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds = parameters.getCoPForwardOffsetBoundsInFoot();
      for (int waypointIndex = 0; waypointIndex < parameters.getCoPPointsToPlan().length; waypointIndex++)
      {
         Vector2D bounds = copForwardOffsetBounds.get(parameters.getCoPPointsToPlan()[waypointIndex]);
         minCoPOffsets.get(parameters.getCoPPointsToPlan()[waypointIndex]).set(bounds.getX());
         maxCoPOffsets.get(parameters.getCoPPointsToPlan()[waypointIndex]).set(bounds.getY());
      }
   }

   public void holdPosition()
   {
      holdDesiredState.set(true);
      heldCoPPosition.setIncludingFrame(desiredCoPPosition);
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
         List<CoPPointName> copPointNames = copPointsInFoot.getCoPPointList();
         for (int i = 0; i < copPointNames.size(); i++)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(footIndex + "Foot CoP Waypoint" + copPointNames.get(i).toString(),
                                                             copPointsInFoot.getWaypointInWorldFrameReadOnly(i), COP_POINT_SIZE, YoAppearance.Green(),
                                                             GraphicType.BALL);
            yoGraphicsList.add(copViz);
            artifactList.add(copViz.createArtifact());
         }
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
      clearHeldPosition();
      clearPlan();
   }

   public void clearPlan()
   {
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
      this.initialTime = currentTime;
      this.activeTrajectory = swingCoPTrajectories.get(0);
   }

   @Override
   public void update(double currentTime)
   {
      if (activeTrajectory != null)
         activeTrajectory.update(currentTime - this.initialTime, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(desiredCoPPosition);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
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
      updateFootPolygons();
      // Put first CoP as per chicken support computations in case starting from rest
      if (atAStop || numberOfUpcomingFootsteps.getIntegerValue() == 0)
      {
         updateDoubleSupportPolygon(supportFootPolygon, swingFootInitialPolygon);
         if (holdDesiredState.getBooleanValue())
            tempFramePoint.setIncludingFrame(heldCoPPosition);
         else
            computeMidFeetPointWithChickenSupportForInitialTransfer(tempFramePoint);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(CoPPointName.START_COP, 0.0, tempFramePoint);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(supportFootPolygon.getCentroid());
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(swingFootInitialPolygon.getCentroid());
         computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      }
      // Put first CoP at the exitCoP of the swing foot if not starting from rest 
      else
      {
         computeCoPPointLocationForPreviousPlan(tempFramePoint2d, exitCoPName, upcomingFootstepsData.get(footstepIndex).getSwingSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempFramePoint);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(supportFootPolygon.getCentroid());
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(swingFootInitialPolygon.getCentroid());
         computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      }
      generateCoPTrajectoriesFromWayPoints();
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      footstepIndex = 0;
      updateFootPolygons();
      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
         return;
      else
      {
         computeCoPPointLocationForPreviousPlan(tempFramePoint2d, exitCoPName, upcomingFootstepsData.get(footstepIndex).getSwingSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPName, 0.0, tempFramePoint);
         copLocationWaypoints.get(footstepIndex).setSwingFootLocation(supportFootPolygon.getCentroid());
         copLocationWaypoints.get(footstepIndex).setSupportFootLocation(swingFootInitialPolygon.getCentroid());
         computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
      }
      generateCoPTrajectoriesFromWayPoints();
   }

   private void computeMidFeetPointWithChickenSupportForInitialTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, swingFootInitialPolygon, supportFootPolygon);
   }

   private void computeMidFeetPointWithChickenSupportForFinalTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, supportFootPolygon, swingFootPredictedFinalPolygon);
   }

   /**
    * Assumes foot polygon and double support polygon has been updated
    * @param framePointToPack
    * @param transferToSide
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint framePointToPack, FrameConvexPolygon2d supportFootPolygon,
                                                      FrameConvexPolygon2d swingFootPolygon)
   {
      computeMidFeetPointByPositionFraction(tempFramePoint2d, supportFootPolygon, swingFootPolygon, percentageChickenSupport.getDoubleValue());
      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   private void computeMidFeetPointByPositionFraction(FramePoint2d framePointToPack, FrameConvexPolygon2d footPolygonA,
                                                      FrameConvexPolygon2d footPolygonB, double fraction)
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
      this.tempDoubleSupportPolygon.changeFrame(worldFrame);
      this.framePolygonReference.changeFrame(worldFrame);
      framePointToPack.interpolate(this.tempDoubleSupportPolygon.getCentroid(), this.framePolygonReference.getCentroid(), this.tempDouble);
   }

   private void computeCoPPointLocationForPreviousPlan(FramePoint2d framePointToPack, CoPPointName copPointName, RobotSide swingSide)
   {
      if (copPointName == exitCoPName && setInitialExitCoPUnderSpecialCases(framePointToPack, swingSide))
         return;
      setInitialCoPPointToPolygonOrigin(framePointToPack, copPointName);
      this.tempDouble = copOffsets.get(swingSide).get(copPointName).getX() + getInitialStepLengthToCoPOffset(copPointName);

      if (isConstrainedToMinMaxFlags.get(copPointName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());
      framePointToPack.add(this.tempDouble, copOffsets.get(swingSide).get(copPointName).getY());
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainInitialCoPPointToSupportPolygon(framePointToPack, copPointName);
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
         return getStepLengthBasedOffset(swingFootInitialPolygon, supportFootPolygon, stepLengthToCoPOffsetFactors.get(copPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(swingFootInitialPolygon, supportFootPolygon);
         return getStepLengthBasedOffset(swingFootInitialPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(copPointName));
      case SUPPORT_FOOT_POLYGON:
      case NULL:
      default:
         return 0.0;
      }
   }

   private void constrainInitialCoPPointToSupportPolygon(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to constrain initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         constrainToPolygon(tempFramePoint2d, supportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(swingFootInitialPolygon, supportFootPolygon);
         constrainToPolygon(tempFramePoint2d, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         constrainToPolygon(tempFramePoint2d, swingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
      }
   }

   private void setInitialCoPPointToPolygonOrigin(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case NULL:
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
      case INITIAL_SWING_POLYGON:
         throw new RuntimeException("Unable to compute initial exit CoP using given parameters");
      case FINAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(supportFootPolygon.getCentroid());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(swingFootInitialPolygon, supportFootPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case SUPPORT_FOOT_POLYGON:
      default:
         copPointToPlan.setIncludingFrame(swingFootInitialPolygon.getCentroid());
      }
   }

   private void computeCoPPointsForUpcomingFootsteps(int copLocationIndex)
   {
      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getIntegerValue());
      for (int i = 0; i < numberOfUpcomingFootsteps; i++)
      {
         updateFootPolygons();
         computeCoPPointsForFootstep(copLocationIndex);
         footstepIndex++;
         copLocationIndex++;
      }
      computeCoPPointsForFinalTransfer(copLocationIndex);
   }

   private void computeCoPPointsForFinalTransfer(int copLocationIndex)
   {
      footstepIndex--;
      copLocationWaypoints.get(copLocationIndex).setSwingFootLocation(swingFootPredictedFinalPolygon.getCentroid());
      copLocationWaypoints.get(copLocationIndex).setSupportFootLocation(supportFootPolygon.getCentroid());
      int loopEnd = CoPPlanningTools.getCoPPointIndex(transferCoPPointList, endCoPName);
      for (int i = 0; i < loopEnd; i++)
      {
         computeCoPPointLocation(tempFramePoint2d, transferCoPPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(transferCoPPointList[i], getTransferSegmentTimes(i), tempFramePoint);
      }
      updateDoubleSupportPolygon(supportFootPolygon, swingFootPredictedFinalPolygon);
      computeMidFeetPointWithChickenSupportForFinalTransfer(tempFramePoint);
      copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(endCoPName,
                                                                         getTransferSegmentTimes(loopEnd) + additionalTimeForFinalTransfer.getDoubleValue(),
                                                                         tempFramePoint);
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the current footstep
    */
   private void computeCoPPointsForFootstep(int copLocationIndex)
   {
      copLocationWaypoints.get(copLocationIndex).setSwingFootLocation(swingFootPredictedFinalPolygon.getCentroid());
      copLocationWaypoints.get(copLocationIndex).setSupportFootLocation(supportFootPolygon.getCentroid());
      computeCoPPointsForFootstepTransfer(copLocationIndex);
      computeCoPPointsForFootstepSwing(copLocationIndex);
   }

   private void computeCoPPointsForFootstepTransfer(int copLocationsIndex)
   {
      for (int i = 0; i < transferCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempFramePoint2d, transferCoPPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(transferCoPPointList[i], getTransferSegmentTimes(i), tempFramePoint);
      }
   }

   private double getTransferSegmentTimes(int segmentIndex)
   {
      switch (segmentIndex)
      {
      case 0:
         return transferDurations.get(footstepIndex).getDoubleValue() * transferSplitFractions.get(footstepIndex).getDoubleValue();
      case 1:
         return transferDurations.get(footstepIndex).getDoubleValue() * (1.0 - transferSplitFractions.get(footstepIndex).getDoubleValue());
      default:
         throw new RuntimeException("For some reason we didn't just use a array that summed to one");
      }
   }

   private void computeCoPPointsForFootstepSwing(int copLocationsIndex)
   {
      for (int i = 0; i < swingCoPPointList.length; i++)
      {
         computeCoPPointLocation(tempFramePoint2d, swingCoPPointList[i], upcomingFootstepsData.get(footstepIndex).getSupportSide());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(swingCoPPointList[i], getSwingSegmentTimes(i), tempFramePoint);
      }
      computeCoPPointLocation(tempFramePoint2d, swingCoPPointList[swingCoPPointList.length - 1], upcomingFootstepsData.get(footstepIndex).getSupportSide());
      tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
      copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(swingCoPPointList[swingCoPPointList.length - 1],
                                                                          getSwingSegmentTimes(swingCoPPointList.length), tempFramePoint);
   }

   private double getSwingSegmentTimes(int segmentIndex)
   {
      switch (segmentIndex)
      {
      case 0:
         return swingDurations.get(footstepIndex).getDoubleValue() * swingDurationShiftFractions.get(footstepIndex).getDoubleValue()
               * swingSplitFractions.get(footstepIndex).getDoubleValue();
      case 1:
         return swingDurations.get(footstepIndex).getDoubleValue() * swingDurationShiftFractions.get(footstepIndex).getDoubleValue()
               * (1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue());
      case 2:
         return swingDurations.get(footstepIndex).getDoubleValue() * (1.0 - swingDurationShiftFractions.get(footstepIndex).getDoubleValue());
      default:
         throw new RuntimeException("For some reason we didn't just use a array that summed to one here as well");
      }
   }

   private void computeCoPPointLocation(FramePoint2d copPointToPlan, CoPPointName copPointName, RobotSide supportSide)
   {
      if (copPointName == exitCoPName && setExitCoPUnderSpecialCases(copPointToPlan, supportSide))
         return;
      setCoPPointInPolygon(copPointToPlan, copPointName);
      this.tempDouble = copOffsets.get(supportSide).get(copPointName).getX() + getStepLengthToCoPOffset(copPointName);
      if (isConstrainedToMinMaxFlags.get(copPointName))
         this.tempDouble = MathTools.clamp(this.tempDouble, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());
      copPointToPlan.add(this.tempDouble, copOffsets.get(supportSide).get(copPointName).getY());
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainCoPPointToSupportPolygon(copPointToPlan, copPointName);
   }

   private boolean setInitialExitCoPUnderSpecialCases(FramePoint2d framePointToPack, RobotSide supportSide)
   {
      return setExitCoPUnderSpecialCases(framePointToPack, swingFootInitialPolygon, supportFootPolygon, supportSide);
   }
   
   private boolean setExitCoPUnderSpecialCases(FramePoint2d framePointToPack, RobotSide supportSide)
   {
      return setExitCoPUnderSpecialCases(framePointToPack, supportFootPolygon, swingFootPredictedFinalPolygon, supportSide);
   }
   
   private boolean setExitCoPUnderSpecialCases(FramePoint2d framePointToPack, FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootFinalPolygon, RobotSide supportSide)
   {
      double supportToSwingStepLength = swingFootFinalPolygon.getCentroid().getX()
            - supportFootPolygon.getCentroid().getX();
      tempFramePoint.setXYIncludingFrame(swingFootFinalPolygon.getCentroid());
      tempFramePoint.changeFrame(supportFootPolygon.getReferenceFrame());
      double supportToSwingStepHeight = tempFramePoint.getZ();
      if (supportFootPolygon.getArea() == 0.0)
      {
         framePointToPack.setToZero(supportFootPolygon.getReferenceFrame());
         framePointToPack.set(supportFootPolygon.getVertex(0));
         return true;
      }
      else if (putExitCoPOnToes.getBooleanValue() &&  supportToSwingStepLength > footstepLengthThresholdToPutExitCoPOnToes.getDoubleValue())
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid());
         framePointToPack.add(supportFootPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue(), copOffsets.get(supportSide).get(exitCoPName).getY());
         return true;
      }
      else if (-supportToSwingStepHeight > footstepHeightThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue() && supportToSwingStepLength > footstepLengthThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue())
      {
         framePointToPack.setIncludingFrame(supportFootPolygon.getCentroid());
         framePointToPack.add(supportFootPolygon.getMaxX(), 0.0);
         constrainToPolygon(framePointToPack, supportFootPolygon, safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getDoubleValue());
         return true;
      }
      else 
         return false;
   }
   
   private double getStepLengthToCoPOffset(CoPPointName coPPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(coPPointName))
      {
      case INITIAL_SWING_POLYGON:
         return getStepLengthBasedOffset(supportFootPolygon, swingFootInitialPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_SWING_POLYGON:
         return getStepLengthBasedOffset(supportFootPolygon, swingFootPredictedFinalPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(supportFootPolygon, swingFootInitialPolygon);
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(supportFootPolygon, swingFootPredictedFinalPolygon);
         return getStepLengthBasedOffset(supportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      default:
         return 0.0;
      }
   }

   private void setCoPPointInPolygon(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case INITIAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(swingFootInitialPolygon.getCentroid());
         return;
      case FINAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(swingFootPredictedFinalPolygon.getCentroid());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(supportFootPolygon, swingFootInitialPolygon);
         if(copPointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointByPositionFraction(copPointToPlan, swingFootInitialPolygon, supportFootPolygon, transferSplitFractions.get(footstepIndex).getDoubleValue());
         else
            copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(supportFootPolygon, swingFootPredictedFinalPolygon);
         if(copPointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointByPositionFraction(copPointToPlan, supportFootPolygon, swingFootPredictedFinalPolygon, transferSplitFractions.get(footstepIndex).getDoubleValue());
         else
            copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case NULL:
         throw new RuntimeException("No frame defined for CoP point:" + copPointName.toString());
      default:
         copPointToPlan.setIncludingFrame(supportFootPolygon.getCentroid());
         return;
      }
   }

   private double getStepLengthBasedOffset(FrameConvexPolygon2d supportPolygon, FrameConvexPolygon2d referencePolygon, double stepLengthToCoPOffsetFactor)
   {
      return stepLengthToCoPOffsetFactor * (referencePolygon.getCentroid().getX() - supportPolygon.getCentroid().getX());
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by projection
    * @param copPointToConstrain
    * @param supportFoot
    * @param safeDistanceFromSupportPolygonEdges
    */
   private void constrainToPolygon(FramePoint2d copPointToConstrain, FrameConvexPolygon2d supportPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonScaler.scaleConvexPolygon(supportPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      tempPolygon.orthogonalProjection(copPointToConstrain);
   }

   private void constrainCoPPointToSupportPolygon(FramePoint2d copPointToConstrain, CoPPointName copPointName)
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
         updateDoubleSupportPolygon(supportFootPolygon, swingFootInitialPolygon);
         constrainToPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(supportFootPolygon, swingFootPredictedFinalPolygon);
         constrainToPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         return;
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
   private void updateDoubleSupportPolygon(FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon)
   {
      tempDoubleSupportPolygon.setIncludingFrame(supportFootPolygon);
      tempDoubleSupportPolygon.changeFrame(worldFrame);
      tempPolygon.setIncludingFrame(swingFootPolygon);
      tempPolygon.changeFrame(worldFrame);
      tempDoubleSupportPolygon.addVertices(tempPolygon);
      tempDoubleSupportPolygon.update();
   }

   /**
    * Updates the swing and support foot polygons based on footstepIndex
    * <p> Has no memory of the previous state so should be used carefully </p>
    */
   private void updateFootPolygons()
   {
      if (footstepIndex == plannedFootstepIndex)
         return;

      switch (footstepIndex)
      {
      case 0:
         initializeFootPolygons();
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
   private void initializeFootPolygons()
   {
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
      framePolygonToPack.addVertices(defaultFootPolygons.get(upcomingFootstepsData.get(footstepIndex).getSwingSide()));
      framePolygonToPack.update();
      framePolygonToPack.changeFrame(worldFrame);
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
      framePolygonToPack.changeFrame(worldFrame);
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

   private WalkingTrajectoryType trajectoryType = WalkingTrajectoryType.TRANSFER;
   private double timeInState = 0.0;
   private int transferTrajectoryIndex = -1;
   private int swingTrajectoryIndex = -1;
   private List<CoPPointName> copList;

   private void generateCoPTrajectoriesFromWayPoints()
   {
      //It is always guaranteed that the initial state will be transfer the way this code is written. This is needed for the angular momentum approximation to work
      tempFramePoint.setToNaN();
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
            if (!tempFramePoint.containsNaN())
            {
               if (trajectoryType == WalkingTrajectoryType.SWING)
                  swingCoPTrajectories.get(swingTrajectoryIndex).setSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint,
                                                                            currentPoint.getPosition().getFrameTuple());
               else
                  transferCoPTrajectories.get(transferTrajectoryIndex).setSegment(timeInState, timeInState + currentPoint.getTime(), tempFramePoint,
                                                                                  currentPoint.getPosition().getFrameTuple());
            }
            else
            {
               transferTrajectoryIndex++;
               currentPoint.getPosition(tempFramePoint);
               continue;
            }
            currentPoint.getPosition(tempFramePoint);

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
}