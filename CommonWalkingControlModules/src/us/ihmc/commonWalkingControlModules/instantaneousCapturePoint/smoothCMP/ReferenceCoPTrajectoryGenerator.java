package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.awt.*;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPPlanningTools;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CoPPolynomialTrajectoryPlannerInterface;
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

//TODO 1) Modify initializeParamters() to have only things that should be adjusted on the fly there

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

   private final EnumMap<CoPPointName, YoDouble> maxCoPOffsets = new EnumMap<>(CoPPointName.class);
   private final EnumMap<CoPPointName, YoDouble> minCoPOffsets = new EnumMap<>(CoPPointName.class);

   private EnumMap<CoPPointName, Double> stepLengthToCoPOffsetFactors;
   private EnumMap<CoPPointName, CoPSupportPolygonNames> stepLengthOffsetReferencePolygons;

   private final YoDouble footstepHeightThresholdToPutExitCoPOnToesSteppingDown;
   private final YoDouble footstepLengthThresholdToPutExitCoPOnToesSteppingDown;
   private final YoDouble footstepLengthThresholdToPutExitCoPOnToes;

   private final YoBoolean putExitCoPOnToes;
   private final YoDouble exitCoPForwardSafetyMarginOnToes;

   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble safeDistanceFromCoPToSupportEdgesWhenSteppingDown;

   private final YoDouble percentageChickenSupport;

   private CoPPointName entryCoPName;
   private CoPPointName exitCoPName;
   private CoPPointName endCoPName;

   private CoPPointName[] swingCopPointList;
   private CoPPointName[] transferCopPointList;

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

   // Output variables
   private final YoBoolean isDoneWalking;
   private final List<CoPPointsInFoot> copLocationWaypoints = new ArrayList<>();
   private final List<TransferCoPTrajectory> transferCoPTrajectories = new ArrayList<>();
   private final List<SwingCoPTrajectory> swingCoPTrajectories = new ArrayList<>();

   // Runtime variables
   private FramePoint desiredCoPPosition = new FramePoint();
   private FrameVector desiredCoPVelocity = new FrameVector();
   private FrameVector desiredCoPAcceleration = new FrameVector();
   private int footstepIndex = 0;
   private int plannedFootstepIndex = -1;
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FrameConvexPolygon2d tempDoubleSupportPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSupportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private FootstepData currentFootstepData;
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private FramePoint tempFramePoint = new FramePoint();
   private FramePoint2d tempFramePoint2d = new FramePoint2d();

   // Input data
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<FootstepData>(4,
                                                                                                               FootstepData.class);
   /**
    *
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCoPTrajectoryGenerator(String namePrefix, int numberOfPointsPerFoot, int maxNumberOfFootstepsToConsider,
                                          BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                          YoInteger numberFootstepsToConsider, List<YoDouble> swingDurations, List<YoDouble> transferDurations,
                                          List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions,
                                          List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.numberFootstepsToConsider = numberFootstepsToConsider;

      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdgesWhenSteppingDown", registry);
      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);

      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;

      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      footstepHeightThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepHeightThresholdToPutExitCoPOnToesSteppingDown", registry);
      footstepLengthThresholdToPutExitCoPOnToesSteppingDown = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCoPOnToesSteppingDown", registry);

      footstepLengthThresholdToPutExitCoPOnToes = new YoDouble(namePrefix + "FootstepLengthThresholdToPutExitCoPOnToes", registry);
      putExitCoPOnToes = new YoBoolean(namePrefix + "PutExitCoPOnToes", registry);
      exitCoPForwardSafetyMarginOnToes = new YoDouble(namePrefix + "ExitCoPForwardSafetyMarginOnToes", registry);

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


      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT)};
      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      for (int i = 0; i < maxNumberOfFootstepsToConsider + 1; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i, framesToRegister, registry));
      }

      parentRegistry.addChild(registry);
      clear();
   }

   @Override
   public void initializeParameters(SmoothCMPPlannerParameters parameters)
   {
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());
      safeDistanceFromCoPToSupportEdgesWhenSteppingDown.set(parameters.getCoPSafeDistanceAwayFromToesWhenSteppingDown());
      numberOfPointsPerFoot.set(parameters.getNumberOfCoPWayPointsPerFoot());
      orderOfSplineInterpolation.set(parameters.getOrderOfCoPInterpolation());
      percentageChickenSupport.set(0.5);

      this.copSupportPolygon = parameters.getSupportPolygonNames();
      this.isConstrainedToSupportPolygonFlags = parameters.getIsConstrainedToSupportPolygonFlags();
      this.isConstrainedToMinMaxFlags = parameters.getIsConstrainedToMinMaxFlags();
      this.stepLengthToCoPOffsetFactors = parameters.getStepLengthToCoPOffsetFactors();
      this.stepLengthOffsetReferencePolygons = parameters.getStepLengthToCoPOffsetFlags();

      this.entryCoPName = parameters.getEntryCoPName();
      this.exitCoPName = parameters.getExitCoPName();
      this.endCoPName = parameters.getEndCoPName();

      this.swingCopPointList = parameters.getSwingCoPPointsToPlan();
      this.transferCopPointList = parameters.getTransferCoPPointsToPlan();

      footstepHeightThresholdToPutExitCoPOnToesSteppingDown.set(parameters.getStepHeightThresholdForExitCoPOnToesWhenSteppingDown());
      footstepLengthThresholdToPutExitCoPOnToesSteppingDown.set(parameters.getStepLengthThresholdForExitCoPOnToesWhenSteppingDown());

      footstepLengthThresholdToPutExitCoPOnToes.set(parameters.getStepLengthThresholdForExitCoPOnToes());
      putExitCoPOnToes.set(parameters.putExitCoPOnToes());
      exitCoPForwardSafetyMarginOnToes.set(parameters.getExitCoPForwardSafetyMarginOnToes());

      EnumMap<CoPPointName, Vector2D> copOffsets = parameters.getCopOffsetsInFootFrame();
      for (CoPPointName name : CoPPointName.values)
         setSymmetricCoPConstantOffsets(name, copOffsets.get(name));

      EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds = parameters.getCoPForwardOffsetBoundsInFoot();
      for (CoPPointName pointName : CoPPointName.values)
      {
         Vector2D bounds = copForwardOffsetBounds.get(pointName);
         if (bounds != null)
         {
            minCoPOffsets.get(pointName).set(bounds.getX());
            maxCoPOffsets.get(pointName).set(bounds.getY());
         }
      }


      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(),
                                                                                 transferCopPointList.length + 1, registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, orderOfSplineInterpolation.getEnumValue(), swingCopPointList.length + 2,
                                                                        registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, numberFootstepsToConsider.getIntegerValue(),
                                                                              orderOfSplineInterpolation.getEnumValue(), transferCopPointList.length + 1,
                                                                              registry);
      transferCoPTrajectories.add(transferCoPTrajectory);
   }

   @Override
   public void setSymmetricCoPConstantOffsets(CoPPointName name, Vector2D offset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copOffsets.get(robotSide).get(name);
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
                                                             copPointsInFoot.getWaypointInWorldFrameReadOnly(i), COP_POINT_SIZE,
                                                             YoAppearance.Green(), GraphicType.BALL);
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
      double timeInState = currentTime - initialTime;
      if (activeTrajectory != null)
         activeTrajectory.update(timeInState, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
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

      boolean noUpcomingFootsteps = numberOfUpcomingFootsteps.isZero();
      isDoneWalking.set(noUpcomingFootsteps);

      CoPPointsInFoot copWaypoints = copLocationWaypoints.get(footstepIndex);
      updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);

      if (atAStop || noUpcomingFootsteps) // Put first CoP as per chicken support computations in case starting from rest
         computeMidFeetPointWithChickenSupportForInitialTransfer(tempFramePoint);
      else // Put first CoP at the exitCoP of the swing foot if not starting from rest
         computePreviousExitCoP(tempFramePoint, upcomingFootstepsData.get(footstepIndex).getSwingSide());
      copWaypoints.addAndSetIncludingFrame(exitCoPName, 0.0, tempFramePoint);

      computeCoPPointsForTransfer(footstepIndex);
      computeCoPPointsForUpcomingFootsteps();

      generateCoPTrajectoriesFromWayPoints(WalkingTrajectoryType.TRANSFER);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      footstepIndex = 0;
      updateFootPolygons();

      boolean onlyOneUpcomingFootstep = numberOfUpcomingFootsteps.getIntegerValue() == 1;
      isDoneWalking.set(onlyOneUpcomingFootstep);

      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
         throw new RuntimeException("There must be a footstep to be in single support.");
      else
      {
         computeCoPPointLocation(tempFramePoint, entryCoPName, upcomingFootstepsData.get(footstepIndex).getSwingSide().getOppositeSide());
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(entryCoPName, 0.0, tempFramePoint);

         computeCoPPointsForUpcomingFootsteps();
      }
      generateCoPTrajectoriesFromWayPoints(WalkingTrajectoryType.SWING);
   }

   private void computeMidFeetPointWithChickenSupportForInitialTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, currentSupportFootPolygon, currentSwingFootInitialPolygon);
   }

   private void computeMidFeetPointWithChickenSupportForFinalTransfer(FramePoint framePointToPack)
   {
      computeMidFeetPointWithChickenSupport(framePointToPack, currentSupportFootPolygon, currentSwingFootFinalPolygon);
   }

   /**
    * Assumes foot polygon and double support polygon has been updated
    * @param framePointToPack
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint framePointToPack, FrameConvexPolygon2d supportFootPolygon,
                                                      FrameConvexPolygon2d swingFootPolygon)
   {
      computeMidFeetPointWithFraction(framePointToPack, supportFootPolygon, swingFootPolygon, percentageChickenSupport.getDoubleValue());
   }

   private void computeMidFeetPointWithFraction(FramePoint framePointToPack, FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon,
                                                double fraction)
   {
      fraction = MathTools.clamp(fraction, 0.0, 1.0);

      FrameConvexPolygon2d framePolygonReference;
      if (fraction < 0.5)
      {
         fraction *= 2.0;
         framePolygonReference = supportFootPolygon;
      }
      else
      {
         fraction = (0.5 - fraction) * 2.0;
         framePolygonReference = swingFootPolygon;
      }
      tempDoubleSupportPolygon.changeFrame(worldFrame);
      framePolygonReference.changeFrame(worldFrame);

      tempFramePoint2d.interpolate(tempDoubleSupportPolygon.getCentroid(), framePolygonReference.getCentroid(), fraction);
      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   private void computePreviousExitCoP(FramePoint framePointToPack, RobotSide supportSide)
   {
      tempFramePoint2d.set(currentSwingFootInitialPolygon.getCentroid());

      // Determine the forward offset location of the CoP waypoint in the foot
      double forwardOffset = copOffsets.get(supportSide).get(exitCoPName).getX() + getStepLengthToCoPOffset(exitCoPName);
      if (isConstrainedToMinMaxFlags.get(exitCoPName))
         forwardOffset = MathTools.clamp(forwardOffset, minCoPOffsets.get(exitCoPName).getDoubleValue(), maxCoPOffsets.get(exitCoPName).getDoubleValue());

      tempFramePoint2d.add(forwardOffset, copOffsets.get(supportSide).get(exitCoPName).getY());
      if (isConstrainedToSupportPolygonFlags.get(exitCoPName))
         constrainToSupportPolygon(tempFramePoint2d, currentSwingFootInitialPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());

      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   private void computeCoPPointsForUpcomingFootsteps()
   {
      int numberOfUpcomingFootsteps = Math.min(numberFootstepsToConsider.getIntegerValue(), this.numberOfUpcomingFootsteps.getIntegerValue());
      updateFootPolygons();
      computeCoPPointsForSwing(footstepIndex);
      footstepIndex++;

      for (int i = 1; i < numberOfUpcomingFootsteps; i++)
      {
         updateFootPolygons();
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         computeCoPPointsForTransfer(footstepIndex);
         computeCoPPointsForSwing(footstepIndex);
         footstepIndex++;
      }

      // Overwrite the last computed end CoP with the chicken support calculation and additional transfer time
      updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
      computeMidFeetPointWithChickenSupportForFinalTransfer(tempFramePoint);
      copLocationWaypoints.get(numberOfUpcomingFootsteps).addAndSetIncludingFrame(endCoPName, transferDurations.get(footstepIndex).getDoubleValue(), tempFramePoint);
   }

   private void computeCoPPointsForTransfer(int footstepIndex)
   {
      int numberOfSegments = transferCopPointList.length;
      double transferDuration = transferDurations.get(footstepIndex).getDoubleValue();
      double segmentDuration = transferDuration / numberOfSegments;
      CoPPointsInFoot copWaypoints = copLocationWaypoints.get(footstepIndex);

      for (int waypointIndex = 0; waypointIndex < numberOfSegments; waypointIndex++)
      {
         CoPPointName pointName = transferCopPointList[waypointIndex];
         if (pointName == CoPPointName.MIDFEET_COP)
            computeMidFeetPointWithFraction(tempFramePoint, currentSupportFootPolygon, currentSwingFootInitialPolygon, transferSplitFractions.get(footstepIndex).getDoubleValue());
         else
            computeCoPPointLocation(tempFramePoint, pointName, upcomingFootstepsData.get(footstepIndex).getSupportSide());

         copWaypoints.addAndSetIncludingFrame(pointName, segmentDuration, tempFramePoint);
      }
   }



   private final FramePoint soleFrameOrigin = new FramePoint();
   private final FrameVector soleToSoleFrameVector = new FrameVector();

   private void computeCoPPointsForSwing(int footstepIndex)
   {
      RobotSide supportSide = upcomingFootstepsData.get(this.footstepIndex).getSupportSide();
      CoPPointsInFoot copWaypoints = copLocationWaypoints.get(footstepIndex);

      double swingDuration = swingDurations.get(footstepIndex).getDoubleValue();
      double durationOnShifting = swingDurationShiftFractions.get(footstepIndex).getDoubleValue() * swingDuration;
      double durationOnExit = swingDuration - durationOnShifting;

      int numberOfSwingWaypoints = swingCopPointList.length;
      for (int waypointIndex = 0; waypointIndex < numberOfSwingWaypoints; waypointIndex++)
      {
         CoPPointName pointName = swingCopPointList[waypointIndex];

         if (pointName == exitCoPName)
            computeExitCoPPointLocation(tempFramePoint, supportSide);
         else
            computeCoPPointLocation(tempFramePoint, pointName, supportSide);

         double splitFraction;
         if (numberOfSwingWaypoints == 2)
         {
            if (waypointIndex == 0)
               splitFraction = swingSplitFractions.get(footstepIndex).getDoubleValue();
            else
               splitFraction = 1.0 - swingSplitFractions.get(footstepIndex).getDoubleValue();
         }
         else if (numberOfSwingWaypoints == 1)
            splitFraction = 1.0;
         else
            throw new RuntimeException("This logic has not yet been implemented.");

         double segmentDuration = splitFraction * durationOnShifting;

         copWaypoints.addAndSetIncludingFrame(pointName, segmentDuration, tempFramePoint);
      }

      // the last one should still be the exit CoP
      copWaypoints.addAndSetIncludingFrame(exitCoPName, durationOnExit, tempFramePoint);
   }

   private void computeExitCoPPointLocation(FramePoint framePointToPack, RobotSide supportSide)
   {
      boolean polygonIsAPoint = currentSupportFootPolygon.getArea() == 0.0;
      boolean putCMPOnToesWalking = false;
      boolean putCMPOnToesSteppingDown = false;

      boolean isUpcomingFootstepLast = upcomingFootstepsData.size() == 1;
      ReferenceFrame soleFrame = soleZUpFrames.get(supportSide);
      FramePoint2d centroidInSoleFrameOfUpcomingSupportFoot = supportFootPolygonsInSoleZUpFrames.get(supportSide.getOppositeSide()).getCentroid();

      if (!isUpcomingFootstepLast && centroidInSoleFrameOfUpcomingSupportFoot != null && !polygonIsAPoint)
      {
         soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
         soleFrameOrigin.changeFrame(soleFrame);
         soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);
         boolean isSteppingForwardEnough = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue();
         soleToSoleFrameVector.changeFrame(worldFrame);
         boolean isSteppingDownEnough = soleToSoleFrameVector.getZ() < -footstepHeightThresholdToPutExitCoPOnToesSteppingDown.getDoubleValue();

         if (isSteppingDownEnough)
         {
            putCMPOnToesSteppingDown = isSteppingForwardEnough && isSteppingDownEnough;
         }
         else if (putExitCoPOnToes.getBooleanValue())
         {
            soleFrameOrigin.setToZero(centroidInSoleFrameOfUpcomingSupportFoot.getReferenceFrame());
            soleFrameOrigin.changeFrame(soleFrame);
            soleToSoleFrameVector.setIncludingFrame(soleFrameOrigin);

            putCMPOnToesWalking = soleToSoleFrameVector.getX() > footstepLengthThresholdToPutExitCoPOnToes.getDoubleValue();
         }
      }


      if (polygonIsAPoint)
      {
         tempFramePoint2d.setToZero(currentSupportFootPolygon.getReferenceFrame());
         tempFramePoint2d.set(currentSupportFootPolygon.getVertex(0));
         framePointToPack.setXYIncludingFrame(tempFramePoint2d);
      }
      else if (putCMPOnToesWalking)
      {
         putExitCoPOnToes(tempFramePoint2d, currentSupportFootPolygon, copOffsets.get(supportSide).get(exitCoPName).getY());
         framePointToPack.setXYIncludingFrame(tempFramePoint2d);
      }
      else if (putCMPOnToesSteppingDown)
      {
         putExitCoPOnToes(tempFramePoint2d, currentSupportFootPolygon, 0.0);
         framePointToPack.setXYIncludingFrame(tempFramePoint2d);
      }
      else
      {
         computeCoPPointLocation(framePointToPack, exitCoPName, supportSide);
      }

   }

   private void computeCoPPointLocation(FramePoint framePointToPack, CoPPointName copPointName, RobotSide supportSide)
   {
      setCoPPointToPolygonOrigin(tempFramePoint2d, copPointName);

      // Determine the forward offset location of the CoP waypoint in the foot
      double forwardOffset = copOffsets.get(supportSide).get(copPointName).getX() + getStepLengthToCoPOffset(copPointName);
      if (isConstrainedToMinMaxFlags.get(copPointName))
         forwardOffset = MathTools.clamp(forwardOffset, minCoPOffsets.get(copPointName).getDoubleValue(), maxCoPOffsets.get(copPointName).getDoubleValue());

      tempFramePoint2d.add(forwardOffset, copOffsets.get(supportSide).get(copPointName).getY());
      if (isConstrainedToSupportPolygonFlags.get(copPointName))
         constrainCoPPointToSupportPolygon(tempFramePoint2d, copPointName);

      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   private double getStepLengthToCoPOffset(CoPPointName coPPointName)
   {
      switch (stepLengthOffsetReferencePolygons.get(coPPointName))
      {
      case INITIAL_SWING_POLYGON:
         return getStepLengthBasedOffset(currentSupportFootPolygon, currentSwingFootInitialPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_SWING_POLYGON:
         return getStepLengthBasedOffset(currentSupportFootPolygon, currentSwingFootFinalPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         return getStepLengthBasedOffset(currentSupportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         return getStepLengthBasedOffset(currentSupportFootPolygon, tempDoubleSupportPolygon, stepLengthToCoPOffsetFactors.get(coPPointName));
      default:
         return 0.0;
      }
   }

   private void setCoPPointToPolygonOrigin(FramePoint2d copPointToPlan, CoPPointName copPointName)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case INITIAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(currentSwingFootInitialPolygon.getCentroid());
         return;
      case FINAL_SWING_POLYGON:
         copPointToPlan.setIncludingFrame(currentSwingFootFinalPolygon.getCentroid());
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         copPointToPlan.setIncludingFrame(tempDoubleSupportPolygon.getCentroid());
         return;
      case NULL:
         throw new RuntimeException("No frame defined for CoP point:" + copPointName.toString());
      default:
         copPointToPlan.setIncludingFrame(currentSupportFootPolygon.getCentroid());
         return;
      }
   }

   private double getStepLengthBasedOffset(FrameConvexPolygon2d supportPolygon, FrameConvexPolygon2d referencePolygon, double stepLengthToCoPOffsetFactor)
   {
      return stepLengthToCoPOffsetFactor * (referencePolygon.getCentroid().getX() - supportPolygon.getCentroid().getX());
   }

   private void putExitCoPOnToes(FramePoint2d exitCMPToPack, FrameConvexPolygon2d footSupportPolygon, double exitCMPInsideOffset)
   {
      // Set x to have the CMP slightly inside the support polygon
      exitCMPToPack.setToZero(footSupportPolygon.getReferenceFrame());
      exitCMPToPack.setX(footSupportPolygon.getMaxX() - exitCoPForwardSafetyMarginOnToes.getDoubleValue());
      exitCMPToPack.setY(footSupportPolygon.getCentroid().getY() + exitCMPInsideOffset);

      // Then constrain the computed CoP to be inside a safe support region
      constrainCoPPointToSupportPolygon(exitCMPToPack, exitCoPName, safeDistanceFromCoPToSupportEdgesWhenSteppingDown.getDoubleValue());
   }


   private void constrainCoPPointToSupportPolygon(FramePoint2d copPointToConstrain, CoPPointName copPointName)
   {
      constrainCoPPointToSupportPolygon(copPointToConstrain, copPointName, safeDistanceFromCoPToSupportEdges.getDoubleValue());
   }

   private void constrainCoPPointToSupportPolygon(FramePoint2d copPointToConstrain, CoPPointName copPointName, double safeDistance)
   {
      switch (copSupportPolygon.get(copPointName))
      {
      case INITIAL_SWING_POLYGON:
         constrainToSupportPolygon(copPointToConstrain, currentSwingFootInitialPolygon, safeDistance);
         return;
      case FINAL_SWING_POLYGON:
         constrainToSupportPolygon(copPointToConstrain, currentSwingFootFinalPolygon, safeDistance);
         return;
      case INITIAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         constrainToSupportPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistance);
         return;
      case FINAL_DOUBLE_SUPPORT_POLYGON:
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);
         constrainToSupportPolygon(copPointToConstrain, tempDoubleSupportPolygon, safeDistance);
         return;
      case NULL:
         throw new RuntimeException("Invalid constraining frame defined for " + copPointName.toString());
      default:
         constrainToSupportPolygon(copPointToConstrain, currentSupportFootPolygon, safeDistanceFromCoPToSupportEdges.getDoubleValue());
         break;
      }
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by projection
    * @param copPointToConstrain
    * @param safeDistanceFromSupportPolygonEdges
    */
   private void constrainToSupportPolygon(FramePoint2d copPointToConstrain, FrameConvexPolygon2d supportPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonScaler.scaleConvexPolygon(supportPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      tempPolygon.orthogonalProjection(copPointToConstrain);
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

      if (footstepIndex == 0)
      {
         initializeFootPolygons();
      }
      else
      {
         if (upcomingFootstepsData.get(footstepIndex).getSwingSide() == upcomingFootstepsData.get(footstepIndex - 1).getSupportSide()) // the normal way
         {
            currentSwingFootInitialPolygon.setIncludingFrame(currentSupportFootPolygon);
            currentSupportFootPolygon.setIncludingFrame(currentSwingFootFinalPolygon);
         }
         else
         {
            currentSwingFootInitialPolygon.setIncludingFrame(currentSwingFootFinalPolygon);
         }
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
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
         setFootPolygonFromCurrentState(currentSwingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromCurrentState(currentSupportFootPolygon, currentFootstepData.getSupportSide());
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
         break;
      case 1:
         setFootPolygonFromCurrentState(currentSwingFootInitialPolygon, currentFootstepData.getSwingSide());
         setFootPolygonFromFootstep(currentSupportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
         break;
      default:
         setFootPolygonFromFootstep(currentSwingFootInitialPolygon, footstepIndex - 2);
         setFootPolygonFromFootstep(currentSupportFootPolygon, footstepIndex - 1);
         setFootPolygonFromFootstep(currentSwingFootFinalPolygon, footstepIndex);
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

   // TODO This function needs aesthetic improvement
   private void generateCoPTrajectoriesFromWayPoints(WalkingTrajectoryType initialTrajectoryType)
   {
      //It is always guaranteed that the initial state will be transfer the way this code is written. This is needed for the angular momentum approximation to work
      WalkingTrajectoryType trajectoryType = WalkingTrajectoryType.TRANSFER;
      double timeInState = 0.0;
      boolean initialTransfer = true;
      int transferTrajectoryIndex = 0;
      int swingTrajectoryIndex = -1;
      CoPSplineType splineInterpolationOrder = orderOfSplineInterpolation.getEnumValue();
      tempFramePoint.setToNaN();

      for (int waypointIndex = 0; waypointIndex < copLocationWaypoints.size() && !copLocationWaypoints.get(waypointIndex).getCoPPointList().isEmpty();
           waypointIndex++)
      {
         List<CoPPointName> copList = copLocationWaypoints.get(waypointIndex).getCoPPointList();
         int initialIndex = 0;
         if (initialTransfer)
         {
            CoPTrajectoryPoint currentPoint = copLocationWaypoints.get(waypointIndex).get(initialIndex);
            currentPoint.getPosition(tempFramePoint);
            initialIndex = 1;
         }

         for (int segmentIndex = initialIndex; segmentIndex < copList.size(); segmentIndex++)
         {
            CoPPointName pointName = copList.get(segmentIndex);
            CoPTrajectoryPoint currentPoint = copLocationWaypoints.get(waypointIndex).get(segmentIndex);

            if (pointName == transferCopPointList[0] && trajectoryType == WalkingTrajectoryType.SWING)
            {
               trajectoryType = WalkingTrajectoryType.TRANSFER;
               timeInState = 0.0;
               transferTrajectoryIndex++;
            }
            else if (pointName == swingCopPointList[0] && trajectoryType == WalkingTrajectoryType.TRANSFER)
            {
               trajectoryType = WalkingTrajectoryType.SWING;
               timeInState = 0.0;
               swingTrajectoryIndex++;
            }


            if (trajectoryType == WalkingTrajectoryType.SWING)
            {
               swingCoPTrajectories.get(swingTrajectoryIndex)
                                   .setSegment(splineInterpolationOrder, timeInState, timeInState + currentPoint.getTime(), tempFramePoint,
                                               currentPoint.getPosition().getFrameTuple());
               initialTransfer = false;
            }
            else
            {
               if (initialTransfer)
               {
                  transferCoPTrajectories.get(0)
                                         .setSegment(splineInterpolationOrder, timeInState, timeInState + currentPoint.getTime(), tempFramePoint,
                                                     currentPoint.getPosition().getFrameTuple());
                  initialTransfer = false;
               }
               else
               {
                  transferCoPTrajectories.get(transferTrajectoryIndex).setSegment(splineInterpolationOrder, timeInState, timeInState + currentPoint.getTime(), tempFramePoint,
                                                            currentPoint.getPosition().getFrameTuple());
               }
            }

            currentPoint.getPosition(tempFramePoint);
            timeInState += currentPoint.getTime();
         }
      }
   }

   @Override
   public List<? extends CoPTrajectoryInterface> getTransferCoPTrajectories()
   {
      return transferCoPTrajectories;
   }

   @Override
   public List<? extends CoPTrajectoryInterface> getSwingCoPTrajectories()
   {
      return swingCoPTrajectories;
   }

}
