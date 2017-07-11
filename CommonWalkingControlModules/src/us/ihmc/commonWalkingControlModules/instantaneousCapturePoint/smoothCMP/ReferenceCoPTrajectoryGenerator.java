package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
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
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
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

public class ReferenceCoPTrajectoryGenerator implements CoPPolynomialTrajectoryPlannerInterface, ReferenceCoPTrajectoryGeneratorInterface
{
   // Standard declarations
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static double COP_POINT_SIZE = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final String namePrefix;

   // Waypoint planning parameters
   private final List<YoDouble> maxCoPOffsets = new ArrayList<>();
   private final List<YoDouble> minCoPOffsets = new ArrayList<>();

   private final SideDependentList<List<YoFrameVector2d>> copUserOffsets = new SideDependentList<>();
   private final YoDouble safeDistanceFromCoPToSupportEdges;
   private final YoDouble stepLengthToCoPOffsetFactor;
   private final YoDouble percentageChickenSupport;
   private final CoPPointName entryCoPPointName;
   private final CoPPointName exitCoPPointName;
   private final CoPPointName[] copPointList;

   // Trajectory planning parameters
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingSplitFractions;
   private final List<YoDouble> swingDurationShiftFractions;
   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferSplitFractions;

   // State variables 
   private final SideDependentList<ReferenceFrame> soleZUpFrames;
   private final SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   // Planner parameters
   private final YoInteger numberOfPointsPerFoot;
   private final YoInteger numberOfUpcomingFootsteps;
   private final YoInteger numberFootstepsToConsider;

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
   private CoPTrajectory activeTrajectory;
   private double initialTime;
   private FrameConvexPolygon2d currentDoubleSupportPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSupportFootPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootInitialPolygon = new FrameConvexPolygon2d();
   private FrameConvexPolygon2d currentSwingFootFinalPolygon = new FrameConvexPolygon2d();

   // Temp variables for computation only
   private FootstepData currentFootstepData;
   private FrameConvexPolygon2d tempPolygon = new FrameConvexPolygon2d();
   private ConvexPolygonShrinker polygonShrinker = new ConvexPolygonShrinker();
   private FramePoint tempFramePoint = new FramePoint();
   private double tempDouble;
   private FramePoint2d tempFramePoint2d = new FramePoint2d();
   private RobotSide currentSupportSide;
   private FrameConvexPolygon2d framePolygonReference;

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
                                          List<YoDouble> swingSplitFractions, List<YoDouble> swingDurationShiftFractions,
                                          List<YoDouble> transferSplitFractions, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.numberFootstepsToConsider = numberFootstepsToConsider;
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingSplitFractions = swingSplitFractions;
      this.swingDurationShiftFractions = swingDurationShiftFractions;
      this.transferSplitFractions = transferSplitFractions;

      safeDistanceFromCoPToSupportEdges = new YoDouble(namePrefix + "SafeDistanceFromCoPToSupportEdges", registry);
      stepLengthToCoPOffsetFactor = new YoDouble(namePrefix + "StepLengthToCMPOffsetFactor", registry);

      percentageChickenSupport = new YoDouble("PercentageChickenSupport", registry);
      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      for (int waypointIndex = 0; waypointIndex < numberOfPointsPerFoot; waypointIndex++)
      {
         YoDouble maxCoPOffset = new YoDouble("maxCoPForwardOffset" + waypointIndex, registry);
         YoDouble minCoPOffset = new YoDouble("minCoPForwardOffset" + waypointIndex, registry);

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
         
         for(int i = 0; i < numberOfPointsPerFoot; i++)
         {
            YoFrameVector2d copUserOffset = new YoFrameVector2d(namePrefix + sidePrefix + "CoPConstantOffset" + i, null, registry);
            copUserOffsets.add(copUserOffset);
         }

         this.copUserOffsets.put(robotSide, copUserOffsets);
      }


      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, i, transferDurations.get(i), transferSplitFractions.get(i),
                                                                                 registry);
         SwingCoPTrajectory swingCoPTrajectory = new SwingCoPTrajectory(namePrefix, i, swingDurations.get(i), swingSplitFractions.get(i),
                                                                        swingDurationShiftFractions.get(i), registry);
         transferCoPTrajectories.add(transferCoPTrajectory);
         swingCoPTrajectories.add(swingCoPTrajectory);
      }
      // Also save the final transfer trajectory
      TransferCoPTrajectory transferCoPTrajectory = new TransferCoPTrajectory(namePrefix, maxNumberOfFootstepsToConsider,
                                                                              transferDurations.get(maxNumberOfFootstepsToConsider),
                                                                              transferSplitFractions.get(maxNumberOfFootstepsToConsider), registry);
      transferCoPTrajectories.add(transferCoPTrajectory);

      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, bipedSupportPolygons.getMidFeetZUpFrame(), soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT)};
      // Need two additional CoPPoints in Foot to store the initial and final footstep CoPs
      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         copLocationWaypoints.add(new CoPPointsInFoot(i, framesToRegister, registry));
      }
      copLocationWaypoints.add(new CoPPointsInFoot(maxNumberOfFootstepsToConsider + 1, framesToRegister, registry));

      parentRegistry.addChild(registry);
      clear();
   }

   public void initializeParameters(SmoothCMPPlannerParameters parameters)
   {
      // TODO modify to have things that should be adjusted on the fly here
      safeDistanceFromCoPToSupportEdges.set(parameters.getCoPSafeDistanceAwayFromSupportEdges());
      stepLengthToCoPOffsetFactor.set(parameters.getStepLengthToBallCoPOffsetFactor());
      numberOfPointsPerFoot.set(parameters.getNumberOfCoPWayPointsPerFoot());
      orderOfSplineInterpolation.set(parameters.getOrderOfCoPInterpolation());

      percentageChickenSupport.set(0.5);

      List<Vector2D> copOffsets = parameters.getCoPOffsets();
      for(int waypointNumber = 0; waypointNumber < copOffsets.size(); waypointNumber++)
         setSymmetricCoPConstantOffsets(waypointNumber, copOffsets.get(waypointNumber));

      List<Vector2D> copForwardOffsetBounds = parameters.getCoPForwardOffsetBounds();
      for (int waypointIndex = 0; waypointIndex < copForwardOffsetBounds.size(); waypointIndex++)
      {
         Vector2D bounds = copForwardOffsetBounds.get(waypointIndex);
         minCoPOffsets.get(waypointIndex).set(bounds.getX());
         maxCoPOffsets.get(waypointIndex).set(bounds.getY());
      }
   }

   @Override
   public void setSymmetricCoPConstantOffsets(int waypointNumber, Vector2D heelOffset)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         YoFrameVector2d copUserOffset = copUserOffsets.get(robotSide).get(waypointNumber);
         copUserOffset.setX(heelOffset.getX());
         copUserOffset.setY(robotSide.negateIfLeftSide(heelOffset.getY()));
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
                                                             copPointsInFoot.getWaypointInWorldFrameReadOnly(copPointNames.get(i)), COP_POINT_SIZE,
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

      footstepIndex = 0;
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
         updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootInitialPolygon);
         computeMidFeetPointWithChickenSupportForInitialTransfer(tempFramePoint);
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(CoPPointName.MIDFEET_COP, tempFramePoint);
      }
      // Put first CoP at the exitCoP of the swing foot if not starting from rest 
      else
      {
         computeExitCoP(tempFramePoint, upcomingFootstepsData.get(footstepIndex).getSupportSide());
         copLocationWaypoints.get(footstepIndex).addAndSetIncludingFrame(exitCoPPointName, tempFramePoint);
      }
      computeCoPPointsForUpcomingFootsteps(footstepIndex + 1);
   }

   @Override
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      footstepIndex = 0;      
      // Do nothing if no footsteps planned
      if (numberOfUpcomingFootsteps.getIntegerValue() == 0)
         return;
      else
      {
         computeCoPPointsForUpcomingFootsteps(footstepIndex);
      }
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
    * Assumes foot polygon and double support polygon has been udpated
    * @param framePointToPack
    * @param transferToSide
    */
   private void computeMidFeetPointWithChickenSupport(FramePoint framePointToPack, FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon)
   {      
      this.tempDouble = MathTools.clamp(percentageChickenSupport.getDoubleValue(), 0.0, 1.0);
      if (this.tempDouble < 0.5)
      {
         this.tempDouble *= 2.0;
         this.framePolygonReference = supportFootPolygon;

      }
      else
      {
         this.tempDouble = (0.5 - this.tempDouble) * 2.0;
         this.framePolygonReference = swingFootPolygon;
      }
      this.currentDoubleSupportPolygon.changeFrame(worldFrame);
      this.framePolygonReference.changeFrame(worldFrame);
      this.tempFramePoint2d.interpolate(this.currentDoubleSupportPolygon.getCentroid(), this.framePolygonReference.getCentroid(), this.tempDouble);
      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
   }

   /**
    * Calculates the exit CoP for a given footstep
    * @param framePointToPack
    * @param robotSide
    */
   private void computeExitCoP(FramePoint framePointToPack, RobotSide supportSide)
   {
      computeCoPPointLocation(tempFramePoint2d, currentSwingFootInitialPolygon, currentSupportFootPolygon,
                              copUserOffsets.get(supportSide.getOppositeSide()).get(exitCoPPointName).getX(), copUserOffsets.get(supportSide.getOppositeSide()).get(exitCoPPointName).getY(),
                              minCoPOffsets.get(exitCoPPointName).getDoubleValue(), maxCoPOffsets.get(exitCoPPointName).getDoubleValue(),
                              stepLengthToCoPOffsetFactor.getDoubleValue(), safeDistanceFromCoPToSupportEdges.getDoubleValue());
      framePointToPack.setXYIncludingFrame(tempFramePoint2d);
      framePointToPack.changeFrame(worldFrame);
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
      updateDoubleSupportPolygon(currentSupportFootPolygon, currentSwingFootFinalPolygon);      
      computeMidFeetPointWithChickenSupportForFinalTransfer(tempFramePoint);
      copLocationWaypoints.get(copLocationIndex).addAndSetIncludingFrame(CoPPointName.MIDFEET_COP, tempFramePoint);
   }

   /**
    * Assumes all the support polygons have been set accordingly and computes the CoP points for the current footstep
    */
   private void computeCoPPointsForFootstep(int copLocationsIndex)
   {
      this.currentSupportSide = upcomingFootstepsData.get(footstepIndex).getSupportSide();
      for (int i = 0; i < copPointList.length; i++)
      {
         computeCoPPointLocation(tempFramePoint2d, currentSupportFootPolygon, currentSwingFootFinalPolygon,
                                 copUserOffsets.get(this.currentSupportSide).get(i).getX(),
                                 copUserOffsets.get(this.currentSupportSide).get(i).getY(), minCoPOffsets.get(i).getDoubleValue(),
                                 maxCoPOffsets.get(i).getDoubleValue(), stepLengthToCoPOffsetFactor.getDoubleValue(),
                                 safeDistanceFromCoPToSupportEdges.getDoubleValue());
         tempFramePoint.setXYIncludingFrame(tempFramePoint2d);
         copLocationWaypoints.get(copLocationsIndex).addAndSetIncludingFrame(copPointList[i], tempFramePoint);
      }
   }

   /**
    * Generic implementation of CoP calculation. Performs 
    * <ol> 
    * <li> calculation of step length offset factor in X axis </li>
    * <li> adding the specified CoP offsets in both axes
    * <li> constraining to min/max offsets </li>
    * <li> constraining to safe distance within support foot polygon </li> </ol>
    * in that order
    * @param copPointToPlan
    * @param supportFoot
    * @param swingFootEstimatedFinalLocation
    * @param xOffset
    * @param yOffset
    * @param maxXOffset
    * @param minXOffset
    * @param stepLengthToCoPOffsetFactor
    */
   private void computeCoPPointLocation(FramePoint2d copPointToPlan, FrameConvexPolygon2d supportFoot, FrameConvexPolygon2d swingFootEstimatedFinalLocation,
                                        double xOffset, double yOffset, double minXOffset, double maxXOffset, double stepLengthToCoPOffsetFactor,
                                        double safeDistanceFromSupportPolygonEdges)
   {
      // Calculating the CoP point and constraining to min/max X axis offsets
      this.tempDouble = MathTools.clamp(stepLengthToCoPOffsetFactor * (swingFootEstimatedFinalLocation.getCentroid().getX() - supportFoot.getCentroid().getX())
            + xOffset, minXOffset, maxXOffset);      
      copPointToPlan.setIncludingFrame(supportFoot.getCentroid());
      copPointToPlan.add(tempDouble, yOffset);

      constrainToSupportPolygon(copPointToPlan, supportFoot, safeDistanceFromSupportPolygonEdges);
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by projection
    * @param copPointToConstrain
    * @param supportFoot
    * @param safeDistanceFromSupportPolygonEdges
    */
   private void constrainToSupportPolygon(FramePoint2d copPointToConstrain, FrameConvexPolygon2d supportPolygon, double safeDistanceFromSupportPolygonEdges)
   {
      polygonShrinker.shrinkConstantDistanceInto(supportPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      tempPolygon.orthogonalProjection(copPointToConstrain);
   }

   /**
    * Updates the variable {@code currentDoubleSupportPolygon} from the specified swing and support polygons 
    */
   private void updateDoubleSupportPolygon(FrameConvexPolygon2d supportFootPolygon, FrameConvexPolygon2d swingFootPolygon)
   {
      currentDoubleSupportPolygon.setIncludingFrame(supportFootPolygon);
      currentDoubleSupportPolygon.changeFrame(worldFrame);
      tempPolygon.setIncludingFrame(swingFootPolygon);
      tempPolygon.changeFrame(worldFrame);
      currentDoubleSupportPolygon.addVertices(tempPolygon);
      currentDoubleSupportPolygon.update();
   }

   /**
    * Updates the swing and support foot polygons based on footstepIndex
    * <p> Has no memory of the previous state so should be used carefully </p>
    */
   private void updateFootPolygons()
   {
      switch (footstepIndex)
      {
      case 0:
         initializeFootPolygons();
         break;
      default:
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
      framePolygonToPack.setIncludingFrame(supportFootPolygonsInSoleZUpFrames.get(robotSide));
      framePolygonToPack.update();
      framePolygonToPack.changeFrame(worldFrame);
   }

   @Override
   public void removeFootstepQueueFront()
   {
      removeFootstep(0);
   }

   @Override
   public void removeFootstepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
      {
         removeFootstep(0);
         numberOfUpcomingFootsteps.decrement();
      }
   }

   @Override
   public void removeFootstep(int index)
   {
      upcomingFootstepsData.remove(index);
      numberOfUpcomingFootsteps.decrement();
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
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setInitialCoPPosition(FramePoint initialCoPPosition)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setInitialCoPVelocity(FrameVector2d intialCoPVelocity)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setInitialCoPVelocity(FrameVector intialCoPVelocity)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setInitialCoPAcceleration(FrameVector2d initialCoPAcceleration)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setInitialCoPAcceleration(FrameVector initialCoPAcceleration)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setFinalCoPVelocity(FrameVector finalCoPVelocity)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void setFinalCoPVelocity(FrameVector2d finalCoPVelocity)
   {
      // TODO Auto-generated method stub

   }
}
