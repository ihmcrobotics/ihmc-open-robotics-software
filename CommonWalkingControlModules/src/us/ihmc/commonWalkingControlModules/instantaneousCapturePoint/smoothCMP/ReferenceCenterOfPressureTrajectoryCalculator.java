package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CenterOfPressurePlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CoPPolynomialTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.FootstepTrajectoryPoint;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCenterOfPressureTrajectoryCalculator implements CoPPolynomialTrajectoryPlannerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CoPPointSize = 0.005;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private YoVariableRegistry parentRegistry;
   private String namePrefix;

   private YoBoolean isDoneWalking;
   private YoInteger numberOfUpcomingFootsteps;
   private YoInteger numberOfPointsPerFoot;
   private YoInteger numberOfFootstepstoConsider;
   private YoEnum<CoPSplineType> orderOfSplineInterpolation;
   private YoDouble defaultFinalTransferDuration;
   private YoDouble defaultStationaryTransferDuration;
   private Vector2D finalTransferOffset;
   private List<FootstepTrajectoryPoint> footstepTrajectory = new ArrayList<>();
   private FrameEuclideanTrajectoryPointList copWayPoints = new FrameEuclideanTrajectoryPointList();
   private ArrayList<YoPolynomial3D> copTrajectoryPolynomials = new ArrayList<>();
   private SideDependentList<List<Vector2D>> copWayPointOffsets = new SideDependentList<>();
   private SideDependentList<List<Double>> copWayPointAlphas = new SideDependentList<>();
   private FramePoint currentCoPPosition;
   private FrameVector currentCoPVelocity;
   private FrameVector currentCoPAcceleration;
   private FrameVector finalCoPVelocity;

   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ReferenceFrame> currentSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> currentSupportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();
   private final SideDependentList<YoFrameVector2d> copUserOffsets = new SideDependentList<>();

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCenterOfPressureTrajectoryCalculator(String namePrefix)
   {
      this.namePrefix = namePrefix;
   }

   /**
    * @param copPlannerParameters
=   * @param bipedSupportPolygons
    * @param contactableFeet
    * @param parentRegistry
    */
   public void initializeParameters(CenterOfPressurePlannerParameters copPlannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry)
   {
      this.parentRegistry = parentRegistry;
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
      isDoneWalking = new YoBoolean(namePrefix + "IsDoneWalking", registry);

      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(side).getContactPoints2d());
         defaultFootPolygons.put(side, defaultFootPolygon);
         currentSupportFootPolygonsInSoleZUpFrames.put(side, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(side));
         copWayPointOffsets.put(side, copPlannerParameters.getCoPWayPointLocationsFootFrame(side));
         if (copWayPointOffsets.get(side).size() != copPlannerParameters.getNumberOfWayPointsPerFoot())
         {
            PrintTools.warn(this, "Mismatch in CoP Offsets size (" + copWayPointOffsets.get(side).size() + ") and number of CoP trajectory way points ("
                  + copPlannerParameters.getNumberOfWayPointsPerFoot() + ")");
            for (int i = copWayPointOffsets.get(side).size(); i < copPlannerParameters.getNumberOfWayPointsPerFoot(); i++)
               copWayPointOffsets.get(side).add(new Vector2D());
            for (int i = copWayPointOffsets.get(side).size() - copPlannerParameters.getNumberOfWayPointsPerFoot(); i > 0; i--)
               copWayPointOffsets.get(side).remove(i);
         }

         copWayPointAlphas.put(side, copPlannerParameters.getCoPWayPointAlpha(side));
         if (copWayPointAlphas.get(side).size() != (copPlannerParameters.getNumberOfWayPointsPerFoot() - 1))
         {
            PrintTools.warn(this, "Mismatch in CoPAlpha size (" + copWayPointAlphas.get(side).size() + ") and number of CoP trajectory way points ("
                  + copPlannerParameters.getNumberOfWayPointsPerFoot() + ")");
            for (int i = copWayPointAlphas.get(side).size(); i < copPlannerParameters.getNumberOfWayPointsPerFoot() - 1; i++)
               copWayPointAlphas.get(side).add(0.0);
            for (int i = copWayPointAlphas.get(side).size() - copPlannerParameters.getNumberOfWayPointsPerFoot() + 1; i > 0; i--)
               copWayPointAlphas.get(side).remove(i);
         }
      }
      currentSoleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      this.bipedSupportPolygons = bipedSupportPolygons;

      this.defaultFinalTransferDuration = new YoDouble(namePrefix + "FinalTransferDuration", registry);
      this.defaultFinalTransferDuration.set(copPlannerParameters.getDefaultFinalTransferDuration());
      this.defaultStationaryTransferDuration = new YoDouble(namePrefix + "StationaryTransferDuration", registry);
      this.defaultStationaryTransferDuration.set(copPlannerParameters.getDefaultStationaryTransferTime());
      this.finalTransferOffset = copPlannerParameters.getFinalTransferCoPOffset();
      this.numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfUpcomingFootsteps.set(0);
      this.numberOfPointsPerFoot = new YoInteger(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.numberOfPointsPerFoot.set(copPlannerParameters.getNumberOfWayPointsPerFoot());
      this.numberOfFootstepstoConsider = new YoInteger(namePrefix + "NumberOfFootstepsToConsider", registry);
      this.numberOfFootstepstoConsider.set(copPlannerParameters.getNumberOfFootstepsToConsider());
      this.orderOfSplineInterpolation = new YoEnum<>(namePrefix + "OrderOfSplineInterpolation", registry, CoPSplineType.class);
      this.orderOfSplineInterpolation.set(copPlannerParameters.getOrderOfCoPInterpolation());
   }

   /**
    * Creates a visualizer for the planned CoP trajectory
    * @param yoGraphicsList
    * @param artifactList
    */
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      if (parentRegistry != null)
      {
         for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints(); i++)
         {
            YoFramePoint graphicFramePoint = new YoFramePoint(namePrefix + "CoPWayPoint" + i, worldFrame, parentRegistry);
            graphicFramePoint.set(copWayPoints.getTrajectoryPoint(i).getPositionX(), copWayPoints.getTrajectoryPoint(i).getPositionY(),
                                  copWayPoints.getTrajectoryPoint(i).getPositionZ());
            YoGraphicPosition CoPViz = new YoGraphicPosition(namePrefix + "GraphicCoPWaypoint" + i, graphicFramePoint, CoPPointSize, YoAppearance.Green(),
                                                             GraphicType.SOLID_BALL);
            yoGraphicsList.add(CoPViz);
            artifactList.add(CoPViz.createArtifact());
         }
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
            footstepTrajectory.add(new FootstepTrajectoryPoint(footstep, timing));
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
      numberOfUpcomingFootsteps.set(footstepTrajectory.size());
   }

   /**
    * Remove first footstep in the upcoming footstep queue from planner
    */
   public void removeFootStepQueueFront()
   {
      removeFootstep(0);
      numberOfUpcomingFootsteps.set(footstepTrajectory.size());
   }

   /**
    * Removes the specified number of footsteps from the queue front
    * @param numberOfFootstepsToRemove number of steps to remove
    */

   public void removeFootStepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
      numberOfUpcomingFootsteps.set(footstepTrajectory.size());
   }

   /**
    * Removes specified footstep from upcoming footstep queue
    * @param index
    */
   public void removeFootstep(int index)
   {
      footstepTrajectory.remove(index);
      numberOfUpcomingFootsteps.set(footstepTrajectory.size());
   }

   /**
    * Clears the CoP plan and footsteps used to generate current plan
    */
   public void clear()
   {
      footstepTrajectory.clear();
      copWayPoints.clear();
      copTrajectoryPolynomials.clear();
      numberOfUpcomingFootsteps.set(0);
      currentCoPPosition = null;
      currentCoPVelocity = null;
   }

   /**
    * Clears the CoP plan. Footsteps used to generate the plan are retained
    */
   public void clearPlan()
   {
      copWayPoints.clear();
      copTrajectoryPolynomials.clear();
      currentCoPVelocity = null;
      currentCoPPosition = null;
   }

   public boolean isDoneWalking()
   {
      return isDoneWalking.getBooleanValue();
   }

   public void update()
   {
      // TODO Auto-generated method stub
   }

   public void setSafeDistanceFromSupportEdges(double distance)
   {
      //TODO Implement this with CoP user Offsets
      return;
   }

   /**
    * 
    * @param side
    * @param copOffsets Offsets are to be provided from the foot polygon centroid in the soleZUpFrame
    */
   public void setCoPWayPoints(RobotSide side, List<Vector2D> copOffsets)
   {
      this.copWayPointOffsets.put(side, copOffsets);
   }

   public int getNumberOfFootstepRegistered()
   {
      return footstepTrajectory.size();
   }

   public int getNumberOfCoPPlannedFootSteps()
   {
      return copWayPoints.getNumberOfTrajectoryPoints();
   }

   public void computeReferenceCoPsForTransfer(RobotSide transferToSide)
   {
      FramePoint2d currentSupportPolygonCentroid = getSupportPolygonCentroid();
      copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(0.0, currentSupportPolygonCentroid));
      FramePoint2d finalSupportPolygonCentroid = getFootSupportPolygonCentroid(transferToSide);
      copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(defaultStationaryTransferDuration.getDoubleValue(),
                                                                                    finalSupportPolygonCentroid));
   }

   private FrameConvexPolygon2d getFootSupportPolygon(RobotSide side)
   {
      return currentSupportFootPolygonsInSoleZUpFrames.get(side);
   }

   private FramePoint2d getFootSupportPolygonCentroid(RobotSide side)
   {
      return getFootSupportPolygon(side).getCentroid();
   }

   private FramePoint2d getSupportPolygonCentroid()
   {
      FramePoint2d p1 = getFootSupportPolygonCentroid(RobotSide.LEFT);
      FramePoint2d p2 = getFootSupportPolygonCentroid(RobotSide.RIGHT);
      FramePoint2d ret = new FramePoint2d(p1);
      p2.changeFrame(p1.getReferenceFrame());
      ret.add(p2);
      ret.scale(0.5);
      return ret;
   }

   public void computeReferenceCoPsStartingFromDoubleSupport(boolean defaultStartingPosition)
   {
      if (!footstepTrajectory.isEmpty())
      {
         RobotSide swingSide = footstepTrajectory.get(0).getSwingSide();
         FrameConvexPolygon2d currentSupportFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(swingSide.getOppositeSide());
         FramePoint2d currentSupportFootPolygonCentroid = new FramePoint2d(currentSupportFootPolygon.getCentroid());
         if (defaultStartingPosition)
         {
            FrameConvexPolygon2d currentSwingFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(swingSide);
            FramePoint2d currentSwingFootPolygonCentroid = currentSwingFootPolygon.getCentroid();

            FramePoint2d currentSupportPolygonCentroid = new FramePoint2d(currentSupportFootPolygonCentroid);
            currentSwingFootPolygonCentroid.changeFrame(currentSupportFootPolygonCentroid.getReferenceFrame());
            currentSupportPolygonCentroid.add(currentSwingFootPolygonCentroid);
            currentSupportPolygonCentroid.scale(0.5);

            copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(0.0, currentSupportPolygonCentroid));
            setInitialCoPVelocity(new FrameVector(currentSoleZUpFrames.get(swingSide.getOppositeSide())));
         }
         else
         {
            copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(0.0, currentCoPPosition));
         }
         computeReferenceCoPsForFootstep(copWayPoints, copWayPointOffsets.get(swingSide.getOppositeSide()), copWayPointAlphas.get(swingSide.getOppositeSide()),
                                         currentSupportFootPolygonCentroid, footstepTrajectory.get(0).getSwingTime(),
                                         footstepTrajectory.get(0).getTransferTime());
         computeReferenceCoPsForUpcomingFootstep();
      }
   }

   public void computeReferenceCoPsStartingFromSingleSupport()
   {
      computeReferenceCoPsStartingFromSingleSupport(0.0);
   }

   public void computeReferenceCoPsStartingFromSingleSupport(double time)
   {
      if (!footstepTrajectory.isEmpty())
      {
         RobotSide swingSide = footstepTrajectory.get(0).getSwingSide();
         double percentageTime = time / footstepTrajectory.get(0).getSwingTime();
         int splineSegmentIndex = getSplineSegmentIndexFromTime(swingSide, percentageTime);
         if(currentCoPPosition != null)
            copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(time, currentCoPPosition));
         else
         {  
            FramePoint2d nextCoPLocation = getFootSupportPolygonCentroid(swingSide.getOppositeSide());
            nextCoPLocation.add(copWayPointOffsets.get(swingSide.getOppositeSide()).get(splineSegmentIndex - 1));
            copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(time, nextCoPLocation));
         }
         FramePoint2d centroidOfSupportFoot = new FramePoint2d(getFootSupportPolygonCentroid(swingSide.getOppositeSide()));
         computeReferenceCoPsForFootstep(copWayPoints, copWayPointOffsets.get(swingSide.getOppositeSide()), copWayPointAlphas.get(swingSide.getOppositeSide()),
                                         centroidOfSupportFoot, footstepTrajectory.get(0).getSwingTime(), splineSegmentIndex);
         computeReferenceCoPsForUpcomingFootstep();
      }
   }

   private int getSplineSegmentIndexFromTime(RobotSide swingSide, double percentageTime)
   {
      int splineSegmentIndex = 0;
      double t = 0.0;
      for (int i = 0; i < copWayPointAlphas.size(); i++)
      {
         t = t + copWayPointAlphas.get(swingSide.getOppositeSide()).get(i);
         if (percentageTime < t)
         {
            splineSegmentIndex = i + 1;
            break;
         }
      }
      return splineSegmentIndex;
   }

   private void computeReferenceCoPsForUpcomingFootstep()
   {
      int numberOfFootstepsToPlan = footstepTrajectory.size() - 1 > numberOfFootstepstoConsider.getIntegerValue() - 1
            ? numberOfFootstepstoConsider.getIntegerValue() - 1 : footstepTrajectory.size() - 1;
      computeReferenceCoPsForUpcomingFootstep(numberOfFootstepsToPlan);
   }

   private void computeReferenceCoPsForUpcomingFootstep(int numberOfFootstepsToPlan)
   {
      int index = 0;
      for (; index < numberOfFootstepsToPlan; index++)
      {
         FootstepTrajectoryPoint planningFootstep = footstepTrajectory.get(index);
         FramePoint2d footstepLocation = planningFootstep.getCentroidOfExpectedFootPolygonLocation().toFramePoint2d();
         computeReferenceCoPsForFootstep(copWayPoints, copWayPointOffsets.get(planningFootstep.getSupportSide()),
                                         copWayPointAlphas.get(planningFootstep.getSupportSide()), footstepLocation, planningFootstep.getSwingTime(),
                                         planningFootstep.getTransferTime());
      }
      if (footstepTrajectory.size() - 1 <= numberOfFootstepstoConsider.getIntegerValue())
         computeReferenceCoPsForFinalTransfer(index);
   }

   private void computeReferenceCoPsForFinalTransfer(int indexOfFinalFootStep)
   {
      FootstepTrajectoryPoint planningFootstep = footstepTrajectory.get(indexOfFinalFootStep);
      FramePoint2d lastFootstepLocation = planningFootstep.getCentroidOfExpectedFootPolygonLocation().toFramePoint2d();

      FramePoint2d nextCoPLocation;
      if (footstepTrajectory.size() == 1)
      {
         FrameConvexPolygon2d lastSupportFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(planningFootstep.getSupportSide());
         nextCoPLocation = lastSupportFootPolygon.getCentroidCopy();
      }
      else
      {
         nextCoPLocation = new FramePoint2d(footstepTrajectory.get(indexOfFinalFootStep - 1).getCentroidOfExpectedFootPolygonLocation().toFramePoint2d());
      }
      lastFootstepLocation.changeFrame(nextCoPLocation.getReferenceFrame());
      nextCoPLocation.add(lastFootstepLocation);
      nextCoPLocation.scale(0.5);
      nextCoPLocation.add(finalTransferOffset);
      copWayPoints.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(defaultFinalTransferDuration.getDoubleValue(), nextCoPLocation));
   }

   private void computeReferenceCoPsForFootstep(FrameEuclideanTrajectoryPointList copListToPack, List<Vector2D> copOffsets, List<Double> copWayPointAlpha,
                                                FramePoint2d centroidOfFootstep, double swingTime, double transferTime)
   {
      FramePoint2d nextCoPLocation = new FramePoint2d(centroidOfFootstep);
      nextCoPLocation.add(copOffsets.get(0));
      copListToPack.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(transferTime, nextCoPLocation));
      computeReferenceCoPsForFootstep(copListToPack, copOffsets, copWayPointAlpha, centroidOfFootstep, swingTime, 1);
   }

   /**
    * 
    * @param copListToPack
    * @param copOffsets
    * @param copWayPointAlpha
    * @param centroidOfFootstep
    * @param swingTime
    * @param copOffsetIndex Must be greater than 1
    */
   private void computeReferenceCoPsForFootstep(FrameEuclideanTrajectoryPointList copListToPack, List<Vector2D> copOffsets, List<Double> copWayPointAlpha,
                                                FramePoint2d centroidOfFootstep, double swingTime, int copOffsetIndex)
   {
      for (int i = copOffsetIndex; i < copOffsets.size(); i++)
      {
         FramePoint2d nextCoPLocation = new FramePoint2d(centroidOfFootstep);
         nextCoPLocation.add(copOffsets.get(i));
         copListToPack.addTrajectoryPoint(convertToWorldFrameAndPackIntoTrajectoryPoint(copWayPointAlpha.get(i - 1) * swingTime, nextCoPLocation));
      }
   }

   public List<FramePoint> getCoPs()
   {
      convertCoPWayPointsToWorldFrame();
      List<FramePoint> wayPointPositionList = new ArrayList<>(copWayPoints.getNumberOfTrajectoryPoints());
      for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints(); i++)
         wayPointPositionList.add(copWayPoints.getTrajectoryPoint(i).getPositionCopy());
      return wayPointPositionList;
   }

   public FramePoint getNextCoP()
   {
      convertCoPWayPointsToWorldFrame();
      return copWayPoints.getTrajectoryPoint(0).getPositionCopy();
   }

   public void getNextCoP(FramePoint entryCMPToPack)
   {
      FramePoint nextCoP = copWayPoints.getTrajectoryPoint(0).getPositionCopy();
      entryCMPToPack.setIncludingFrame(nextCoP);
   }

   @Override
   public List<YoPolynomial3D> getPolynomialTrajectory()
   {
      convertCoPWayPointsToWorldFrame();
      generatePolynomialCoefficients();
      return copTrajectoryPolynomials;
   }

   private void generatePolynomialCoefficients()
   {
      copTrajectoryPolynomials.clear();
      if (orderOfSplineInterpolation.getEnumValue() == CoPSplineType.CUBIC)
         generateCubicCoefficients();
      /*else if (orderOfSplineInterpolation.getEnumValue() == CoPSplineType.NATURAL_CUBIC)
         generateNaturalCubicCoefficients();
      else if (orderOfSplineInterpolation.getEnumValue() == CoPSplineType.CLAMPED_CUBIC)
         generateClampedCubicCoefficients(); */
      else if (orderOfSplineInterpolation.getEnumValue() == CoPSplineType.LINEAR)
         generateLinearCoefficients();
   }

   // TODO Implement clamped and natural cubic interpolation
   /*private void generateClampedCubicCoefficients()
   {
      Vector3D initialVelocity, finalVelocity;
      if (currentCoPVelocity != null)
      {
         currentCoPVelocity.changeFrame(worldFrame);
         initialVelocity = currentCoPVelocity.getVector();
      }
      else
         initialVelocity = new Vector3D();

      if (finalCoPVelocity != null)
      {
         currentCoPVelocity.changeFrame(worldFrame);
         finalVelocity = currentCoPVelocity.getVector();
      }
      else
         finalVelocity = new Vector3D();
      
      LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(4*(copWayPoints.getNumberOfTrajectoryPoints() - 1));
      for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints() - 1; i++)
      {
         
      }
   }

   private void generateNaturalCubicCoefficients()
   {
      PrintTools.warn(this, "Spline interpolation with natural cubic splines has not been implemented");
      return;
   }*/

   private void generateCubicCoefficients()
   {
      Vector3D initialVelocity, initialAcceleration;
      if (currentCoPVelocity != null)
      {
         currentCoPVelocity.changeFrame(worldFrame);
         initialVelocity = currentCoPVelocity.getVector();
      }
      else
         initialVelocity = new Vector3D();

      if (currentCoPAcceleration != null)
      {
         currentCoPAcceleration.changeFrame(worldFrame);
         initialAcceleration = currentCoPAcceleration.getVector();
      }
      else
         initialAcceleration = new Vector3D();

      for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints() - 1; i++)
      {
         YoPolynomial3D piecewiseSpline = new YoPolynomial3D(namePrefix + "CoPSpline" + i, 2, registry);
         FrameEuclideanTrajectoryPoint waypoint1 = copWayPoints.getTrajectoryPoint(i);
         FrameEuclideanTrajectoryPoint waypoint2 = copWayPoints.getTrajectoryPoint(i + 1);
         Point3DReadOnly point1 = waypoint1.getPositionCopy().getPoint();
         Point3DReadOnly point2 = waypoint2.getPositionCopy().getPoint();
         piecewiseSpline.setCubicThreeInitialConditionsFinalPosition(0.0, waypoint2.getTime(), point1, initialVelocity, initialAcceleration, point2);
         copTrajectoryPolynomials.add(piecewiseSpline);
         piecewiseSpline.compute(waypoint2.getTime());
         initialVelocity.set(piecewiseSpline.getVelocity());
         initialAcceleration.set(piecewiseSpline.getAcceleration());
      }
   }

   private void generateLinearCoefficients()
   {
      for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints() - 1; i++)
      {
         YoPolynomial3D piecewiseSpline = new YoPolynomial3D(namePrefix + "CoPSpline" + i, 2, registry);
         FrameEuclideanTrajectoryPoint wayPoint1 = copWayPoints.getTrajectoryPoint(i);
         FrameEuclideanTrajectoryPoint wayPoint2 = copWayPoints.getTrajectoryPoint(i + 1);
         Point3D point1 = wayPoint1.getPositionCopy().getPoint();
         Point3D point2 = wayPoint2.getPositionCopy().getPoint();
         piecewiseSpline.setLinear(wayPoint1.getTime(), wayPoint2.getTime(), point1, point2);
         copTrajectoryPolynomials.add(piecewiseSpline);
      }
   }

   private void convertCoPWayPointsToWorldFrame()
   {
      for (int i = 0; i < copWayPoints.getNumberOfTrajectoryPoints(); i++)
      {
         copWayPoints.getTrajectoryPoint(i).changeFrame(worldFrame);
      }
   }

   @Override
   public void setInitialCoPAcceleration(FrameVector initialCoPAcceleration)
   {
      this.currentCoPAcceleration = new FrameVector(initialCoPAcceleration);
   }

   @Override
   public void setInitialCoPAcceleration(FrameVector2d initialCoPAccel)
   {
      this.currentCoPAcceleration = new FrameVector(initialCoPAccel.getReferenceFrame(), initialCoPAccel.getX(), initialCoPAccel.getY(), 0.0);
   }

   @Override
   public void setInitialCoPVelocity(FrameVector initialCoPVelocity)
   {
      this.currentCoPVelocity = new FrameVector(initialCoPVelocity);
   }

   @Override
   public void setInitialCoPVelocity(FrameVector2d initialCoPVelocity)
   {
      this.currentCoPVelocity = new FrameVector(initialCoPVelocity.getReferenceFrame(), initialCoPVelocity.getX(), initialCoPVelocity.getY(), 0.0);
   }
   
   @Override
   public void setFinalCoPVelocity(FrameVector finalCoPVelocity)
   {
      this.currentCoPVelocity = new FrameVector(finalCoPVelocity);
   }

   @Override
   public void setFinalCoPVelocity(FrameVector2d finalCoPVelocity)
   {
      this.currentCoPVelocity = new FrameVector(finalCoPVelocity.getReferenceFrame(), finalCoPVelocity.getX(), finalCoPVelocity.getY(), 0.0);
   }

   @Override
   public void setInitialCoPPosition(FramePoint initialCoPPosition)
   {
      this.currentCoPPosition = new FramePoint(initialCoPPosition);
   }

   @Override
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition)
   {
      this.currentCoPPosition = new FramePoint(initialCoPPosition.getReferenceFrame(), initialCoPPosition.getX(), initialCoPPosition.getY(), 0.0);
   }

   private FrameEuclideanTrajectoryPoint convertToWorldFrameAndPackIntoTrajectoryPoint(double time, FramePoint2d position)
   {
      position.changeFrame(worldFrame);
      return new FrameEuclideanTrajectoryPoint(time, position.toFramePoint(), new FrameVector(position.getReferenceFrame()));
   }

   private FrameEuclideanTrajectoryPoint convertToWorldFrameAndPackIntoTrajectoryPoint(double time, FramePoint position)
   {
      position.changeFrame(worldFrame);
      return new FrameEuclideanTrajectoryPoint(time, position, new FrameVector(position.getReferenceFrame()));
   }

}
