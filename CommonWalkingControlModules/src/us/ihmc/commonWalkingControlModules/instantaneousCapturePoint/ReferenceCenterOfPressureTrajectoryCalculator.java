package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
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
import us.ihmc.robotics.math.trajectories.waypoints.FrameTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.LinearInterpolater;

public class ReferenceCenterOfPressureTrajectoryCalculator implements CMPComponentPolynomialTrajectoryPlannerInterface
{
   // Some general hygiene declarations  
   private static final CMPComponentType cmpComponentType = CMPComponentType.CoP;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CoPPointSize = 0.005;

   // Storing registry and name data for YoVariable creation
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private YoVariableRegistry parentRegistry;
   private String namePrefix;

   // State variables 
   private BooleanYoVariable isDoneWalking;
   private IntegerYoVariable numberOfUpcomingFootsteps;
   private IntegerYoVariable numberOfPointsPerFoot;
   private IntegerYoVariable numberOfFootstepstoConsider;
   private IntegerYoVariable orderOfSplineInterpolation;
   private DoubleYoVariable defaultFinalTransferDuration;

   // Plan variables
   private List<FootstepTrajectoryPoint> footstepLocation = new ArrayList<>();
   private FrameEuclideanTrajectoryPointList coPWayPoints = new FrameEuclideanTrajectoryPointList();
   private ArrayList<YoPolynomial3D> coPTrajectoryPolynomials = new ArrayList<>();
   private SideDependentList<List<FrameVector2d>> coPOffsets = new SideDependentList<>();
   private SideDependentList<List<Double>> coPWayPointAlphas = new SideDependentList<>();
   private FrameVector2d currentCoPVelocity;
   private FramePoint2d currentCoPPosition;

   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ReferenceFrame> currentSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> currentSupportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();
   private final SideDependentList<YoFrameVector2d> coPUserOffsets = new SideDependentList<>();

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCenterOfPressureTrajectoryCalculator(String namePrefix)
   {
      this.namePrefix = namePrefix;
   }

   /**
    * @param icpPlannerParameters
    * @param bipedSupportPolygons
    * @param contactableFeet
    * @param parentRegistry
    */
   public void initializeParameters(ExtendedCapturePointPlannerParameters icpPlannerParameters, BipedSupportPolygons bipedSupportPolygons,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoVariableRegistry parentRegistry,
                                    double defaultFinalTransferDuration)
   {
      this.parentRegistry = parentRegistry;
      this.parentRegistry.addChild(registry);
      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);

      for (RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(side).getContactPoints2d());
         defaultFootPolygons.put(side, defaultFootPolygon);
         currentSupportFootPolygonsInSoleZUpFrames.put(side, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(side));
         coPOffsets.put(side, icpPlannerParameters.getCoPWayPointLocationsFootFrame(side));
         if (coPOffsets.get(side).size() != icpPlannerParameters.getNumberOfPointsPerFoot())
         {
            PrintTools.warn(this, "Mismatch in CoP Offsets size (" + coPOffsets.size() + " and number of CoP trajectory way points ("
                  + icpPlannerParameters.getNumberOfPointsPerFoot() + ")");
            for (int i = coPOffsets.size(); i < icpPlannerParameters.getNumberOfPointsPerFoot(); i++)
               coPOffsets.get(side).add(new FrameVector2d());
            for (int i = coPOffsets.size() - icpPlannerParameters.getNumberOfPointsPerFoot(); i > 0; i--)
               coPOffsets.get(side).remove(i);
         }

         coPWayPointAlphas.put(side, icpPlannerParameters.getCoPWayPointAlpha(side));
         if (coPWayPointAlphas.get(side).size() != icpPlannerParameters.getNumberOfPointsPerFoot() - 1)
         {
            PrintTools.warn(this, "Mismatch in CoPAlpha size (" + coPWayPointAlphas.size() + " and number of CoP trajectory way points ("
                  + icpPlannerParameters.getNumberOfPointsPerFoot() + ")");
            for (int i = coPWayPointAlphas.size(); i < icpPlannerParameters.getNumberOfPointsPerFoot() - 1; i++)
               coPWayPointAlphas.get(side).add(0.0);
            for (int i = coPWayPointAlphas.size() - icpPlannerParameters.getNumberOfPointsPerFoot() + 1; i > 0; i++)
               coPWayPointAlphas.get(side).remove(i);
         }
      }
      currentSoleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();
      this.bipedSupportPolygons = bipedSupportPolygons;

      this.defaultFinalTransferDuration = new DoubleYoVariable(namePrefix + "FinalTransferDuration", registry);
      this.defaultFinalTransferDuration.set(defaultFinalTransferDuration);
      this.numberOfUpcomingFootsteps = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfUpcomingFootsteps.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      this.numberOfPointsPerFoot = new IntegerYoVariable(namePrefix + "NumberOfPointsPerFootstep", registry);
      this.numberOfPointsPerFoot.set(icpPlannerParameters.getNumberOfPointsPerFoot());
      this.numberOfFootstepstoConsider = new IntegerYoVariable(namePrefix + "NumberOfFootstepsToConsider", registry);
      this.numberOfFootstepstoConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      this.orderOfSplineInterpolation = new IntegerYoVariable(namePrefix + "OrderOfCoPInterpolation", registry);
      this.orderOfSplineInterpolation.set(icpPlannerParameters.getOrderOfCoPInterpolation());
   }

   /**
    * Creates a visualizer for the planned CoP trajectory
    * @param yoGraphicsList 
    * @param artifactList
    */
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int i = 0; i < coPWayPoints.getNumberOfTrajectoryPoints(); i++)
      {
         YoFramePoint graphicFramePoint = new YoFramePoint(namePrefix + "CoPWayPoint" + i, worldFrame, parentRegistry);
         graphicFramePoint.set(coPWayPoints.getTrajectoryPoint(i).getPositionX(), coPWayPoints.getTrajectoryPoint(i).getPositionY(),
                               coPWayPoints.getTrajectoryPoint(i).getPositionZ());
         YoGraphicPosition entryCMPViz = new YoGraphicPosition(namePrefix + "GraphicCoPWaypoint" + i, graphicFramePoint, CoPPointSize, YoAppearance.Green(),
                                                               GraphicType.SOLID_BALL);
         yoGraphicsList.add(entryCMPViz);
         artifactList.add(entryCMPViz.createArtifact());
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
            footstepLocation.add(new FootstepTrajectoryPoint(footstep, timing));
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
      numberOfUpcomingFootsteps.set(footstepLocation.size());
   }

   /**
    * Remove first footstep in the upcoming footstep queue from planner
    */
   public void removeFootStepQueueFront()
   {
      removeFootstep(0);
      numberOfUpcomingFootsteps.set(footstepLocation.size());
   }

   /**
    * Removes the specified number of footsteps from the queue front
    * @param numberOfFootstepsToRemove number of steps to remove
    */

   public void removeFootStepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
      numberOfUpcomingFootsteps.set(footstepLocation.size());
   }

   /**
    * Removes specified footstep from upcoming footstep queue
    * @param index
    */
   public void removeFootstep(int index)
   {
      footstepLocation.remove(index);
      numberOfUpcomingFootsteps.set(footstepLocation.size());
   }

   /**
    * Clears the CoP plan and footsteps used to generate current plan
    */
   public void clear()
   {
      footstepLocation.clear();
      numberOfUpcomingFootsteps.set(0);
      coPWayPoints.clear();
      currentCoPPosition = null;
      currentCoPVelocity = null;
   }

   /**
    * Clears the CoP plan. Footsteps used to generate the plan are retained
    */
   public void clearPlan()
   {
      coPWayPoints.clear();
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

   public void setSymmetricCoPConstantOffsets(double entryCMPForwardOffset, double entryCMPInsideOffset)
   {
      // TODO Implement with CoP user offsets
      return;
   }

   public int getNumberOfFootstepRegistered()
   {
      return footstepLocation.size();
   }

   public int getNumberOfCoPPlannedFootSteps()
   {
      return coPWayPoints.getNumberOfTrajectoryPoints();
   }

   public void computeReferenceCoPsStartingFromDoubleSupport(RobotSide swingSide, boolean defaultStartingPosition)
   {
      FrameConvexPolygon2d currentSupportFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(swingSide.getOppositeSide());
      FramePoint2d currentSupportFootPolygonCentroid = new FramePoint2d(currentSupportFootPolygon.getCentroid());
      if (defaultStartingPosition)
      {
         FrameConvexPolygon2d currentSwingFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(swingSide);
         FramePoint2d currentSwingFootPolygonCentroid = currentSwingFootPolygon.getCentroid();

         FramePoint2d currentSupportPolygonCentroid = new FramePoint2d(currentSupportFootPolygonCentroid);
         currentSupportPolygonCentroid.add(currentSwingFootPolygonCentroid);
         currentSupportPolygonCentroid.scale(0.5);

         FrameEuclideanTrajectoryPoint currentTrajectoryPoint = new FrameEuclideanTrajectoryPoint();
         currentTrajectoryPoint.setPosition(currentSupportPolygonCentroid.toFramePoint());

         coPWayPoints.addTrajectoryPoint(packIntoTrajectoryPoint(0.0, currentSupportPolygonCentroid));
         setInitialCoPVelocity(new FrameVector(currentSoleZUpFrames.get(swingSide.getOppositeSide())));
      }
      else
      {
         coPWayPoints.addTrajectoryPoint(packIntoTrajectoryPoint(0.0, currentCoPPosition));
      }
      if (!footstepLocation.isEmpty())
      {
         computeReferenceCoPsForFootstep(coPWayPoints, coPOffsets.get(swingSide.getOppositeSide()), coPWayPointAlphas.get(swingSide.getOppositeSide()), currentSupportFootPolygonCentroid,
                                         footstepLocation.get(0).getSwingTime(), footstepLocation.get(0).getTransferTime());
         computeReferenceCoPsForUpcomingFootstep();
      }
   }

   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide swingSide, double time)
   {
      double percentageTime = time / footstepLocation.get(0).getSwingTime();
      int splineSegmentIndex = getSplineSegmentIndexFromTime(swingSide, percentageTime);
      if (!footstepLocation.isEmpty())
      {
         coPWayPoints.addTrajectoryPoint(packIntoTrajectoryPoint(time, currentCoPPosition));
         FramePoint2d centroidOfSupportFoot = new FramePoint2d(currentSoleZUpFrames.get(swingSide.getOppositeSide()));
         computeReferenceCoPsForFootstep(coPWayPoints, coPOffsets.get(swingSide.getOppositeSide()), coPWayPointAlphas.get(swingSide.getOppositeSide()), centroidOfSupportFoot,
                                         footstepLocation.get(0).getSwingTime(), splineSegmentIndex);
         computeReferenceCoPsForUpcomingFootstep();
      }
   }

   private int getSplineSegmentIndexFromTime(RobotSide swingSide, double percentageTime)
   {
      int splineSegmentIndex = 0;
      double t = 0.0;
      for (int i = 0; i < coPWayPointAlphas.size(); i++)
      {
         t = t + coPWayPointAlphas.get(swingSide.getOppositeSide()).get(i);
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
      int numberOfFootstepsToPlan = footstepLocation.size() - 1 > numberOfFootstepstoConsider.getIntegerValue() ? numberOfFootstepstoConsider.getIntegerValue()
            : footstepLocation.size() - 1;
      int index = 0;
      for (; index < numberOfFootstepsToPlan; index++)
      {
         FootstepTrajectoryPoint planningFootstep = footstepLocation.get(index);
         FramePoint2d footstepLocation = planningFootstep.getCentroidOfExpectedFootPolygonLocation().toFramePoint2d();
         computeReferenceCoPsForFootstep(coPWayPoints, coPOffsets.get(planningFootstep.getSupportSide()), coPWayPointAlphas.get(planningFootstep.getSupportSide()),
                                         footstepLocation, planningFootstep.getSwingTime(), planningFootstep.getTransferTime());
      }
      if (footstepLocation.size() - 1 <= numberOfFootstepstoConsider.getIntegerValue())
         computeReferenceCoPsForFinalTransfer(index);
   }

   private void computeReferenceCoPsForFinalTransfer(int indexOfFinalFootStep)
   {
      FootstepTrajectoryPoint planningFootstep = footstepLocation.get(indexOfFinalFootStep);
      FramePoint2d lastFootstepLocation = planningFootstep.getCentroidOfExpectedFootPolygonLocation().toFramePoint2d();

      FramePoint2d nextCoPLocation;
      if (footstepLocation.size() == 1)
      {
         FrameConvexPolygon2d lastSupportFootPolygon = currentSupportFootPolygonsInSoleZUpFrames.get(planningFootstep.getSupportSide());
         nextCoPLocation = lastSupportFootPolygon.getCentroidCopy();
      }
      else
      {
         nextCoPLocation = new FramePoint2d(footstepLocation.get(indexOfFinalFootStep - 1).getCentroidOfExpectedFootPolygonLocation().toFramePoint2d());
      }

      nextCoPLocation.add(lastFootstepLocation);
      nextCoPLocation.scale(0.5);
      coPWayPoints.addTrajectoryPoint(packIntoTrajectoryPoint(defaultFinalTransferDuration.getDoubleValue(), nextCoPLocation));
   }

   private void computeReferenceCoPsForFootstep(FrameEuclideanTrajectoryPointList coPListToPack, List<FrameVector2d> coPOffsets, List<Double> coPWayPointAlpha,
                                                FramePoint2d centroidOfFootstep, double swingTime, double transferTime)
   {
      FramePoint2d nextCoPLocation = new FramePoint2d(centroidOfFootstep);
      nextCoPLocation.add(coPOffsets.get(0));
      coPListToPack.addTrajectoryPoint(packIntoTrajectoryPoint(transferTime, nextCoPLocation));
      computeReferenceCoPsForFootstep(coPListToPack, coPOffsets, coPWayPointAlpha, centroidOfFootstep, swingTime, 1);
   }

   /**
    * 
    * @param coPListToPack
    * @param coPOffsets
    * @param coPWayPointAlpha
    * @param centroidOfFootstep
    * @param swingTime
    * @param coPOffsetIndex Must be greater than 1
    */
   private void computeReferenceCoPsForFootstep(FrameEuclideanTrajectoryPointList coPListToPack, List<FrameVector2d> coPOffsets, List<Double> coPWayPointAlpha,
                                                FramePoint2d centroidOfFootstep, double swingTime, int coPOffsetIndex)
   {
      for (int i = coPOffsetIndex; i < coPOffsets.size(); i++)
      {
         FramePoint2d nextCoPLocation = new FramePoint2d(centroidOfFootstep);
         nextCoPLocation.add(coPOffsets.get(i));
         coPListToPack.addTrajectoryPoint(packIntoTrajectoryPoint(coPWayPointAlpha.get(i - 1) * swingTime, nextCoPLocation));
      }
   }

   public List<FramePoint> getCoPs()
   {
      List<FramePoint> wayPointPositionList = new ArrayList<>(coPWayPoints.getNumberOfTrajectoryPoints());
      for (int i = 0; i < coPWayPoints.getNumberOfTrajectoryPoints(); i++)
         wayPointPositionList.add(coPWayPoints.getTrajectoryPoint(i).getPositionCopy());
      return wayPointPositionList;
   }

   public FramePoint getNextCoP()
   {
      return coPWayPoints.getTrajectoryPoint(0).getPositionCopy();
   }

   public void getNextCoP(FramePoint entryCMPToPack)
   {
      FramePoint nextCoP = coPWayPoints.getTrajectoryPoint(0).getPositionCopy();
      entryCMPToPack.setIncludingFrame(nextCoP);
   }

   @Override
   public CMPComponentType getComponentType()
   {
      return cmpComponentType;
   }

   @Override
   public List<YoPolynomial3D> getPolynomialTrajectory()
   {
      generatePolynomialCoefficients();
      return coPTrajectoryPolynomials;
   }

   private void generatePolynomialCoefficients()
   {
      coPTrajectoryPolynomials.clear();
      //TODO
   }

   @Override
   public void setInitialCoPVelocity(FrameVector initialCoPVelocity)
   {
      this.currentCoPVelocity = initialCoPVelocity.toFrameVector2d();
   }

   @Override
   public void setInitialCoPVelocity(FrameVector2d initiaCoPVelocity)
   {
      this.currentCoPVelocity = new FrameVector2d(initiaCoPVelocity);
   }

   @Override
   public void setInitialCoPPosition(FramePoint initialCoPPosition)
   {
      this.currentCoPPosition = initialCoPPosition.toFramePoint2d();
   }

   @Override
   public void setInitialCoPPosition(FramePoint2d initialCoPPosition)
   {
      this.currentCoPPosition = new FramePoint2d(initialCoPPosition);
   }

   private FrameEuclideanTrajectoryPoint packIntoTrajectoryPoint(double time, FramePoint2d position)
   {
      return new FrameEuclideanTrajectoryPoint(time, position.toFramePoint(), new FrameVector(position.getReferenceFrame()));
   }
}
