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
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ReferenceCenterOfPressureLocationsCalculator implements CMPComponentPolynomialTrajectoryPlannerInterface
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
   private IntegerYoVariable plannedCoPIndex;
   private IntegerYoVariable orderOfSplineInterpolation;
   private DoubleYoVariable defaultFinalTransferDuration;

   // Plan variables
   private List<FootstepPoints> footstepLocation = new ArrayList<>();
   private List<FramePoint2d> coPLocations = new ArrayList<>();
   private ArrayList<YoPolynomial3D> coPTrajectoryPolynomials = new ArrayList<>();
   private SideDependentList<List<FrameVector2d>> coPOffsets = new SideDependentList<>();
   
   private BipedSupportPolygons bipedSupportPolygons;
   private SideDependentList<ReferenceFrame> currentSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> currentSupportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();
   // TODO User customizable input declarations
   private final SideDependentList<YoFrameVector2d> coPUserOffsets = new SideDependentList<>();

   /**
    * Creates CoP planner object. Should be followed by call to {@code initializeParamters()} to pass planning parameters 
    * @param namePrefix
    */
   public ReferenceCenterOfPressureLocationsCalculator(String namePrefix)
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
         coPOffsets.put(side, icpPlannerParameters.getCoPOffsets(side));
         if (coPOffsets.get(side).size() != icpPlannerParameters.getNumberOfPointsPerFoot())
         {
            PrintTools.warn(this, "Mismatch in CoP Offsets size (" + coPOffsets.size() + " and number of CoP trajectory way points ("
                  + icpPlannerParameters.getNumberOfPointsPerFoot() + ")");
            if (coPOffsets.size() < icpPlannerParameters.getNumberOfPointsPerFoot())
            {
               for (int i = 0; i < icpPlannerParameters.getNumberOfPointsPerFoot() - coPOffsets.size(); i++)
                  coPOffsets.get(side).add(new FrameVector2d());
            }
            else
            {
               for (int i = coPOffsets.size() - icpPlannerParameters.getNumberOfPointsPerFoot(); i > 0; i--)
                  coPOffsets.get(side).remove(i);
            }
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
      this.plannedCoPIndex = new IntegerYoVariable(namePrefix + "PlannedCoPIndex", registry);      
   }

   /**
    * Creates a visualizer for the planned CoP trajectory
    * @param yoGraphicsList 
    * @param artifactList
    */
   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int i = 0; i < coPLocations.size(); i++)
      {
         YoFramePoint graphicFramePoint = new YoFramePoint(namePrefix + "CoPWayPoint" + i, worldFrame, parentRegistry);
         graphicFramePoint.set(coPLocations.get(i).getX(), coPLocations.get(i).getY(), 0.0);
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
            footstepLocation.add(new FootstepPoints(footstep, timing));
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
   }

   /**
    * Remove first footstep in the upcoming footstep queue from planner
    */
   public void removeFootStepQueueFront()
   {
      removeFootstep(0);
   }

   /**
    * Removes the specified number of footsteps from the queue front
    * @param numberOfFootstepsToRemove number of steps to remove
    */

   public void removeFootStepQueueFront(int numberOfFootstepsToRemove)
   {
      for (int i = 0; i < numberOfFootstepsToRemove; i++)
         removeFootstep(0);
   }

   /**
    * Removes specified footstep from upcoming footstep queue
    * @param index
    */
   public void removeFootstep(int index)
   {
      footstepLocation.remove(index);
   }

   /**
    * Clears the CoP plan and footsteps used to generate current plan
    */
   public void clear()
   {
      footstepLocation.clear();
      coPLocations.clear();
      plannedCoPIndex.set(0);
   }

   /**
    * Clears the CoP plan. Footsteps used to generate the plan are retained
    */
   public void clearPlan()
   {
      for (int i = 0; i < footstepLocation.size(); i++)
         footstepLocation.get(i).clearFootstepPoints();
      coPLocations.clear();
      plannedCoPIndex.set(0);
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
      return numberOfUpcomingFootsteps.getIntegerValue();
   }

   public int getNumberOfCoPPlannedFootSteps()
   {
      return plannedCoPIndex.getIntegerValue();
   }

   public void computeReferenceCoPsForUpcomingSteps(RobotSide side, int numberOfUpcomingFootSteps, int copIndex)
   {
      // TODO
   }

   private void computeReferenceCoPsForLastStep(ArrayList<FramePoint> coPPointsToPack, RobotSide supportSide, ReferenceFrame supportSoleFrame,
                                                FrameConvexPolygon2d defaultFootPolygon)
   {

   }

   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      int plannedCoPIndex = this.plannedCoPIndex.getIntegerValue();
      if (plannedCoPIndex + 1 < footstepLocation.size())
      {
         clearPlannedCoPs(plannedCoPIndex + 1, footstepLocation.size());
         PrintTools.warn(this.getClass(), "Replanning footsteps due to index-data mismatch");
      }
      else if (plannedCoPIndex + 1 > footstepLocation.size())
      {
         plannedCoPIndex = footstepLocation.size() - 1;
         PrintTools.warn(this.getClass(), "Resetting CoP plan index due to index-data mismatch");
      }
   }

   public void computeReferenceCoPsStartingFromDoubleSupport(RobotSide side, boolean atAStop)
   {
      // TODO
   }
  
   private void computeReferenceCoPsForFootstep(List<FramePoint2d> coPPointsToPack, ReferenceFrame supportSoleFrame,
                                                FrameConvexPolygon2d defaultFootPolygon, List<FrameVector2d> coPOffsets, int numberOfPointsPerFoot)
   {
      for (int i = 0; i < numberOfPointsPerFoot; i++)
      {
         FramePoint2d coPPoint = new FramePoint2d(supportSoleFrame);
         coPPoint.add(coPOffsets.get(i));
         coPPointsToPack.add(coPPoint.changeFrameAndProjectToXYPlaneCopy(worldFrame));
      }
   }

   private void clearPlannedCoPs(int plannedCoPCountFromIndex, int plannedCoPsStored)
   {
      if (plannedCoPCountFromIndex < plannedCoPsStored)
      {
         for (int i = plannedCoPsStored; i > plannedCoPCountFromIndex; i--)
         {
            footstepLocation.remove(i - 1);
         }
      }
   }

   public List<FramePoint2d> getCoPs()
   {
      return coPLocations;
   }

   public FramePoint2d getNextCoP()
   {
      return coPLocations.get(0);
   }

   public void getNextCoP(FramePoint entryCMPToPack)
   {
      FramePoint2d nextCoP = coPLocations.get(0);
      entryCMPToPack.setIncludingFrame(nextCoP.getReferenceFrame(), nextCoP.getX(), nextCoP.getY(), 0.0);
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
   }
}
