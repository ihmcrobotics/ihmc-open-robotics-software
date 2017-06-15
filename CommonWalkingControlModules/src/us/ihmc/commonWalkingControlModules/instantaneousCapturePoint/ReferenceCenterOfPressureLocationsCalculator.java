package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ReferenceCenterOfPressureLocationsCalculator
{
   // Some general hygiene declarations   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CoPPointSize = 0.005;

   // Some Yo declarations
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private YoVariableRegistry parentRegistry;
   private String namePrefix;

   // Data storage
   private BipedSupportPolygons bipedSupportPolygons;
   
   // State variables 
   private BooleanYoVariable isDoneWalking;
   private IntegerYoVariable numberOfUpcomingFootsteps;
   private IntegerYoVariable numberOfPointsPerFoot;
   private IntegerYoVariable numberOfFootstepstoConsider;
   
   private List<FootstepPoints> footCoPLocation = new ArrayList<>();
   private List<FramePoint2d> coPLocations  = new ArrayList<>(); 
   private List<FrameVector2d> coPOffsets = new ArrayList<>();
   private SideDependentList<FrameConvexPolygon2d> supportFootPolygonsInSoleZUpFrames = new SideDependentList<>();
   private SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();
   
   private List<Footstep> upcomingFootsteps = new ArrayList<>();  
   
   // TODO User customizable input declarations
   private final SideDependentList<YoFrameVector2d> CoPUserOffsets = new SideDependentList<>();
   
   public ReferenceCenterOfPressureLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons,
                                                       SideDependentList<? extends ContactablePlaneBody> contactableFeet, int numberFootstepsToConsider,
                                                       YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.parentRegistry = parentRegistry;
      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);
      
      for(RobotSide side : RobotSide.values)
      {
         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFeet.get(side).getContactPoints2d());
         defaultFootPolygons.put(side, defaultFootPolygon);
         supportFootPolygonsInSoleZUpFrames.put(side, bipedSupportPolygons.getFootPolygonInSoleZUpFrame(side));         
      }
      
      this.numberOfUpcomingFootsteps = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfPointsPerFoot = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfFootstepstoConsider = new IntegerYoVariable(namePrefix + "NumberOfFootstepsToConsider", registry);
      this.numberOfFootstepstoConsider.set(numberFootstepsToConsider);      
      this.parentRegistry.addChild(registry);      
   }
   
   public void initializeParameters(ExtendedCapturePointPlannerParameters icpPlannerParameters)
   {
      this.numberOfUpcomingFootsteps.set(icpPlannerParameters.getNumberOfFootstepsToConsider());      
      this.numberOfPointsPerFoot.set(icpPlannerParameters.getNumberOfPointsPerFoot());
      this.coPOffsets = icpPlannerParameters.getCoPOffsets();
      
      if(coPOffsets.size() != icpPlannerParameters.getNumberOfPointsPerFoot())
      {
         PrintTools.warn(this, "Mismatch in CoP Offsets size (" + coPOffsets.size() + " and number of CoP trajectory way points (" + icpPlannerParameters.getNumberOfPointsPerFoot() + ")");
         if(coPOffsets.size() < icpPlannerParameters.getNumberOfPointsPerFoot())
         {
            for(int i = 0; i < icpPlannerParameters.getNumberOfPointsPerFoot() - coPOffsets.size(); i++)
               coPOffsets.add(new FrameVector2d());
         }
         else
         {
            for(int i = coPOffsets.size() - icpPlannerParameters.getNumberOfPointsPerFoot(); i>0; i--)               
               coPOffsets.remove(i);
         }
      }
   }

   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      for (int i = 0; i < coPLocations.size(); i++)
      {
         YoFramePoint graphicFramePoint = new YoFramePoint(namePrefix +"CoPWayPoint" + i, worldFrame, parentRegistry);
         graphicFramePoint.set(coPLocations.get(i).getX(), coPLocations.get(i).getY(), 0.0);
         YoGraphicPosition entryCMPViz = new YoGraphicPosition(namePrefix + "GraphicCoPWaypoint" + i, graphicFramePoint, CoPPointSize, YoAppearance.Green(),
               GraphicType.SOLID_BALL);
         yoGraphicsList.add(entryCMPViz);
         artifactList.add(entryCMPViz.createArtifact());
      }      
   }
   
   public void addUpcomingFootstep(Footstep footstep)
   {
      if(footstep != null)
      {
         if(!footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
            upcomingFootsteps.add(footstep);
         else
            PrintTools.warn(this, "Received bad footstep: " + footstep);
      }
   }
   
   public void clear()
   {
      upcomingFootsteps.clear();
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

   public void computeReferenceCoPsStartingFromDoubleSupport(boolean atAStop, RobotSide transferToSide)
   {
      // TODO Auto-generated method stub      
   }
   
   public void computeReferenceCoPsStartingFromSingleSupport(RobotSide supportSide)
   {
      // TODO 
      footCoPLocation.clear();
      coPLocations.clear();
      
   }
   
   public void computeReferenceCoPsStartingFromDoubleSupport(RobotSide side, boolean atAStop)
   {
      // TODO 
      footCoPLocation.clear();
      coPLocations.clear();      
   }
   
   private void computeReferenceCoPsForUpcomingFootSteps(RobotSide side, int numberOfUpcomingFootSteps, int copIndex)   
   {
      // TODO 
      
   }
   
   private void computeReferenceCoPsForFootstep(ArrayList<FramePoint> coPPointsToPack, RobotSide side, ReferenceFrame soleFrame, FrameConvexPolygon2d defaultFootPolygon, boolean isLastUpcomingStep)
   {
      FootstepPoints newFootstepCoPs = new FootstepPoints(side, soleFrame);
      if(!isLastUpcomingStep)
      {
         for(int i=0; i<numberOfPointsPerFoot.getIntegerValue();i++)
         {  
            FramePoint2d coPPoint = new FramePoint2d(soleFrame);
            coPPoint.add(coPOffsets.get(i));
            newFootstepCoPs.addFootstepPoint(coPPoint);
            coPLocations.add(coPPoint.changeFrameAndProjectToXYPlaneCopy(worldFrame));
         }         
      }
      else
      {
         // TODO 
         
      }
      footCoPLocation.add(newFootstepCoPs);         
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
}
