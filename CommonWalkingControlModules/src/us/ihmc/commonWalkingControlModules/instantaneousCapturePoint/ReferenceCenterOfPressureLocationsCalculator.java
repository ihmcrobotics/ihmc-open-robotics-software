package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.*;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ExtendedCapturePointPlannerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ReferenceCenterOfPressureLocationsCalculator
{
   // Some general hygiene declarations   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double CoP_Point_Size = 0.005;

   // Some Yo declarations
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private YoVariableRegistry parentRegistry;
   private String namePrefix;

   // Data storage
   private BipedSupportPolygons bipedSupportPolygons;
   
   // State variables 
   private BooleanYoVariable isDoneWalking;
   private IntegerYoVariable numberOfUpcomingFootsteps;
   private IntegerYoVariable numberOfFootstepstoConsider;
   
   private List<FootstepPoints> footCoPLocation = new ArrayList<>();
   private List<YoFramePoint2d> coPLocations  = new ArrayList<>(); 
   private List<FrameVector2d> coPOffsets = new ArrayList<>();
   
   //TODO add visualization for CoP trajectories
   private List<FootstepPoints> coPLocationsReadOnly = new ArrayList<>();
   
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
      numberOfUpcomingFootsteps = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);
      this.numberOfFootstepstoConsider = new IntegerYoVariable(namePrefix + "NumberOfFootstepsToConsider", registry);
      this.numberOfFootstepstoConsider.set(numberFootstepsToConsider);      
      this.parentRegistry.addChild(registry);      
   }
   
   public void initializeParameters(ExtendedCapturePointPlannerParameters icpPlannerParameters)
   {
      numberOfUpcomingFootsteps.set(icpPlannerParameters.getNumberOfFootstepsToConsider());      
      coPOffsets = icpPlannerParameters.getCoPOffsets();
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

   public void createVisualizerForConstantCoPs(YoGraphicsList yoGraphicsList, ArtifactList artifactList)
   {
      // TODO 
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
      // TODO Auto-generated method stub      
   }
   
   public void computeReferenceCoPForFootstep()
   {
      
   }
   
   public List<YoFramePoint2d> getCoPs()
   {      
      return coPLocations;
   }

   
   public YoFramePoint2d getNextCoP()
   {
      return coPLocations.get(0);
   }

   
   public void getNextCoP(FramePoint entryCMPToPack)
   {
      // TODO Auto-generated method stub      
   }
}
