package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.*;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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

   // State variables 
   private BooleanYoVariable isDoneWalking;
   private IntegerYoVariable numberOfUpcomingFootsteps;
   private List<FootstepPoints> coPLocationsinFootFrame = new ArrayList<>();
   private List<Footstep> upcomingFootsteps = new ArrayList<>();
   
   // User customizable input declarations
   private List<YoFrameVector2d> CoPOffsets;
   
   private final SideDependentList<YoFrameVector2d> CoPUserOffsets = new SideDependentList<>();
   
   public ReferenceCenterOfPressureLocationsCalculator(String namePrefix, BipedSupportPolygons bipedSupportPolygons,
                                                       SideDependentList<? extends ContactablePlaneBody> contactableFeet, int numberFootstepsToConsider,
                                                       YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.parentRegistry = parentRegistry;      
      isDoneWalking = new BooleanYoVariable(namePrefix + "IsDoneWalking", registry);
      numberOfUpcomingFootsteps = new IntegerYoVariable(namePrefix + "NumberOfUpcomingFootsteps", registry);
      
      this.parentRegistry.addChild(registry);      
   }
   
   public void initializeParameters(CapturePointPlannerParameters icpPlannerParameters)
   {
      
   }

   public void updateYoVariables()
   {
      
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
}
