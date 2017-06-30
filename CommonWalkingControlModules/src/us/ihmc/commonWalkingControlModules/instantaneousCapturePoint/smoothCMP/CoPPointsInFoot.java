package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<YoFramePointInMultipleFrames> copLocations = new ArrayList<>();
   private final List<YoFramePoint> copLocationsInWorldFrameReadOnly = new ArrayList<>();

   private final int stepNumber;
   private final int maxNumberOfPointsPerFoot;

   public CoPPointsInFoot(int stepNumber, int maxNumberOfPointsPerFoot, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.stepNumber = stepNumber;
      this.maxNumberOfPointsPerFoot = maxNumberOfPointsPerFoot;

      for (int index = 0; index < maxNumberOfPointsPerFoot; index++)
      {
         YoFramePointInMultipleFrames constantCoP = new YoFramePointInMultipleFrames("step" + stepNumber + "CoP" + index, registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.add(constantCoP);
         copLocationsInWorldFrameReadOnly.add(constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
   }

   public void notifyVariableChangedListeners()
   {
      for (int i = 0; i < maxNumberOfPointsPerFoot; i++)
         copLocations.get(i).notifyVariableChangedListeners();
   }

   public void reset()
   {
      for (int i = 0; i < maxNumberOfPointsPerFoot; i++)
      {
         copLocations.get(i).setToNaN(worldFrame);
         copLocationsInWorldFrameReadOnly.get(i).setToNaN();
      }
   }

   public void setIncludingFrame(int waypointNumber, FramePoint location)
   {
      copLocations.get(waypointNumber).setIncludingFrame(location);
   }

   public void setIncludingFrame(int waypointNumber, YoFramePoint location)
   {
      copLocations.get(waypointNumber).setIncludingFrame(location);
   }

   public void setToNaN(int waypointNumber)
   {
      copLocations.get(waypointNumber).setToNaN();
   }

   public void set(int waypointNumber, FramePoint location)
   {
      copLocations.get(waypointNumber).set(location);
   }
   
   public void set(CoPPointsInFoot copPoints)
   {      
      List<YoFramePointInMultipleFrames> newCoPLocations = copPoints.copLocations;
      if(newCoPLocations.size() >  this.maxNumberOfPointsPerFoot)
         return;      
      int index = 0;
      for(; index < newCoPLocations.size(); index++)
         copLocations.get(index).setIncludingFrame(newCoPLocations.get(index));      
      for(;index < this.maxNumberOfPointsPerFoot; index++)
         copLocations.get(index).setToNaN();
   }

   public void set(int waypointNumber, YoFramePoint location)
   {
      copLocations.get(waypointNumber).set(location);
   }

   public YoFramePointInMultipleFrames get(int waypointNumber)
   {
      return copLocations.get(waypointNumber);
   }

   public YoFramePoint getWaypointInWorldFrameReadOnly(int waypointNumber)
   {
      return copLocationsInWorldFrameReadOnly.get(waypointNumber);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < maxNumberOfPointsPerFoot; i++)
         copLocations.get(i).changeFrame(desiredFrame);
   }

   public void switchCurrentReferenceFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < maxNumberOfPointsPerFoot; i++)
         copLocations.get(i).switchCurrentReferenceFrame(desiredFrame);
   }

   public void switchCurrentReferenceFrame(int waypointIndex, ReferenceFrame desiredFrame)
   {
      copLocations.get(waypointIndex).switchCurrentReferenceFrame(desiredFrame);
   }
}
