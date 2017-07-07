package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<CoPPointName> copPointsList = new RecyclingArrayList(CoPPointName.values.length, CoPPointName.class); // List of CoP way points defined for this footstep
   private final EnumMap<CoPPointName, CoPTrajectoryPoint> copLocations = new EnumMap<>(CoPPointName.class); // Location of CoP points defined 
   private final EnumMap<CoPPointName, YoFramePoint> copLocationsInWorldFrameReadOnly = new EnumMap<>(CoPPointName.class);  // YoFramePoints for visualization
   
   private final int stepNumber;

   private final YoFramePointInMultipleFrames footStepCentroid;

   public CoPPointsInFoot(int stepNumber, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.stepNumber = stepNumber;
      this.copPointsList.clear();
      
      for(CoPPointName copPointName: CoPPointName.values)
      {
         CoPTrajectoryPoint constantCoP = new CoPTrajectoryPoint("step" + stepNumber + "CoP" + copPointName.toString(), "", registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.put(copPointName, constantCoP);
         copLocationsInWorldFrameReadOnly.put(copPointName, constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
      footStepCentroid = new YoFramePointInMultipleFrames("step" + stepNumber + "swingCentroid", registry, framesToRegister);
   }

   public void notifyVariableChangedListeners()
   {
      for(CoPPointName copPointName : CoPPointName.values)
         copLocations.get(copPointName).notifyVariableChangedListeners();
   }

   public void reset()
   {
      for(CoPPointName copPointName : CoPPointName.values)
      {
         copPointsList.clear();
         copLocations.get(copPointName).setToNaN(worldFrame);
         copLocationsInWorldFrameReadOnly.get(copPointName).setToNaN();
      }
   }

   public void setIncludingFrame(CoPPointName copPointName, FramePoint location)
   {
      //TODO this accesses the frame tuple directly without matching or setting the frame. Needs to be replaced with something more robust that compares frames.
      copLocations.get(copPointName).setPosition(location);
   }

   public void setIncludingFrame(CoPPointName copPointName, YoFramePoint location)
   {
      //TODO this accesses the frame tuple directly without matching or setting the frame. Needs to be replaced with something more robust that compares frames.
      copLocations.get(copPointName).setPosition(location.getFrameTuple());
   }

   public void setIncludingFrame(CoPPointName copPointName, CoPTrajectoryPoint location)
   {
      copLocations.get(copPointName).setIncludingFrame(location);
   }
   
   public void setToNaN(CoPPointName copPointName)
   {
      copLocations.get(copPointName).setToNaN();
   }

//   public void set(int waypointNumber, FramePoint location)
//   {
//      copLocations.get(waypointNumber).set(location);
//   }
//   
//   public void set(int waypointNumber, YoFramePoint location)
//   {
//      copLocations.get(waypointNumber).set(location);
//   }
//   
//   public void set(int waypointNumber, CoPTrajectoryPoint location)
//   {
//      copLocations.get(waypointNumber).set(location);
//   }
//
   public void set(CoPPointsInFoot copPointsInFoot)
   {      
      for(CoPPointName copPointName : CoPPointName.values)
      {
         this.copLocations.get(copPointName).set(copPointsInFoot.get(copPointName));
      }
   }

   public CoPTrajectoryPoint get(CoPPointName copPointName)
   {
      return copLocations.get(copPointName);
   }

   public YoFramePoint getWaypointInWorldFrameReadOnly(CoPPointName copPointName)
   {
      return copLocationsInWorldFrameReadOnly.get(copPointName);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      footStepCentroid.changeFrame(desiredFrame);
      for(CoPPointName copPointName : CoPPointName.values)
         copLocations.get(copPointName).changeFrame(desiredFrame);
   }

   public void switchCurrentReferenceFrame(ReferenceFrame desiredFrame)
   {
      footStepCentroid.switchCurrentReferenceFrame(desiredFrame);
      for(CoPPointName copPointName : CoPPointName.values)
         copLocations.get(copPointName).switchCurrentReferenceFrame(desiredFrame);
   }

   public void switchCurrentReferenceFrame(int waypointIndex, ReferenceFrame desiredFrame)
   {
      copLocations.get(waypointIndex).switchCurrentReferenceFrame(desiredFrame);
   }
   
   public void setFootLocation(FramePoint footLocation)
   {
      this.footStepCentroid.setIncludingFrame(footLocation);
   }
   
   public void setFootLocation(FramePoint2d footLocation)
   {
      this.footStepCentroid.setXYIncludingFrame(footLocation);
   }
   
   public void getFootLocation(FramePoint footLocationToPack)
   {
      footLocationToPack.setIncludingFrame(footStepCentroid.getFrameTuple());
   }
   
   public String toString()
   {
      String output = "FootstepLocation: " + footStepCentroid.toString() + "\n";
      for(CoPPointName copPointName : copPointsList)
         output += copPointName.toString() + " : " + copLocations.get(copPointName) + "\n";
      return output;
   }

   public List<CoPPointName> getCoPPointList()
   {
      return copPointsList;
   }
}
