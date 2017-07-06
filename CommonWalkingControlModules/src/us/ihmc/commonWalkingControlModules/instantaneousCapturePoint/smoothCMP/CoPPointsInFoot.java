package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<CoPTrajectoryPoint> copLocations = new ArrayList<>();
   private final List<YoFramePoint> copLocationsInWorldFrameReadOnly = new ArrayList<>();
   
   private final int stepNumber;
   private final int maxNumberOfPointsPerFoot;

   private final FramePoint footStepCentroid;

   public CoPPointsInFoot(int stepNumber, int maxNumberOfPointsPerFoot, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.stepNumber = stepNumber;
      this.maxNumberOfPointsPerFoot = maxNumberOfPointsPerFoot;

      for (int index = 0; index < maxNumberOfPointsPerFoot; index++)
      {
         CoPTrajectoryPoint constantCoP = new CoPTrajectoryPoint("step" + stepNumber + "CoP" + index, "", registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.add(constantCoP);
         copLocationsInWorldFrameReadOnly.add(constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
      footStepCentroid = new FramePoint();
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

   public void setIncludingFrame(int waypointNumber, CoPTrajectoryPoint location)
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
   
   public void set(int waypointNumber, YoFramePoint location)
   {
      copLocations.get(waypointNumber).set(location);
   }
   
   public void set(int waypointNumber, CoPTrajectoryPoint location)
   {
      copLocations.get(waypointNumber).set(location);
   }

   public void set(CoPPointsInFoot copPoints)
   {      
      List<CoPTrajectoryPoint> newCoPLocations = copPoints.copLocations;
      if(newCoPLocations.size() >  this.maxNumberOfPointsPerFoot)
         return;      
      int index = 0;
      for(; index < newCoPLocations.size(); index++)
         copLocations.get(index).setIncludingFrame(newCoPLocations.get(index));      
      for(;index < this.maxNumberOfPointsPerFoot; index++)
         copLocations.get(index).setToNaN();
   }

   public CoPTrajectoryPoint get(int waypointNumber)
   {
      return copLocations.get(waypointNumber);
   }

   public YoFramePoint getWaypointInWorldFrameReadOnly(int waypointNumber)
   {
      return copLocationsInWorldFrameReadOnly.get(waypointNumber);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      footStepCentroid.changeFrame(desiredFrame);
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
      footLocationToPack.setIncludingFrame(footStepCentroid);
   }
   
   public String toString()
   {
      String output = "FootstepLocation: " + footStepCentroid.toString() + "\n";
      for(int i = 0; i < copLocations.size(); i++)
         output += "Point " + i + " : " + copLocations.get(i) + "\n";
      return output;
   }
}
