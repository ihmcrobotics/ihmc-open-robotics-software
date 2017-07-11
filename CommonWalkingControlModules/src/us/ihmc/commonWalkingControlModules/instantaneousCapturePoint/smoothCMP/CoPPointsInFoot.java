package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.CoPTrajectoryPoint;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector zeroVector = new FrameVector();
   
   private final List<CoPPointName> copPointsList = new ArrayList<>(CoPPointName.values.length); // List of CoP way points defined for this footstep. Hopefully this does not create garbage
   private final EnumMap<CoPPointName, CoPTrajectoryPoint> copLocations = new EnumMap<>(CoPPointName.class); // Location of CoP points defined 
   private final EnumMap<CoPPointName, YoFramePoint> copLocationsInWorldFrameReadOnly = new EnumMap<>(CoPPointName.class); // YoFramePoints for visualization

   private final int stepNumber;

   private final YoFramePointInMultipleFrames footStepCentroid;

   public CoPPointsInFoot(int stepNumber, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.stepNumber = stepNumber;

      for (int i = 0; i < CoPPointName.values.length; i++)
      {
         CoPPointName copPointName = CoPPointName.values[i];
         CoPTrajectoryPoint constantCoP = new CoPTrajectoryPoint("step" + stepNumber + "CoP" + copPointName.toString(), "", registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.put(copPointName, constantCoP);
         copLocationsInWorldFrameReadOnly.put(copPointName, constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
      footStepCentroid = new YoFramePointInMultipleFrames("step" + stepNumber + "swingCentroid", registry, framesToRegister);
   }

   public void notifyVariableChangedListeners()
   {
      for (int i = 0; i < CoPPointName.values.length; i++)
         copLocations.get(CoPPointName.values[i]).notifyVariableChangedListeners();
   }

   public void reset()
   {
      copPointsList.clear();
      for (int i = 0; i < CoPPointName.values.length; i++)
      {
         copLocations.get(CoPPointName.values[i]).setToNaN(worldFrame);
         copLocationsInWorldFrameReadOnly.get(CoPPointName.values[i]).setToNaN();
      }
   }

   public void addWayPoint(CoPPointName copPointName)
   {
      this.copPointsList.add(copPointName);
   }

   public void setIncludingFrame(CoPPointName copPointName, double time, FramePoint location)
   {
      copLocations.get(copPointName).set(time, location, zeroVector);
   }

   public void setIncludingFrame(CoPPointName copPointName, double time, YoFramePoint location)
   {
      copLocations.get(copPointName).set(time, location.getFrameTuple(), zeroVector);
   }

   public void setIncludingFrame(CoPPointName copPointName, double time, CoPTrajectoryPoint location)
   {
      copLocations.get(copPointName).set(time, location.getPosition().getFrameTuple(), zeroVector);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, FramePoint location)
   {
      addWayPoint(copPointName);
      setIncludingFrame(copPointName, time, location);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, YoFramePoint location)
   {
      addWayPoint(copPointName);
      setIncludingFrame(copPointName, time, location);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, CoPTrajectoryPoint location)
   {
      addWayPoint(copPointName);
      setIncludingFrame(copPointName, time, location);
   }

   public void setToNaN(CoPPointName copPointName)
   {
      copLocations.get(copPointName).setToNaN();
   }

   public void addWayPoints(CoPPointName[] copPointNames)
   {
      for (int i = 0; i < copPointNames.length; i++)
         this.copPointsList.add(copPointNames[i]);
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
   public void setIncludingFrame(CoPPointsInFoot other)
   {
      this.footStepCentroid.setIncludingFrame(other.footStepCentroid);
      for (int i = 0; i < CoPPointName.values.length; i++)
         this.copLocations.get(CoPPointName.values[i]).setIncludingFrame(other.get(CoPPointName.values[i]));
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
      for (int i = 0; i < CoPPointName.values.length; i++)
         copLocations.get(CoPPointName.values[i]).changeFrame(desiredFrame);
   }

   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      footStepCentroid.registerReferenceFrame(newReferenceFrame);
      for (int i = 0; i < CoPPointName.values.length; i++)
         copLocations.get(CoPPointName.values[i]).registerReferenceFrame(newReferenceFrame);      
   }
   
   public void switchCurrentReferenceFrame(ReferenceFrame desiredFrame)
   {
      footStepCentroid.switchCurrentReferenceFrame(desiredFrame);
      for (int i = 0; i < CoPPointName.values.length; i++)
         copLocations.get(CoPPointName.values[i]).switchCurrentReferenceFrame(desiredFrame);
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
      for (int i = 0; i < CoPPointName.values.length; i++)
         output += CoPPointName.values[i].toString() + " : " + copLocations.get(CoPPointName.values[i]) + "\n";
      return output;
   }

   public List<CoPPointName> getCoPPointList()
   {
      return copPointsList;
   }

   public CoPPointName getEntryCoPName()
   {
      return copPointsList.get(0);
   }

   public CoPPointName getExitCoPName()
   {
      return copPointsList.get(copPointsList.size() - 1);
   }
}
