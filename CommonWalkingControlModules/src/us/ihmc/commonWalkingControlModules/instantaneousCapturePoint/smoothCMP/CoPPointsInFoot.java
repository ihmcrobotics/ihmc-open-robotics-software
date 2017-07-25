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
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector zeroVector = new FrameVector();

   private final List<CoPPointName> copPointsList = new ArrayList<>(); // List of CoP way points defined for this footstep. Hopefully this does not create garbage
   private final List<CoPTrajectoryPoint> copLocations = new ArrayList<>(); // Location of CoP points defined
   private final List<YoFramePoint> copLocationsInWorldFrameReadOnly = new ArrayList<>(); // YoFramePoints for visualization

   private final FrameEuclideanTrajectoryPoint tempVariableForSetting = new FrameEuclideanTrajectoryPoint();

   private final int stepNumber;

   public CoPPointsInFoot(int stepNumber, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this(stepNumber, 6, framesToRegister, registry);
   }

   public CoPPointsInFoot(int stepNumber, int size, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.stepNumber = stepNumber;

      for (int i = 0; i < size; i++)
      {
         CoPTrajectoryPoint constantCoP = new CoPTrajectoryPoint("step" + stepNumber + "CoP" + i, "", registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.add(constantCoP);
         copLocationsInWorldFrameReadOnly.add(constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
   }

   public void notifyVariableChangedListeners()
   {
      for (int i = 0; i < copLocations.size(); i++)
         copLocations.get(i).notifyVariableChangedListeners();
   }

   public void reset()
   {
      copPointsList.clear();
      for (int i = 0; i < copLocations.size(); i++)
      {
         copLocations.get(i).setToNaN(worldFrame);
         copLocationsInWorldFrameReadOnly.get(i).setToNaN();
      }
   }

   public int addWayPoint(CoPPointName copPointName)
   {
      int waypointIndex = copPointsList.size();
      this.copPointsList.add(copPointName);

      return waypointIndex;
   }

   public void setIncludingFrame(int waypointIndex, double time, FramePoint location)
   {
      copLocations.get(waypointIndex).registerReferenceFrame(location.getReferenceFrame());
      zeroVector.setToZero(location.getReferenceFrame());;
      tempVariableForSetting.setIncludingFrame(time, location, zeroVector);
      copLocations.get(waypointIndex).setIncludingFrame(tempVariableForSetting);
   }

   public void setIncludingFrame(int waypointIndex, double time, YoFramePoint location)
   {
      copLocations.get(waypointIndex).registerReferenceFrame(location.getReferenceFrame());
      zeroVector.setToZero(location.getReferenceFrame());
      tempVariableForSetting.setIncludingFrame(time, location.getFrameTuple(), zeroVector);
      copLocations.get(waypointIndex).setIncludingFrame(tempVariableForSetting);
   }

   public void setIncludingFrame(int waypointIndex, double time, CoPTrajectoryPoint location)
   {
      copLocations.get(waypointIndex).registerReferenceFrame(location.getReferenceFrame());
      zeroVector.setToZero(location.getReferenceFrame());;
      tempVariableForSetting.setIncludingFrame(time, location.getPosition().getFrameTuple(), zeroVector);
      copLocations.get(waypointIndex).setIncludingFrame(tempVariableForSetting);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, FramePoint location)
   {
      int waypointIndex = addWayPoint(copPointName);
      setIncludingFrame(waypointIndex, time, location);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, YoFramePoint location)
   {
      int waypointIndex = addWayPoint(copPointName);
      setIncludingFrame(waypointIndex, time, location);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, CoPTrajectoryPoint location)
   {
      int waypointIndex = addWayPoint(copPointName);
      setIncludingFrame(waypointIndex, time, location);
   }

   public void setToNaN(int waypointIndex)
   {
      copLocations.get(waypointIndex).setToNaN();
   }

   public void addWayPoints(CoPPointName[] copPointNames)
   {
      for (int i = 0; i < copPointNames.length; i++)
         this.copPointsList.add(copPointNames[i]);
   }

   public void setIncludingFrame(CoPPointsInFoot other)
   {
      for (int i = 0; i < other.copPointsList.size(); i++)
         this.copLocations.get(i).setIncludingFrame(other.get(i));
   }

   public CoPTrajectoryPoint get(int waypointIndex)
   {
      return copLocations.get(waypointIndex);
   }

   public YoFramePoint getWaypointInWorldFrameReadOnly(int waypointIndex)
   {
      return copLocationsInWorldFrameReadOnly.get(waypointIndex);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < copLocations.size(); i++)
         copLocations.get(i).changeFrame(desiredFrame);
   }

   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      for (int i = 0; i < copLocations.size(); i++)
         copLocations.get(i).registerReferenceFrame(newReferenceFrame);
   }
   
   public void switchCurrentReferenceFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < copLocations.size(); i++)
         copLocations.get(i).switchCurrentReferenceFrame(desiredFrame);
   }

   public void switchCurrentReferenceFrame(int waypointIndex, ReferenceFrame desiredFrame)
   {
      copLocations.get(waypointIndex).switchCurrentReferenceFrame(desiredFrame);
   }

   public String toString()
   {
      String output = "";
      for (int i = 0; i < copPointsList.size(); i++)
         output += copPointsList.get(i) + " : " + copLocations.get(i) + "\n";
      return output;
   }

   public List<CoPPointName> getCoPPointList()
   {
      return copPointsList;
   }
}
