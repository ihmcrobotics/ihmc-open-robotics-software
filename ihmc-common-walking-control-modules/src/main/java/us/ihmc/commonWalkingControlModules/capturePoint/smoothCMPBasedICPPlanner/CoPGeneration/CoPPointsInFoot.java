package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class CoPPointsInFoot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3D zeroVector = new FrameVector3D();
   private static final FrameEuclideanTrajectoryPoint tempVariableForSetting = new FrameEuclideanTrajectoryPoint();
   private static final int maxNumberOfTrajectoryPoints = 10;
   
   private final List<CoPPointName> copPointsList = new ArrayList<>(maxNumberOfTrajectoryPoints); // List of CoP way points defined for this footstep. Hopefully this does not create garbage

   private final List<CoPTrajectoryPoint> copLocations = new ArrayList<>(maxNumberOfTrajectoryPoints); // Location of CoP points defined 
   private final List<YoFramePoint3D> copLocationsInWorldFrameReadOnly = new ArrayList<>(maxNumberOfTrajectoryPoints); // YoFramePoints for visualization

   private final YoFramePointInMultipleFrames swingFootCentroid;
   private final YoFramePointInMultipleFrames supportFootCentroid;
   private final String name;

   public CoPPointsInFoot(String namePrefix, int stepNumber, ReferenceFrame[] framesToRegister, YoVariableRegistry registry)
   {
      this.name = namePrefix + "Step" + stepNumber;
      for (int i = 0; i < maxNumberOfTrajectoryPoints; i++)
      {
         CoPTrajectoryPoint constantCoP = new CoPTrajectoryPoint(name + "CoP" + i, "", registry, framesToRegister);
         constantCoP.setToNaN();
         copLocations.add(constantCoP);
         copLocationsInWorldFrameReadOnly.add(constantCoP.buildUpdatedYoFramePointForVisualizationOnly());
      }
      swingFootCentroid = new YoFramePointInMultipleFrames(name + "SwingCentroid", registry, framesToRegister);
      supportFootCentroid = new YoFramePointInMultipleFrames(name + "SupportCentroid", registry, framesToRegister);
   }

   public void setupVisualizers(YoGraphicsList graphicsList, ArtifactList artifactList, double pointSize)
   {
      for (int i = 0; i < copLocationsInWorldFrameReadOnly.size(); i++)
      {
         YoFramePoint3D copLocation = copLocationsInWorldFrameReadOnly.get(i);
         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(copLocation.getNamePrefix(), copLocation, pointSize, YoAppearance.Green(),
                                                                     YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         graphicsList.add(yoGraphicPosition);
         artifactList.add(yoGraphicPosition.createArtifact());
      }
   }

   public void notifyVariableChangedListeners()
   {
      for (int i = 0; i < copLocations.size(); i++)
         copLocations.get(i).notifyVariableChangedListeners();
   }

   public void reset()
   {
      swingFootCentroid.setToNaN();
      supportFootCentroid.setToNaN();
      copPointsList.clear();
      for (int i = 0; i < copLocations.size(); i++)
      {
         copLocations.get(i).setToNaN(worldFrame);
         copLocationsInWorldFrameReadOnly.get(i).setToNaN();
      }
   }

   public void addWayPoint(CoPPointName copPointName)
   {
      this.copPointsList.add(copPointName);
   }
   
   public void setIncludingFrame(CoPPointsInFoot other)
   {
      this.swingFootCentroid.setIncludingFrame(other.swingFootCentroid);
      this.supportFootCentroid.setIncludingFrame(other.supportFootCentroid);
      this.copPointsList.clear();
      int index = 0;
      for(index = 0; index < other.copPointsList.size(); index++)
      {
         this.copPointsList.add(other.copPointsList.get(index));
         this.copLocations.get(index).setIncludingFrame(other.get(index));
      }
      for(; index < maxNumberOfTrajectoryPoints; index++)
         this.copLocations.get(index).setToNaN();
   }

   public void setIncludingFrame(int waypointIndex, double time, FramePoint3DReadOnly location)
   {
      copLocations.get(waypointIndex).registerReferenceFrame(location.getReferenceFrame());
      zeroVector.setToZero(location.getReferenceFrame());
      tempVariableForSetting.setIncludingFrame(time, location, zeroVector);
      copLocations.get(waypointIndex).setIncludingFrame(tempVariableForSetting);
   }

   public void setIncludingFrame(int waypointIndex, double time, CoPTrajectoryPoint location)
   {
      copLocations.get(waypointIndex).registerReferenceFrame(location.getReferenceFrame());
      zeroVector.setToZero(location.getReferenceFrame());
      tempVariableForSetting.setIncludingFrame(time, location.getPosition(), zeroVector);
      copLocations.get(waypointIndex).setIncludingFrame(tempVariableForSetting);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, FramePoint3DReadOnly location)
   {
      setIncludingFrame(copPointsList.size(), time, location);
      addWayPoint(copPointName);
   }

   public void addAndSetIncludingFrame(CoPPointName copPointName, double time, CoPTrajectoryPoint location)
   {
      setIncludingFrame(copPointsList.size(), time, location);
      addWayPoint(copPointName);
   }

   public void setToNaN(int waypointIndex)
   {
      copLocations.get(waypointIndex).setToNaN();
   }

   public CoPTrajectoryPoint get(int copPointIndex)
   {
      return copLocations.get(copPointIndex);
   }

   public FixedFramePoint3DBasics getWaypointInWorldFrameReadOnly(int copPointIndex)
   {
      return copLocationsInWorldFrameReadOnly.get(copPointIndex);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      swingFootCentroid.changeFrame(desiredFrame);
      supportFootCentroid.changeFrame(desiredFrame);
      for (int i = 0; i < copPointsList.size(); i++)
         copLocations.get(i).changeFrame(desiredFrame);
   }

   public void registerReferenceFrame(ReferenceFrame newReferenceFrame)
   {
      swingFootCentroid.registerReferenceFrame(newReferenceFrame);
      supportFootCentroid.registerReferenceFrame(newReferenceFrame);
      for (int i = 0; i < copPointsList.size(); i++)
         copLocations.get(i).registerReferenceFrame(newReferenceFrame);
   }

   public void switchCurrentReferenceFrame(int waypointIndex, ReferenceFrame desiredFrame)
   {
      copLocations.get(waypointIndex).switchCurrentReferenceFrame(desiredFrame);
   }

   public void setSwingFootLocation(FramePoint3DReadOnly footLocation)
   {
      this.swingFootCentroid.setIncludingFrame(footLocation);
   }

   public void getSwingFootLocation(FramePoint3D footLocationToPack)
   {
      footLocationToPack.setIncludingFrame(swingFootCentroid);
   }

   public void setSupportFootLocation(FramePoint3DReadOnly footLocation)
   {
      this.supportFootCentroid.setIncludingFrame(footLocation);
   }

   public void getSupportFootLocation(FramePoint3D footLocationToPack)
   {
      footLocationToPack.setIncludingFrame(supportFootCentroid);
   }
   
   public void setFeetLocation(FramePoint3DReadOnly swingFootLocation, FramePoint3DReadOnly supportFootLocation)
   {
      setSwingFootLocation(swingFootLocation);
      setSupportFootLocation(supportFootLocation);
   }

   public List<CoPPointName> getCoPPointList()
   {
      return copPointsList;
   }

   public void getFinalCoPPosition(FixedFramePoint3DBasics tempFinalICPToPack)
   {
      copLocations.get(copPointsList.size() - 1).getPosition(tempFinalICPToPack);
   }

   public boolean isEmpty()
   {
      return copPointsList.isEmpty();
   }

   public int getNumberOfCoPPoints()
   {
      return copPointsList.size();
   }
   
   public String toString()
   {
      String string = name;
      for(int i = 0; i < getNumberOfCoPPoints(); i++)
      {
         string += getCoPPointList().get(i).toString() + ": " + get(i).toString() + "\n";
      }
      return string;
   }
   
   public String toString2()
   {
      String string = name;
      for(int i = 0; i < getNumberOfCoPPoints(); i++)
      {
         string += getCoPPointList().get(i).toString() + "\n";
      }
      return string;
   }

   
}
