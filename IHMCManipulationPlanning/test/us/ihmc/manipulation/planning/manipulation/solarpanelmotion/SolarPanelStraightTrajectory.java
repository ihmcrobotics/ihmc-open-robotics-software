package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;

public class SolarPanelStraightTrajectory
{
   private SolarPanelCleaningPose startPose;
   private SolarPanelCleaningPose endPose;
   private int numberOfWayPoints;
   private ArrayList<SolarPanelCleaningPose> wayPointPoses = new ArrayList<SolarPanelCleaningPose>();
   
   public SolarPanelStraightTrajectory(SolarPanelCleaningPose startPose, SolarPanelCleaningPose endPose, int numberOfWayPoints)
   {
      this.startPose = startPose;
      this.endPose = endPose;
      this.numberOfWayPoints = numberOfWayPoints;
      initialize();
   }
   
   private void initialize()
   {  
      for(int i=0;i<numberOfWayPoints;i++)
      {         
         Point3D startLocation = startPose.getLocation();
         Point3D endLocation = endPose.getLocation();
         Point3D wayPointLocation = new Point3D((endLocation.getX() - startLocation.getX())/(numberOfWayPoints-1)*i+startLocation.getX(), (endLocation.getY() - startLocation.getY())/(numberOfWayPoints-1)*i+startLocation.getY(), (endLocation.getZ() - startLocation.getZ())/(numberOfWayPoints-1)*i+startLocation.getZ());
         SolarPanelCleaningPose wayPointPose = new SolarPanelCleaningPose(wayPointLocation, 0.0, -Math.PI/0.25, 0.0);
      
         wayPointPoses.add(wayPointPose);
      }
   }
   
   public ArrayList<SolarPanelCleaningPose> getPoses()
   {
      return wayPointPoses;
   }
}
