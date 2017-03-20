package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;

public class SolarPanelStraightTrajectory
{
   private SolarPanel solarPanel;
   private int numberOfWayPoints;
   
   public SolarPanelStraightTrajectory(SolarPanel solarPanel, int numberOfWayPoints)
   {
      this.solarPanel = solarPanel;
      this.numberOfWayPoints = numberOfWayPoints;    
   }
   
}
