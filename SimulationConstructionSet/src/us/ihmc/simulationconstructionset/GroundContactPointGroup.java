package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

public class GroundContactPointGroup
{
   private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<GroundContactPoint>();
   private ArrayList<GroundContactPoint> groundContactPointsInContact = new ArrayList<GroundContactPoint>();
   private ArrayList<GroundContactPoint> groundContactPointsNotInContact = new ArrayList<GroundContactPoint>();

   public GroundContactPointGroup()
   {
   }

   public void addGroundContactPoint(GroundContactPoint groundContactPoint)
   {
      groundContactPoints.add(groundContactPoint);
   }

   public ArrayList<GroundContactPoint> getGroundContactPoints()
   {
      return groundContactPoints;
   }

   public void decideGroundContactPointsInContact()
   {
      groundContactPointsInContact.clear();
      groundContactPointsNotInContact.clear();

      for (int i = 0; i < groundContactPoints.size(); i++)
      {
         GroundContactPoint point = groundContactPoints.get(i);
         if (point.isInContact())
            groundContactPointsInContact.add(point);
         else
            groundContactPointsNotInContact.add(point);
      }
   }

   public ArrayList<GroundContactPoint> getGroundContactPointsInContact()
   {
      return groundContactPointsInContact;
   }

   public ArrayList<GroundContactPoint> getGroundContactPointsNotInContact()
   {
      return groundContactPointsNotInContact;
   }

}
