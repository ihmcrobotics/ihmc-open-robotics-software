package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

public abstract class MultiJointArticulatedContactable implements Contactable
{
   private final String name;
   private final Robot robot;
   
   private final ArrayList<ArrayList<GroundContactPoint>> allGroundContactPoints = new ArrayList<ArrayList<GroundContactPoint>>();
   private final ArrayList<ArrayList<BooleanYoVariable>> contactsAvailable = new ArrayList<ArrayList<BooleanYoVariable>>();
   
   public MultiJointArticulatedContactable(String name, Robot robot)
   {
      this.name = name;
      this.robot = robot;
   }
   
   public abstract ArrayList<Joint> getJoints();
   
   public void createAvailableContactPoints(int groundIdentifier, ArrayList<Integer> totalContactPointsAvailable, double forceVectorScale, boolean addYoGraphicForceVectorsForceVectors)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = null;
      if (addYoGraphicForceVectorsForceVectors) yoGraphicsListRegistry = new YoGraphicsListRegistry();

      for(int i = 0; i < totalContactPointsAvailable.size(); i++)
      {         
         Joint joint = getJoints().get(i);

         for(int j = 0; j < totalContactPointsAvailable.get(i).intValue(); j++)
         {
            if(j == 0) 
            {
               allGroundContactPoints.add(i, new ArrayList<GroundContactPoint>());
               contactsAvailable.add(i, new ArrayList<BooleanYoVariable>());
            }
            
            GroundContactPoint contactPoint = new GroundContactPoint("contact_" + name + "_" + joint.getName() + "_" + j, robot.getRobotsYoVariableRegistry());
            joint.addGroundContactPoint(groundIdentifier, contactPoint);
            allGroundContactPoints.get(i).add(contactPoint);
            
            BooleanYoVariable contactAvailable = new BooleanYoVariable("contact_" + name + "_" + joint.getName() + "_" + j + "_avail", robot.getRobotsYoVariableRegistry());
            contactAvailable.set(true);
            contactsAvailable.get(i).add(contactAvailable);
            
            if (addYoGraphicForceVectorsForceVectors)
            {
               YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(name + "Point" + joint.getName() + j, contactPoint.getYoPosition(), 0.02, YoAppearance.Green());
               YoGraphicVector yoGraphicVector = new YoGraphicVector(name + "Force" +joint.getName() + j, contactPoint.getYoPosition(), contactPoint.getYoForce(), forceVectorScale, YoAppearance.Green());
               yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicPosition);
               yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicVector);               
            }
         }
      }
            
      if (addYoGraphicForceVectorsForceVectors)
      {
         robot.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      }
   }
   
   @Override
   public int getAndLockAvailableContactPoint()
   {
      for(int i = 0; i < allGroundContactPoints.size(); i++)
      {
         try
         {
            return getAndLockAvailableContactPoint(i);
         }
         catch(RuntimeException e)
         {
         }
      }
      
      throw new RuntimeException("No contact points are available");
   }
   
   public int getAndLockAvailableContactPoint(int jointIndex)
   {
      for(int j = 0; j < allGroundContactPoints.get(jointIndex).size(); j++)
      {
         BooleanYoVariable contactAvailable = contactsAvailable.get(jointIndex).get(j);

         if(contactAvailable.getBooleanValue())
         {
            contactAvailable.set(false);
            
            return j;
         }
      }
      
      throw new RuntimeException("No contact points are available, trying to access them for joint " + jointIndex);
   }
   
   @Override
   public void unlockContactPoint(GroundContactPoint groundContactPoint)
   {
      for(int i = 0; i < allGroundContactPoints.size(); i++)
      {
         for(int j = 0; j < allGroundContactPoints.get(i).size(); j++)
         {
            if(groundContactPoint == allGroundContactPoints.get(i).get(j))
            {
               BooleanYoVariable contactAvailable = contactsAvailable.get(i).get(j);
               if(!contactAvailable.getBooleanValue())
               {
                  contactAvailable.set(true);
                  
                  return;
               }
               else
               {
                  throw new RuntimeException("Returning a contact point that is already available!");
               }
            }
            
         }
      }
   }
   
   public GroundContactPoint getLockedContactPoint(int jointIndex, int contactPointIndex)
   {      
      if (contactsAvailable.get(jointIndex).get(contactPointIndex).getBooleanValue())
      {
         throw new RuntimeException("Trying to get a contact point that isn't checked out! - " + "Joint " + jointIndex + ", Contact Point " + contactPointIndex);
      }
      
      return allGroundContactPoints.get(jointIndex).get(contactPointIndex);
   }
   
   @Override
   public void updateContactPoints()
   {
      robot.update();
      robot.updateVelocities();
   }
}
