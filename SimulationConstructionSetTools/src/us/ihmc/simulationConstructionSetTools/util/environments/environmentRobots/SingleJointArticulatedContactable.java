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

public abstract class SingleJointArticulatedContactable implements Contactable
{
   private final String name;
   private final Robot robot;
   
   private final ArrayList<GroundContactPoint> allGroundContactPoints = new ArrayList<GroundContactPoint>();
   private final ArrayList<BooleanYoVariable> contactsAvailable = new ArrayList<BooleanYoVariable>();

   public SingleJointArticulatedContactable(String name, Robot robot)
   {
      this.name = name;
      this.robot = robot;
   }

   public abstract Joint getJoint();

   public void createAvailableContactPoints(int groupIdentifier, int totalContactPointsAvailable, double forceVectorScale, boolean addYoGraphicForceVectorsForceVectors)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = null;
      if (addYoGraphicForceVectorsForceVectors) yoGraphicsListRegistry = new YoGraphicsListRegistry();

      for (int i = 0; i < totalContactPointsAvailable; i++)
      {
         GroundContactPoint contactPoint = new GroundContactPoint("contact_" + name + "_" + i, robot.getRobotsYoVariableRegistry());
         getJoint().addGroundContactPoint(groupIdentifier, contactPoint);
         allGroundContactPoints.add(contactPoint);

         BooleanYoVariable contactAvailable = new BooleanYoVariable("contact_" + name + "_" + i + "_avail", robot.getRobotsYoVariableRegistry());
         contactAvailable.set(true);
         contactsAvailable.add(contactAvailable);

         if (addYoGraphicForceVectorsForceVectors)
         {
            YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(name + "Point" + i, contactPoint.getYoPosition(), 0.02, YoAppearance.Green());
            YoGraphicVector yoGraphicVector = new YoGraphicVector(name + "Force" + i, contactPoint.getYoPosition(), contactPoint.getYoForce(), forceVectorScale, YoAppearance.Green());
            yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicPosition);
            yoGraphicsListRegistry.registerYoGraphic(name, yoGraphicVector);
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
      for (int i = 0; i < allGroundContactPoints.size(); i++)
      {
         BooleanYoVariable contactAvailable = contactsAvailable.get(i);

         if (contactAvailable.getBooleanValue())
         {
            contactAvailable.set(false);

            return i;
         }
      }

      throw new RuntimeException("No contact points are available");
   }

   @Override
   public void unlockContactPoint(GroundContactPoint groundContactPoint)
   {
      for (int i = 0; i < allGroundContactPoints.size(); i++)
      {
         if (groundContactPoint == allGroundContactPoints.get(i))
         {
            BooleanYoVariable contactAvailable = contactsAvailable.get(i);
            if (!contactAvailable.getBooleanValue())
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

   @Override
   public GroundContactPoint getLockedContactPoint(int contactPointIndex)
   {
      if (contactsAvailable.get(contactPointIndex).getBooleanValue())
      {
         throw new RuntimeException("Trying to get a contact point that isn't checked out!");
      }

      return allGroundContactPoints.get(contactPointIndex);
   }

   @Override
   public void updateContactPoints()
   {
      robot.update();
      robot.updateVelocities();
   }

}
