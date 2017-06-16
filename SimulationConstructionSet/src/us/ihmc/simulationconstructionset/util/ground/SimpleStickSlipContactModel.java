package us.ihmc.simulationconstructionset.util.ground;


import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;

public class SimpleStickSlipContactModel
{
   private final ArrayList<ExternalForcePoint> contactPointAs = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactableBs = new ArrayList<Contactable>();

   private final ArrayList<YoInteger> contactPointAContactingContactableIndices = new ArrayList<YoInteger>();
   private final ArrayList<YoInteger> contactPointAContactingGroundContactIndices = new ArrayList<YoInteger>();

   private final Point3D contactATempPosition = new Point3D();


   private final YoDouble kContact, bContact;
   private final YoDouble alphaStick, alphaSlip;

   private final StickSlipContactCalculator stickSlipContactCalculator;

   private final YoVariableRegistry registry;

   public SimpleStickSlipContactModel(String namePrefix, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      kContact = new YoDouble(namePrefix + "KContact", registry);
      bContact = new YoDouble(namePrefix + "BContact", registry);

      alphaStick = new YoDouble(namePrefix + "AlphaStick", registry);
      alphaSlip = new YoDouble(namePrefix + "AlphaSlip", registry);

      stickSlipContactCalculator = new StickSlipContactCalculator();

      parentRegistry.addChild(registry);
   }

   public void setKContact(double kContact)
   {
      this.kContact.set(kContact);
   }

   public void setBContact(double bContact)
   {
      this.bContact.set(bContact);
   }

   public void setFrictionCoefficients(double alphaStick, double alphaSlip)
   {
      if (alphaSlip > alphaStick)
      {
         throw new RuntimeException("alphaSlip > alphaStick");
      }

      if ((alphaSlip < 0.0) || (alphaStick < 0.0))
      {
         throw new RuntimeException("alphaStick and alphaSlip must both be greater than 0.0");
      }

      this.alphaStick.set(alphaStick);
      this.alphaSlip.set(alphaSlip);
   }

   public void addContactPoint(ExternalForcePoint contactPoint)
   {
      this.contactPointAs.add(contactPoint);

      YoInteger contactableIndex = new YoInteger(contactPoint.getName() + "ContactableIndex", registry);
      contactPointAContactingContactableIndices.add(contactableIndex);
      contactableIndex.set(-1);

      YoInteger contactIndex = new YoInteger(contactPoint.getName() + "GroundContactIndex", registry);
      contactPointAContactingGroundContactIndices.add(contactIndex);
      contactIndex.set(-1);
   }

   public void addContactable(Contactable contactable)
   {
      this.contactableBs.add(contactable);
   }

   public void doContact()
   {
      // New contacts:

      for (int externalForcePointIndex = 0; externalForcePointIndex < contactPointAs.size(); externalForcePointIndex++)
      {
         ExternalForcePoint contactPointA = contactPointAs.get(externalForcePointIndex);
         if (contactPointAContactingContactableIndices.get(externalForcePointIndex).getIntegerValue() >= 0)
            continue;

         for (int contactableBIndex = 0; contactableBIndex < contactableBs.size(); contactableBIndex++)
         {
            Contactable contactableB = contactableBs.get(contactableBIndex);

            contactPointA.getPosition(contactATempPosition);
            boolean areInContact = contactableB.isPointOnOrInside(contactATempPosition);

            if (areInContact)
            {
               int contactPointBIndex = contactableB.getAndLockAvailableContactPoint();
               GroundContactPoint contactPointB = contactableB.getLockedContactPoint(contactPointBIndex);

               contactPointAContactingContactableIndices.get(externalForcePointIndex).set(contactableBIndex);
               contactPointAContactingGroundContactIndices.get(externalForcePointIndex).set(contactPointBIndex);

               stickSlipContactCalculator.doContactMade(contactPointA, contactableB, contactPointB);
            }
         }
      }

      // Existing contacts:
      for (int externalForcePointIndex = 0; externalForcePointIndex < contactPointAs.size(); externalForcePointIndex++)
      {
         ExternalForcePoint contactPointA = contactPointAs.get(externalForcePointIndex);
         int contactableBIndex = contactPointAContactingContactableIndices.get(externalForcePointIndex).getIntegerValue();
         int contactPointBIndex = contactPointAContactingGroundContactIndices.get(externalForcePointIndex).getIntegerValue();

         if (contactableBIndex < 0)
            continue;

         Contactable contactableB = contactableBs.get(contactableBIndex);
         GroundContactPoint contactPointB = contactableB.getLockedContactPoint(contactPointBIndex);

         contactPointA.getPosition(contactATempPosition);
         boolean areInContact = contactableB.isPointOnOrInside(contactATempPosition);

         if (areInContact)
         {
            stickSlipContactCalculator.doCurrentlyInContact(contactPointA, contactableB, contactPointB, kContact.getDoubleValue(), bContact.getDoubleValue(),
                    alphaStick.getDoubleValue(), alphaSlip.getDoubleValue());
         }

         else
         {
            stickSlipContactCalculator.doContactBroken(contactPointA, contactPointB);
            contactPointAContactingContactableIndices.get(externalForcePointIndex).set(-1);
            contactPointAContactingGroundContactIndices.get(externalForcePointIndex).set(-1);
            contactableB.unlockContactPoint(contactPointB);
         }
      }
   }
}
