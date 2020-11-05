package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactStateToForceMatrixHelper
{
   private final RigidBodyBasics rigidBody;
   private final int rhoSize;

   public ContactStateToForceMatrixHelper(RigidBodyBasics rigidBody, int maxNumberOfContactPoints,
                                          int numberOfBasisVectorsPerContactPoint, FrictionConeRotationCalculator coneRotationCalculator,
                                          YoRegistry parentRegistry)
   {
      this.rigidBody = rigidBody;

      rhoSize = maxNumberOfContactPoints * numberOfBasisVectorsPerContactPoint;
   }

   public int getRhoSize()
   {
      return rhoSize;
   }
}
