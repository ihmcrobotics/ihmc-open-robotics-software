package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation;

import javax.vecmath.Vector3d;

import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.ManipulableToroid;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Twist;


/**
 * @author twan
 *         Date: 5/13/13
 */
public class ToroidControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ManipulableToroid toroidUpdater;
   private final PDController toroidAngleController;
   private final Twist jointTwist = new Twist();
   private final Vector3d tempAngularPart = new Vector3d();
   private final Vector3d tempLinearPart = new Vector3d();

   public ToroidControlModule(ManipulableToroid toroidUpdater, YoVariableRegistry parentRegistry)
   {
      this.toroidUpdater = toroidUpdater;
      this.toroidAngleController = new PDController("toroidAngle", registry);
      parentRegistry.addChild(registry);
   }

   public void doControl(double qDesired, double qdDesired, double qddFeedForward)
   {
      OneDoFJoint joint = toroidUpdater.getToroidJoint();
      double q = joint.getQ();
      double qd = joint.getQd();
      double qddDesired = toroidAngleController.compute(q, qDesired, qd, qdDesired) + qddFeedForward;
      joint.setQddDesired(qddDesired);
   }

   public void get(SpatialAccelerationVector desiredSpatialAcceleration)
   {
      OneDoFJoint joint = toroidUpdater.getToroidJoint();
      joint.packDesiredJointAcceleration(desiredSpatialAcceleration);
   }

   public void get(SpatialForceVector feedForwardWrench)
   {
      OneDoFJoint joint = toroidUpdater.getToroidJoint();
      joint.packJointTwist(jointTwist);
      jointTwist.packAngularPart(tempAngularPart);
      jointTwist.packLinearPart(tempLinearPart);
      feedForwardWrench.set(jointTwist.getExpressedInFrame(), tempLinearPart, tempAngularPart);
      feedForwardWrench.scale(toroidUpdater.getDamping());
   }

   public void setProportionalGain(double proportionalGain)
   {
      toroidAngleController.setProportionalGain(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      toroidAngleController.setDerivativeGain(derivativeGain);
   }
}
