package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularExcursionCalculator
{
   private final WholeBodyAngularVelocityCalculator angularVelocityCalculator;
   private final YoFrameVector3D angularExcursion;
   private final YoFrameVector3D wholeBodyAngularVelocity;
   private final double dt;

   public AngularExcursionCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics rootBody, double dt, YoRegistry registry)
   {
      this.dt = dt;

      angularVelocityCalculator = new WholeBodyAngularVelocityCalculator(centerOfMassFrame, rootBody.subtreeArray());

      angularExcursion = new YoFrameVector3D("angularExcursion", centerOfMassFrame, registry);
      wholeBodyAngularVelocity = new YoFrameVector3D("wholeBodyAngularVelocity", centerOfMassFrame, registry);
   }

   public void setToZero()
   {
      angularExcursion.setToZero();
   }

   public void setAngularExcursionValue(Vector3D value)
   {
      angularExcursion.set(value);
   }

   public void compute()
   {
      angularVelocityCalculator.compute();

      wholeBodyAngularVelocity.set(angularVelocityCalculator.getWholeBodyAngularVelocity());
      angularExcursion.scaleAdd(dt, wholeBodyAngularVelocity, angularExcursion);
   }

   public FrameVector3DReadOnly getAngularMomentum()
   {
      return angularVelocityCalculator.getAngularMomentum();
   }
}
