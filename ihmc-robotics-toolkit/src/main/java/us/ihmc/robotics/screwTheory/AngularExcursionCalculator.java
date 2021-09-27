package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AngularExcursionCalculator
{
   private final WholeBodyAngularVelocityCalculator angularVelocityCalculator;
   private final YoFrameVector3D angularExcursion;
   private final YoFrameVector3D wholeBodyAngularVelocity;
   private final YoBoolean zeroAngularExcursionFlag;
   private final double dt;

   public AngularExcursionCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics rootBody, double dt, YoRegistry registry,
                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      this.dt = dt;

      angularVelocityCalculator = new WholeBodyAngularVelocityCalculator(centerOfMassFrame, graphicsListRegistry, rootBody.subtreeArray());

      zeroAngularExcursionFlag = new YoBoolean("zeroAngularExcursionFlag", registry);
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

      if (zeroAngularExcursionFlag.getBooleanValue())
      {
         setToZero();
         zeroAngularExcursionFlag.set(false);
      }

      wholeBodyAngularVelocity.set(angularVelocityCalculator.getWholeBodyAngularVelocity());
      angularExcursion.scaleAdd(dt, wholeBodyAngularVelocity, angularExcursion);
   }

   public FrameVector3DReadOnly getLinearMomentum()
   {
      return angularVelocityCalculator.getLinearMomentum();
   }

   public FrameVector3DReadOnly getAngularMomentum()
   {
      return angularVelocityCalculator.getAngularMomentum();
   }
}
