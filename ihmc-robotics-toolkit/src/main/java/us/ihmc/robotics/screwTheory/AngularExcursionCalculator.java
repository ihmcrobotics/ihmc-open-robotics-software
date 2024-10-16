package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AngularExcursionCalculator
{
   private final WholeBodyAngularVelocityCalculator angularVelocityCalculator;
   private final Vector3D axisAngle = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private final YoFrameQuaternion angularExcursion;
   private final YoFrameYawPitchRoll angularExcursionRPY;
   private final YoFrameVector3D wholeBodyAngularVelocity;
   private final YoFrameVector3D wholeBodyAngularMomentum;
   private final YoBoolean zeroAngularExcursionFlag;
   private final double dt;

   private final Vector3D tempVector = new Vector3D();
   private final YoFramePose3D comPose;
   private final ReferenceFrame centerOfMassFrame;

   public AngularExcursionCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics rootBody, double dt, YoRegistry registry,
                                     YoGraphicsListRegistry graphicsListRegistry)
   {
      this.dt = dt;
      this.centerOfMassFrame = centerOfMassFrame;

      angularVelocityCalculator = new WholeBodyAngularVelocityCalculator(centerOfMassFrame, graphicsListRegistry, rootBody.subtreeArray());

      zeroAngularExcursionFlag = new YoBoolean("zeroAngularExcursionFlag", registry);
      angularExcursion = new YoFrameQuaternion("angularExcursion", centerOfMassFrame, registry);
      angularExcursionRPY = new YoFrameYawPitchRoll("angularExcursion", centerOfMassFrame, registry);
      wholeBodyAngularVelocity = new YoFrameVector3D("wholeBodyAngularVelocity", centerOfMassFrame, registry);
      wholeBodyAngularMomentum = new YoFrameVector3D("wholeBodyAngularMomentum", centerOfMassFrame, registry);

      comPose = new YoFramePose3D("comPose", ReferenceFrame.getWorldFrame(), registry);
      if (graphicsListRegistry != null)
      {
         YoGraphicCoordinateSystem comCoordinate = new YoGraphicCoordinateSystem("comCoordinate", comPose, 1.0);
         graphicsListRegistry.registerYoGraphic("comFrame", comCoordinate);
      }
   }

   public void setToZero()
   {
      angularExcursion.setToZero();
   }

//   public void setAngularExcursionValue(Vector3D value)
//   {
//      angularExcursion.set(value);
//   }

   public void compute()
   {
      comPose.setFromReferenceFrame(centerOfMassFrame);
      angularVelocityCalculator.compute();

      if (zeroAngularExcursionFlag.getBooleanValue())
      {
         setToZero();
         zeroAngularExcursionFlag.set(false);
      }

      wholeBodyAngularVelocity.set(angularVelocityCalculator.getWholeBodyAngularVelocity());
      wholeBodyAngularMomentum.set(angularVelocityCalculator.getAngularMomentum());


      axisAngle.setAndScale(dt, wholeBodyAngularVelocity);
      angularExcursion.inverseTransform(axisAngle);
      rotation.setRotationVector(axisAngle);


      angularExcursion.append(rotation);
      angularExcursionRPY.set(angularExcursion);
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
