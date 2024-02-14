package us.ihmc.commonWalkingControlModules.parameterEstimation;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.yoVariables.euclid.YoMatrix3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoSpatialInertiaTemporary implements SpatialInertiaBasics, Settable<SpatialInertia>
{
   private final YoDouble mass;
   private final YoFrameVector3D centerOfMassOffset;
   private final YoMatrix3D momentOfInertia;
   private ReferenceFrame bodyFrame;
   private ReferenceFrame expressedInFrame;
   private final Point3D translation;

   public YoSpatialInertiaTemporary(ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry) {
      this("", bodyFrame, expressedInFrame, registry);
   }

   public YoSpatialInertiaTemporary(SpatialInertiaReadOnly input, String nameSuffix, YoRegistry registry) {
      this(nameSuffix, input.getBodyFrame(), input.getReferenceFrame(), registry);
      this.mass.set(input.getMass());
      this.centerOfMassOffset.set(input.getCenterOfMassOffset());
      this.momentOfInertia.set(input.getMomentOfInertia());
   }

   public YoSpatialInertiaTemporary(String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoRegistry registry) {
      this.translation = new Point3D();
      this.bodyFrame = bodyFrame;
      this.expressedInFrame = expressedInFrame;
      this.mass = new YoDouble(bodyFrame.getName() + "_mass" + nameSuffix, registry);
      this.centerOfMassOffset = new YoFrameVector3D(bodyFrame.getName() + "_centerOfMassOffset" + nameSuffix, expressedInFrame, registry);
      this.momentOfInertia = new YoMatrix3D(bodyFrame.getName() + "_momentOfInertia" + nameSuffix, registry);
   }

   public void applyTransform(Transform transform) {
      if (transform instanceof RigidBodyTransformReadOnly) {
         this.applyTransform((RigidBodyTransformReadOnly)transform);
      } else {
         this.translation.setToZero();
         this.translation.applyTransform(transform);
         this.momentOfInertia.applyTransform(transform);
         this.centerOfMassOffset.applyTransform(transform);
         MecanoTools.translateMomentOfInertia(this.mass.getDoubleValue(), this.centerOfMassOffset, false, this.translation, this.momentOfInertia);
         this.centerOfMassOffset.add(this.translation);
      }

   }

   public void applyInverseTransform(Transform transform) {
      if (transform instanceof RigidBodyTransformReadOnly) {
         this.applyInverseTransform((RigidBodyTransformReadOnly)transform);
      } else {
         this.translation.setToZero();
         this.translation.applyTransform(transform);
         MecanoTools.translateMomentOfInertia(this.mass.getDoubleValue(), this.centerOfMassOffset, true, this.translation, this.momentOfInertia);
         this.centerOfMassOffset.sub(this.translation);
         this.momentOfInertia.applyInverseTransform(transform);
         this.centerOfMassOffset.applyInverseTransform(transform);
      }

   }

   public ReferenceFrame getBodyFrame() {
      return this.bodyFrame;
   }

   public ReferenceFrame getReferenceFrame() {
      return this.expressedInFrame;
   }

   public double getMass() {
      return this.mass.getDoubleValue();
   }

   public FixedFrameVector3DBasics getCenterOfMassOffset() {
      return this.centerOfMassOffset;
   }

   public Matrix3DBasics getMomentOfInertia() {
      return this.momentOfInertia;
   }

   public void setBodyFrame(ReferenceFrame bodyFrame) {
      this.bodyFrame = bodyFrame;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame) {
      this.expressedInFrame = referenceFrame;
   }

   public void setMass(double mass) {
      this.mass.set(mass);
   }

   public void setCenterOfMassOffset(Tuple3DReadOnly offset) {
      this.centerOfMassOffset.set(offset);
   }

   public void setCenterOfMassOffset(double x, double y, double z) {
      this.centerOfMassOffset.set(x, y, z);
   }

   public void set(SpatialInertia other) {
      this.mass.set(other.getMass());
      this.centerOfMassOffset.set(other.getCenterOfMassOffset());
      this.momentOfInertia.set(other.getMomentOfInertia());
   }
}
