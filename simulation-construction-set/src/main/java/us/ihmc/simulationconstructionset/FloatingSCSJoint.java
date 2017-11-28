package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.kinematics.CommonJoint;

public interface FloatingSCSJoint extends CommonJoint
{
   public void setRotationAndTranslation(RigidBodyTransform transform);

   public void setVelocity(Tuple3DReadOnly velocity);

   public void setAngularVelocityInBody(Vector3DReadOnly angularVelocityInBody);

   public void getVelocity(FrameVector3D linearVelocityToPack);

   public void getAngularVelocity(FrameVector3D angularVelocityToPack, ReferenceFrame bodyFrame);

   public void getTransformToWorld(RigidBodyTransform ret);
}
