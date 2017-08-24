package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.kinematics.CommonJoint;

public interface FloatingSCSJoint extends CommonJoint
{
   public void setRotationAndTranslation(RigidBodyTransform transform);

   public void setVelocity(Tuple3DBasics velocity);

   public void setAngularVelocityInBody(Vector3D angularVelocityInBody);

   public void getVelocity(FrameVector3D linearVelocityToPack);

   public void getAngularVelocity(FrameVector3D angularVelocityToPack, ReferenceFrame bodyFrame);

   public void getTransformToWorld(RigidBodyTransform ret);
}
