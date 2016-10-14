package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

public interface FloatingSCSJoint extends CommonJoint
{
   public void setRotationAndTranslation(RigidBodyTransform transform);

   public void setVelocity(Tuple3d velocity);

   public void setAngularVelocityInBody(Vector3d angularVelocityInBody);

   public void getVelocity(FrameVector linearVelocityToPack);

   public void getAngularVelocity(FrameVector angularVelocityToPack, ReferenceFrame bodyFrame);

   public void getTransformToWorld(RigidBodyTransform ret);
}
