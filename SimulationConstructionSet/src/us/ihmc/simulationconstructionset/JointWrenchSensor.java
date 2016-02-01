package us.ihmc.simulationconstructionset;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class JointWrenchSensor
{
   private final String name;
   private final YoFrameVector jointWrenchForce, jointWrenchTorque;
   private final Vector3d offsetFromJoint = new Vector3d();

   // FIXME offsetFromJoint is probably the offsetToJoint.
   public JointWrenchSensor(String name, Vector3d offsetFromJoint, Robot robot)
   {
      this.name = name;

      jointWrenchForce = new YoFrameVector(name + "_f", null, robot.getRobotsYoVariableRegistry());
      jointWrenchTorque = new YoFrameVector(name + "_t", null, robot.getRobotsYoVariableRegistry());

      this.offsetFromJoint.set(offsetFromJoint);
   }

   public void getOffsetFromJoint(Vector3d offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public void getJointForce(Tuple3d forceToPack)
   {
      jointWrenchForce.get(forceToPack);
   }

   public void getJointTorque(Tuple3d torqueToPack)
   {
      jointWrenchTorque.get(torqueToPack);
   }

   private final Vector3d tempVector = new Vector3d();

   public void setWrench(SpatialVector wrenchToSet)
   {
      wrenchToSet.getTop(tempVector);
      jointWrenchForce.set(tempVector);

      wrenchToSet.getBottom(tempVector);
      jointWrenchTorque.set(tempVector);
   }

   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public String getName()
   {
      return name;
   }
}
