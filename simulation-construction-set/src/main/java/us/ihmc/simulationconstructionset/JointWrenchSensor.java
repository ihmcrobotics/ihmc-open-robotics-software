package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class JointWrenchSensor
{
   private final String name;
   private final YoFrameVector3D jointWrenchForce, jointWrenchTorque;
   private final Vector3D offsetFromJoint = new Vector3D();

   // FIXME offsetFromJoint is probably the offsetToJoint.
   public JointWrenchSensor(String name, Vector3D offsetFromJoint, Robot robot)
   {
      this.name = name;

      jointWrenchForce = new YoFrameVector3D(name + "_f", null, robot.getRobotsYoVariableRegistry());
      jointWrenchTorque = new YoFrameVector3D(name + "_t", null, robot.getRobotsYoVariableRegistry());

      this.offsetFromJoint.set(offsetFromJoint);
   }

   public void getOffsetFromJoint(Vector3D offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public void getJointForce(Tuple3DBasics forceToPack)
   {
      forceToPack.set(jointWrenchForce);
   }

   public void getJointTorque(Tuple3DBasics torqueToPack)
   {
      torqueToPack.set(jointWrenchTorque);
   }

   private final Vector3D tempVector = new Vector3D();

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
