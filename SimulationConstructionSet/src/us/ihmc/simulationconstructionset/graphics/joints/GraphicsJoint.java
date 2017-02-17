package us.ihmc.simulationconstructionset.graphics.joints;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.robotics.kinematics.CommonJoint;

public class GraphicsJoint extends Graphics3DNode
{
   private final CommonJoint joint;
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

   public GraphicsJoint(String name, CommonJoint joint, Graphics3DObject graphics3DObject, Graphics3DNodeType nodeType)
   {
      super(name, nodeType);
      this.joint = joint;

      setGraphicsObject(graphics3DObject);
     
   }

   public final void updateFromJoint()
   {
      transformToParent.setIdentity();
      transformToParent.multiply(joint.getOffsetTransform3D());
      transformToParent.multiply(joint.getJointTransform3D());
      setTransform(transformToParent);
   }
}
