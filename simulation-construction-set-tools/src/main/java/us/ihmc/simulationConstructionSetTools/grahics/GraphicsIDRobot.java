package us.ihmc.simulationConstructionSetTools.grahics;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;
import us.ihmc.simulationconstructionset.util.CommonJoint;

public class GraphicsIDRobot extends GraphicsRobot
{
   public GraphicsIDRobot(String name, RigidBodyBasics rootBody, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, rootBody, graphicsObjectsHolder, false);
   }

   public GraphicsIDRobot(String name, RigidBodyBasics rootBody, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      super(new Graphics3DNode(name, Graphics3DNodeType.TRANSFORM));

      for (JointBasics joint : rootBody.getChildrenJoints())
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT, graphicsObjectsHolder, useCollisionMeshes);
         getRootNode().addChild(rootGraphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), rootGraphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }

      update();
   }

   private void addInverseDynamicsJoints(List<? extends JointBasics> joints, GraphicsJoint parentJoint, GraphicsObjectsHolder graphicsObjectsHolder,
                                         boolean useCollisionMeshes)
   {
      for (JointBasics joint : joints)
      {
         GraphicsJoint graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
         parentJoint.addChild(graphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), graphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }
   }

   private GraphicsJoint createJoint(JointBasics inverseDynamicsJoint, Graphics3DNodeType nodeType, GraphicsObjectsHolder graphicsObjectsHolder,
                                     boolean useCollisionMeshes)
   {
      Graphics3DObject graphics3DObject;
      if (useCollisionMeshes)
      {
         graphics3DObject = generateGraphics3DObjectFromCollisionMeshes(graphicsObjectsHolder.getCollisionObjects(inverseDynamicsJoint.getName()));
      }
      else
      {
         graphics3DObject = graphicsObjectsHolder.getGraphicsObject(inverseDynamicsJoint.getName());
      }

      CommonJoint wrapJointBasics = wrapJointBasics(inverseDynamicsJoint);
      GraphicsJoint graphicsJoint = new GraphicsJoint(inverseDynamicsJoint.getName(), wrapJointBasics, graphics3DObject, nodeType);

      registerJoint(wrapJointBasics, graphicsJoint);
      return graphicsJoint;
   }

   private Graphics3DObject generateGraphics3DObjectFromCollisionMeshes(List<CollisionMeshDescription> collisionObjects)
   {
      System.err.println("Need to implement " + getClass().getSimpleName() + ".generateGraphics3DObjectFromCollisionMesh()!");
      return null;
   }

   private static CommonJoint wrapJointBasics(JointBasics jointToWrap)
   {
      return new CommonJoint()
      {
         @Override
         public RigidBodyTransform getOffsetTransform3D()
         {
            return new RigidBodyTransform(jointToWrap.getFrameBeforeJoint().getTransformToParent());
         }
         
         @Override
         public RigidBodyTransform getJointTransform3D()
         {
            return new RigidBodyTransform(jointToWrap.getFrameAfterJoint().getTransformToParent());
         }
      };
   }
}
