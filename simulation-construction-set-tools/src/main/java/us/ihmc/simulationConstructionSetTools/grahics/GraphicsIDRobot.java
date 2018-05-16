package us.ihmc.simulationConstructionSetTools.grahics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;

public class GraphicsIDRobot extends GraphicsRobot
{
   public GraphicsIDRobot(String name, RigidBody rootBody, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, rootBody, graphicsObjectsHolder, false);
   }

   public GraphicsIDRobot(String name, RigidBody rootBody, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      super(new Graphics3DNode(name, Graphics3DNodeType.TRANSFORM));

      for (InverseDynamicsJoint joint : rootBody.getChildrenJoints())
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT, graphicsObjectsHolder, useCollisionMeshes);
         getRootNode().addChild(rootGraphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), rootGraphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }

      update();
   }

   private void addInverseDynamicsJoints(List<InverseDynamicsJoint> joints, GraphicsJoint parentJoint, GraphicsObjectsHolder graphicsObjectsHolder,
                                         boolean useCollisionMeshes)
   {
      for (InverseDynamicsJoint joint : joints)
      {
         GraphicsJoint graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
         parentJoint.addChild(graphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), graphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }
   }

   private GraphicsJoint createJoint(InverseDynamicsJoint inverseDynamicsJoint, Graphics3DNodeType nodeType, GraphicsObjectsHolder graphicsObjectsHolder,
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

      GraphicsJoint graphicsJoint = new GraphicsJoint(inverseDynamicsJoint.getName(), inverseDynamicsJoint, graphics3DObject, nodeType);

      registerJoint(inverseDynamicsJoint, graphicsJoint);
      return graphicsJoint;
   }

   private Graphics3DObject generateGraphics3DObjectFromCollisionMeshes(ArrayList<CollisionMeshDescription> collisionObjects)
   {
      System.err.println("Need to implement " + getClass().getSimpleName() + ".generateGraphics3DObjectFromCollisionMesh()!");
      return null;
   }
}
