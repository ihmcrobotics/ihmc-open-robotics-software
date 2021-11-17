package us.ihmc.simulationConstructionSetTools.grahics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;
import us.ihmc.simulationconstructionset.util.CommonJoint;

public class GraphicsIDRobot extends GraphicsRobot
{
   private final Map<JointBasics, Graphics3DNode> jointToGraphicsNodeMap = new HashMap<>();

   public GraphicsIDRobot(String name, RigidBodyBasics rootBody, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, rootBody, graphicsObjectsHolder, false);
   }

   public GraphicsIDRobot(String name, RigidBodyBasics rootBody, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      this(name, rootBody.getChildrenJoints(), graphicsObjectsHolder, useCollisionMeshes);
   }

   public GraphicsIDRobot(String name, List<JointBasics> jointsToVisualize, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      this(name, jointsToVisualize, graphicsObjectsHolder, useCollisionMeshes, null);
   }

   public GraphicsIDRobot(String name,
                          List<JointBasics> jointsToVisualize,
                          GraphicsObjectsHolder graphicsObjectsHolder,
                          boolean useCollisionMeshes,
                          List<RigidBodyReadOnly> terminalRigidBodies)
   {
      super(new Graphics3DNode(name, Graphics3DNodeType.TRANSFORM));

      for (JointBasics joint : jointsToVisualize)
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT, graphicsObjectsHolder, useCollisionMeshes);
         getRootNode().addChild(rootGraphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), rootGraphicsJoint, graphicsObjectsHolder, useCollisionMeshes, terminalRigidBodies);
      }

      update();
   }

   private void addInverseDynamicsJoints(List<? extends JointBasics> joints,
                                         GraphicsJoint parentJoint,
                                         GraphicsObjectsHolder graphicsObjectsHolder,
                                         boolean useCollisionMeshes,
                                         List<RigidBodyReadOnly> terminalRigidBodies)
   {
      for (JointBasics joint : joints)
      {
         GraphicsJoint graphicsJoint;

         if (joint instanceof CrossFourBarJoint)
         {
            CrossFourBarJoint crossFourBar = (CrossFourBarJoint) joint;

            if (crossFourBar.getJointA().isLoopClosure())
            {
               GraphicsJoint graphicsJointB = createJoint(crossFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointC = createJoint(crossFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointB.addChild(graphicsJointC);
               GraphicsJoint graphicsJointD = createJoint(crossFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointC.addChild(graphicsJointD);
               GraphicsJoint graphicsJointA = createJoint(crossFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointD.addChild(graphicsJointA);
               graphicsJoint = graphicsJointC;
            }
            else if (crossFourBar.getJointB().isLoopClosure())
            {
               GraphicsJoint graphicsJointA = createJoint(crossFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointD = createJoint(crossFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointA.addChild(graphicsJointD);
               GraphicsJoint graphicsJointC = createJoint(crossFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointD.addChild(graphicsJointC);
               GraphicsJoint graphicsJointB = createJoint(crossFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointC.addChild(graphicsJointB);
               graphicsJoint = graphicsJointD;
            }
            else if (crossFourBar.getJointC().isLoopClosure())
            {
               GraphicsJoint graphicsJointA = createJoint(crossFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointB = createJoint(crossFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointD = createJoint(crossFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointA.addChild(graphicsJointD);
               GraphicsJoint graphicsJointC = createJoint(crossFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointD.addChild(graphicsJointC);
               graphicsJoint = graphicsJointD;
            }
            else
            {
               GraphicsJoint graphicsJointA = createJoint(crossFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointB = createJoint(crossFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointC = createJoint(crossFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointB.addChild(graphicsJointC);
               GraphicsJoint graphicsJointD = createJoint(crossFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
               graphicsJointC.addChild(graphicsJointD);
               graphicsJoint = graphicsJointC;
            }
         }
         else
         {
            graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
            parentJoint.addChild(graphicsJoint);
         }

         if (terminalRigidBodies == null || !terminalRigidBodies.contains(joint.getSuccessor()))
         {
            addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), graphicsJoint, graphicsObjectsHolder, useCollisionMeshes, terminalRigidBodies);
         }
      }
   }

   private GraphicsJoint createJoint(JointBasics inverseDynamicsJoint,
                                     Graphics3DNodeType nodeType,
                                     GraphicsObjectsHolder graphicsObjectsHolder,
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
      jointToGraphicsNodeMap.put(inverseDynamicsJoint, graphicsJoint);
      return graphicsJoint;
   }

   public Graphics3DNode getGraphicsNode(JointBasics joint)
   {
      return jointToGraphicsNodeMap.get(joint);
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
