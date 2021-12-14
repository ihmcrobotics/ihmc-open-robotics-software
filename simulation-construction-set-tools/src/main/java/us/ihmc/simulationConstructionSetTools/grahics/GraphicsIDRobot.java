package us.ihmc.simulationConstructionSetTools.grahics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.screwTheory.InvertedFourBarJoint;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;
import us.ihmc.simulationconstructionset.util.CommonJoint;

public class GraphicsIDRobot extends GraphicsRobot
{
   private final Map<JointBasics, Graphics3DNode> jointToGraphicsNodeMap = new HashMap<>();

   public GraphicsIDRobot(String name, RigidBodyBasics rootBody, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, rootBody.getChildrenJoints(), graphicsObjectsHolder);
   }

   public GraphicsIDRobot(String name, List<JointBasics> jointsToVisualize, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, jointsToVisualize, graphicsObjectsHolder, null);
   }

   public GraphicsIDRobot(String name,
                          List<JointBasics> jointsToVisualize,
                          GraphicsObjectsHolder graphicsObjectsHolder,
                          List<RigidBodyReadOnly> terminalRigidBodies)
   {
      super(new Graphics3DNode(name, Graphics3DNodeType.TRANSFORM));

      for (JointBasics joint : jointsToVisualize)
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT, graphicsObjectsHolder);
         getRootNode().addChild(rootGraphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), rootGraphicsJoint, graphicsObjectsHolder, terminalRigidBodies);
      }

      update();
   }

   private void addInverseDynamicsJoints(List<? extends JointBasics> joints,
                                         GraphicsJoint parentJoint,
                                         GraphicsObjectsHolder graphicsObjectsHolder,
                                         List<RigidBodyReadOnly> terminalRigidBodies)
   {
      for (JointBasics joint : joints)
      {
         GraphicsJoint graphicsJoint;

         if (joint instanceof InvertedFourBarJoint)
         {
            InvertedFourBarJoint invertedFourBar = (InvertedFourBarJoint) joint;

            if (invertedFourBar.getJointA().isLoopClosure())
            {
               GraphicsJoint graphicsJointB = createJoint(invertedFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointC = createJoint(invertedFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointB.addChild(graphicsJointC);
               GraphicsJoint graphicsJointD = createJoint(invertedFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointC.addChild(graphicsJointD);
               graphicsJoint = graphicsJointC;
            }
            else if (invertedFourBar.getJointB().isLoopClosure())
            {
               GraphicsJoint graphicsJointA = createJoint(invertedFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointD = createJoint(invertedFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointA.addChild(graphicsJointD);
               GraphicsJoint graphicsJointC = createJoint(invertedFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointD.addChild(graphicsJointC);
               graphicsJoint = graphicsJointD;
            }
            else if (invertedFourBar.getJointC().isLoopClosure())
            {
               GraphicsJoint graphicsJointA = createJoint(invertedFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointB = createJoint(invertedFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointD = createJoint(invertedFourBar.getJointD(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointA.addChild(graphicsJointD);
               graphicsJoint = graphicsJointD;
            }
            else
            {
               GraphicsJoint graphicsJointA = createJoint(invertedFourBar.getJointA(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointA);
               GraphicsJoint graphicsJointB = createJoint(invertedFourBar.getJointB(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               parentJoint.addChild(graphicsJointB);
               GraphicsJoint graphicsJointC = createJoint(invertedFourBar.getJointC(), Graphics3DNodeType.JOINT, graphicsObjectsHolder);
               graphicsJointB.addChild(graphicsJointC);
               graphicsJoint = graphicsJointC;
            }
         }
         else
         {
            graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT, graphicsObjectsHolder);
            parentJoint.addChild(graphicsJoint);
         }

         if (terminalRigidBodies == null || !terminalRigidBodies.contains(joint.getSuccessor()))
         {
            addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), graphicsJoint, graphicsObjectsHolder, terminalRigidBodies);
         }
      }
   }

   private GraphicsJoint createJoint(JointBasics inverseDynamicsJoint, Graphics3DNodeType nodeType, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      Graphics3DObject graphics3DObject = graphicsObjectsHolder.getGraphicsObject(inverseDynamicsJoint.getName());

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
