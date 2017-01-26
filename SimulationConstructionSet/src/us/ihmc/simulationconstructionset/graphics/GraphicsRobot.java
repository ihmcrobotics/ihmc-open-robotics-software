package us.ihmc.simulationconstructionset.graphics;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.robotics.kinematics.CommonJoint;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class GraphicsRobot implements GraphicsUpdatable
{
   private final Graphics3DNode rootNode;
   private Graphics3DNode cameraNode;

   private final LinkedHashMap<CommonJoint, GraphicsJoint> allJoints = new LinkedHashMap<CommonJoint, GraphicsJoint>();
   private final ArrayList<GraphicsJoint> graphicsJoints = new ArrayList<GraphicsJoint>();

   public GraphicsRobot(Robot robot)
   {
      rootNode = new Graphics3DNode(robot.getName(), Graphics3DNodeType.TRANSFORM);
      for (Joint joint : robot.getRootJoints())
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT);
         if(cameraNode == null)
         {
            cameraNode = rootGraphicsJoint;
         }
         rootNode.addChild(rootGraphicsJoint);
         addJoints(joint.getChildrenJoints(), rootGraphicsJoint);
      }

      update();
   }


   public GraphicsRobot(String name, RigidBody rootBody, GraphicsObjectsHolder graphicsObjectsHolder)
   {
      this(name, rootBody, graphicsObjectsHolder, false);
   }

   public GraphicsRobot(String name, RigidBody rootBody, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      rootNode = new Graphics3DNode(name, Graphics3DNodeType.TRANSFORM);
      for(InverseDynamicsJoint joint : rootBody.getChildrenJoints())
      {
         GraphicsJoint  rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT, graphicsObjectsHolder, useCollisionMeshes);
         if(cameraNode == null)
         {
            cameraNode = rootGraphicsJoint;
         }
         rootNode.addChild(rootGraphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), rootGraphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }

      update();
   }


   @Override
   public void update()
   {
      for (int i = 0; i < graphicsJoints.size(); i++)
      {
         graphicsJoints.get(i).updateFromJoint();
      }
   }

   public Graphics3DNode getRootNode()
   {
      return rootNode;
   }

   public Graphics3DNode getCameraNode()
   {
      return cameraNode;
   }

   private void addJoints(ArrayList<Joint> joints, GraphicsJoint parentJoint)
   {
      for (Joint joint : joints)
      {
         GraphicsJoint graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT);
         parentJoint.addChild(graphicsJoint);
         addJoints(joint.getChildrenJoints(), graphicsJoint);
      }
   }

   private GraphicsJoint createJoint(Joint joint, Graphics3DNodeType nodeType)
   {
      GraphicsJoint graphicsJoint = new GraphicsJoint(joint.getName(), joint, joint.getLink().getLinkGraphics(), nodeType);
      allJoints.put(joint, graphicsJoint);
      graphicsJoints.add(graphicsJoint);
      return graphicsJoint;
   }

   private GraphicsJoint createJoint(InverseDynamicsJoint inverseDynamicsJoint, Graphics3DNodeType nodeType, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      Graphics3DObject graphics3DObject;
      if(useCollisionMeshes)
      {
         graphics3DObject = generateGraphics3DObjectFromCollisionMesh(graphicsObjectsHolder.getCollisionObject(inverseDynamicsJoint.getName()));
      }
      else
      {
         graphics3DObject = graphicsObjectsHolder.getGraphicsObject(inverseDynamicsJoint.getName());
      }

      GraphicsJoint graphicsJoint = new GraphicsJoint(inverseDynamicsJoint.getName(), inverseDynamicsJoint, graphics3DObject, nodeType);


      allJoints.put(inverseDynamicsJoint, graphicsJoint);
      graphicsJoints.add(graphicsJoint);
      return graphicsJoint;
   }

   private Graphics3DObject generateGraphics3DObjectFromCollisionMesh(CollisionMeshDescription collisionObject)
   {
      System.err.println("Need to implement " + getClass().getSimpleName() + ".generateGraphics3DObjectFromCollisionMesh()!");
      return null;
   }


   private void addInverseDynamicsJoints(List<InverseDynamicsJoint> joints, GraphicsJoint parentJoint, GraphicsObjectsHolder graphicsObjectsHolder, boolean useCollisionMeshes)
   {
      for(InverseDynamicsJoint joint : joints)
      {
         GraphicsJoint graphicsJoint = createJoint(joint, Graphics3DNodeType.JOINT, graphicsObjectsHolder, useCollisionMeshes);
         parentJoint.addChild(graphicsJoint);
         addInverseDynamicsJoints(joint.getSuccessor().getChildrenJoints(), graphicsJoint, graphicsObjectsHolder, useCollisionMeshes);
      }
   }


   public GraphicsJoint getGraphicsJoint(CommonJoint joint)
   {
      return allJoints.get(joint);
   }

}
