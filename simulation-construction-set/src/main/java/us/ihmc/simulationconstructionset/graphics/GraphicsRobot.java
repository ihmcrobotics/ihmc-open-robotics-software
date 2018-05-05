package us.ihmc.simulationconstructionset.graphics;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.simulationConstructionSet.util.CommonJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.graphics.joints.GraphicsJoint;

public class GraphicsRobot implements GraphicsUpdatable
{
   private final Graphics3DNode rootNode;

   private final LinkedHashMap<CommonJoint, GraphicsJoint> allJoints = new LinkedHashMap<CommonJoint, GraphicsJoint>();
   private final ArrayList<GraphicsJoint> graphicsJoints = new ArrayList<GraphicsJoint>();

   public GraphicsRobot(Robot robot)
   {
      rootNode = new Graphics3DNode(robot.getName(), Graphics3DNodeType.TRANSFORM);
      for (Joint joint : robot.getRootJoints())
      {
         GraphicsJoint rootGraphicsJoint = createJoint(joint, Graphics3DNodeType.ROOTJOINT);
         rootNode.addChild(rootGraphicsJoint);
         addJoints(joint.getChildrenJoints(), rootGraphicsJoint);
      }

      update();
   }

   protected GraphicsRobot(Graphics3DNode rootNode)
   {
      this.rootNode = rootNode;
   }

   protected void registerJoint(CommonJoint joint, GraphicsJoint graphicsJoint)
   {
      allJoints.put(joint, graphicsJoint);
      graphicsJoints.add(graphicsJoint);
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

   public GraphicsJoint getGraphicsJoint(CommonJoint joint)
   {
      return allJoints.get(joint);
   }

}
