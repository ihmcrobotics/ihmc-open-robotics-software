package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel fullRobotModel;

   private YoInteger updateCount = new YoInteger("UpdateCount", registry);
   
   private YoBoolean isValidNode = new YoBoolean("IsValidNode", registry);
   
   /*
    * a ball in visulaizer need to be represented to show that the node and pose are valid or not. The validity can be expressed with different color.
    */

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);

   private GenericTaskNode rootNode;
   
   private CTTaskNodeTree tree;
   
   private int numberOfExpanding;

   public ConstrainedWholeBodyPlanningToolboxController(FullHumanoidRobotModel fullRobotModel, StatusMessageOutputManager statusOutputManager,
                                                        YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.fullRobotModel = fullRobotModel;
      isDone.set(false);
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + updateCount.getIntegerValue());
     
//      tree.expandTree(100);

      if(tree.expandingTree())
      {         
         ArrayList<CTTaskNode> revertedPath = new ArrayList<CTTaskNode>();
         CTTaskNode currentNode = tree.getNewNode();
         revertedPath.add(currentNode);
         
         while (true)
         {
            currentNode = currentNode.getParentNode();
            if (currentNode != null)
            {
               revertedPath.add(currentNode);
            }
            else
               break;
         }

         int revertedPathSize = revertedPath.size();

         tree.getPath().clear();
         for (int j = 0; j < revertedPathSize; j++)
            tree.getPath().add(revertedPath.get(revertedPathSize - 1 - j));

         PrintTools.info("Constructed Tree size is " + revertedPathSize);

         /*
          * shortcut.
          */
//            optimalPath.clear();
//            for (int j = 0; j < path.size(); j++)
//            {
//               CTTaskNode node = path.get(i).createNodeCopy();
//               optimalPath.add(node);
//            }
         isDone.set(true);         
      }
      else
      {
         /*
          * set Yovariables
          */
         isValidNode.set(tree.getNewNode().getIsValidNode());
         /*
          * set fullRobotModel
          */
         FullHumanoidRobotModel solverRobotModel = GenericTaskNode.nodeTester.getFullRobotModelCopy();
         fullRobotModel.getRootJoint().setPosition(solverRobotModel.getRootJoint().getTranslationForReading());
         fullRobotModel.getRootJoint().setRotation(solverRobotModel.getRootJoint().getRotationForReading());
         
         for(int i=0;i<FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel).length;i++)
            FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel)[i].setQ(FullRobotModelUtils.getAllJointsExcludingHands(solverRobotModel)[i].getQ());
      }
      
      
      /*
       * After get path node, show serial posture for serial path node.
       */
      
      
      
      

      /*
       * terminate condition.
       */
      if (updateCount.getIntegerValue() == numberOfExpanding)
         isDone.set(true);
      updateCount.increment();
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      ConstrainedWholeBodyPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
      {
         return false;
      }

      /*
       * bring control parameters from request
       */
      PrintTools.info("initialize");
      
      numberOfExpanding = request.numberOfExpanding;
      
      /*
       * 
       */      

      /*
       * in near future, find initial posture algorithm is needed.
       */

      /*
       * root node and tree define
       */
      double initialPelvisHeight = GenericTaskNode.initialRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      
      rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
      rootNode.setNodeData(2, -10.0/180 * Math.PI);
      rootNode.setNodeData(10, -30.0/180 * Math.PI);
      rootNode.setConfigurationJoints(GenericTaskNode.initialRobotModel);      

      PrintTools.info("initial node is " + rootNode.isValidNode());

      tree = new CTTaskNodeTree(rootNode);
      tree.getTaskNodeRegion().setRandomRegion(0, 0.0, GenericTaskNode.constrainedEndEffectorTrajectory.getTrajectoryTime());
      tree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.90);
      tree.getTaskNodeRegion().setRandomRegion(2, -25.0 / 180 * Math.PI, 25.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(4, -0.0 / 180 * Math.PI, 0.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(5, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(6, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(7, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(8, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(9, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(10, -150.0/180*Math.PI, 0.0/180*Math.PI);

      rootNode.convertDataToNormalizedData(tree.getTaskNodeRegion());
      
      return true;
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   public PacketConsumer<ConstrainedWholeBodyPlanningRequestPacket> createRequestConsumer()
   {
      return new PacketConsumer<ConstrainedWholeBodyPlanningRequestPacket>()
      {
         @Override
         public void receivedPacket(ConstrainedWholeBodyPlanningRequestPacket packet)
         {
            if (packet == null)
               return;
            latestRequestReference.set(packet);
         }
      };
   }

}
