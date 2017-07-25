package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private int updateCount = 0;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);

   private GenericTaskNode rootNode;
   private CTTaskNodeTree tree;

   public ConstrainedWholeBodyPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      isDone.set(false);
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + updateCount);
      tree.expandTree(100);
      isDone.set(true);
      //      if (updateCount == 100)
      //         isDone.set(true);
      //      updateCount++;

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

      PrintTools.info("initialize");
      PrintTools.info("temp input " + request.tempInputValue);

      double initialPelvisHeight = GenericTaskNode.initialRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();

      /*
       * in near future, find initial posture algorith is needed.
       */

      /*
       * root node and tree define
       */
      rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
      rootNode.setConfigurationJoints(GenericTaskNode.initialRobotModel);

      PrintTools.info("initial node is " + rootNode.isValidNode());

      tree = new CTTaskNodeTree(rootNode);
      tree.getTaskNodeRegion().setRandomRegion(0, 0.0, GenericTaskNode.constrainedEndEffectorTrajectory.getTrajectoryTime());
      tree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.90);
      tree.getTaskNodeRegion().setRandomRegion(2, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(4, -0.0 / 180 * Math.PI, 0.0 / 180 * Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(5, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(6, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(7, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(8, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(9, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(10, 0.0, 0.0);

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
