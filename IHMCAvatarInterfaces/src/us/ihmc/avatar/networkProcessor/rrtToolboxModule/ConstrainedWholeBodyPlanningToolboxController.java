package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private int updateCount = 0;

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);
   
   private WheneverWholeBodyKinematicsSolver wbikTester;
   
   public ConstrainedWholeBodyPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      isDone.set(false);
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + updateCount);

      if (updateCount == 100)
         isDone.set(true);
      updateCount++;
      
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
      
//      GenericTaskNode.constrainedEndEffectorTrajectory = request.constrainedEndEffectorTrajectory;
//      
//      wbikTester = new WheneverWholeBodyKinematicsSolver(request.toolboxfullRobotModelFactory, request.toolboxFullRobotModel);
//
//      GenericTaskNode.nodeTester = wbikTester;
//      GenericTaskNode.midZUpFrame = request.referenceFrames.getMidFootZUpGroundFrame();
//      
//      double initialPelvisHeight = request.toolboxFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
//      GenericTaskNode rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
//      rootNode.setConfigurationJoints(request.toolboxFullRobotModel);
//      
//      CTTaskNodeTree tree = new CTTaskNodeTree(rootNode);
      
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
