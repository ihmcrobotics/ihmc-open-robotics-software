package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private static int terminateToolboxCondition = 100;
   /*
    * YoVariables
    */
   private final YoInteger updateCount = new YoInteger("updateCount", registry);

   private final YoInteger expandingCount = new YoInteger("expandingCount", registry);

   // check the current pose is valid or not.   
   private final YoBoolean currentIsValid = new YoBoolean("currentIsValid", registry);

   // check the tree reaching the normalized time from 0.0 to 1.0.
   private final YoDouble currentTrajectoryTime = new YoDouble("currentNormalizedTime", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final YoDouble solutionQuality = new YoDouble("solutionQuality", registry);

   /*
    * Visualizer
    */
   private boolean startYoVariableServer;

   private CTTaskNode visualizedNode;

   private final FullHumanoidRobotModel visualizedFullRobotModel;

   private TreeStateVisualizer treeStateVisualizer;

   private CTTreeVisualizer treeVisualizer;

   /*
    * Configuration and Time space Tree
    */
   private CTTaskNode rootNode;

   private CTTaskNodeTree tree;

   /*
    * API
    */
   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);

   private int numberOfExpanding;

   /*
    * Toolbox state
    */
   private CWBToolboxState state;
   
   enum CWBToolboxState
   {
      DO_NOTHING,
      FIND_INITIAL_GUESS,
      EXPAND_TREE,
      SHORTCUT_PATH,
      GENERATE_MOTION
   }

   public ConstrainedWholeBodyPlanningToolboxController(FullHumanoidRobotModel fullRobotModel, StatusMessageOutputManager statusOutputManager,
                                                        YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry, boolean startYoVariableServer)
   {
      super(statusOutputManager, registry);
      this.visualizedFullRobotModel = fullRobotModel;
      this.isDone.set(false);

      this.startYoVariableServer = startYoVariableServer;
      this.treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsRegistry, registry);
      this.state = CWBToolboxState.DO_NOTHING;
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + updateCount.getIntegerValue() +" "+ state);

      // ************************************************************************************************************** //
      switch(state)
      {
      case DO_NOTHING:
         
         break;
      case FIND_INITIAL_GUESS:
         visualizedNode = new GenericTaskNode(0.02, 0.8, Math.PI*20/180, Math.PI*10/180, Math.PI*5/180);
         visualizedNode.convertDataToNormalizedData(CTTaskNode.constrainedEndEffectorTrajectory.getTaskNodeRegion());
         break;
      case EXPAND_TREE:
         
         break;
      case SHORTCUT_PATH:
         
         break;
      case GENERATE_MOTION:
         
         break;
      }
      // ************************************************************************************************************** //
      
      
      
      
      
      // ************************************************************************************************************** //
      
      /*
       * set fullRobotModel
       */
      FullHumanoidRobotModel solverRobotModel = CTTaskNode.nodeTester.getFullRobotModelCopy();
      visualizedFullRobotModel.getRootJoint().setPosition(solverRobotModel.getRootJoint().getTranslationForReading());
      visualizedFullRobotModel.getRootJoint().setRotation(solverRobotModel.getRootJoint().getRotationForReading());

      for (int i = 0; i < FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel).length; i++)
         FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel)[i].setQ(FullRobotModelUtils.getAllJointsExcludingHands(solverRobotModel)[i].getQ());
      
      /*
       * update visualizer
       */
      if(visualizedNode != null)
      {
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getNormalizedNodeData(0));
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.getIsValidNode());
         treeStateVisualizer.updateVisualizer();
         
         currentIsValid.set(visualizedNode.getIsValidNode());
         currentTrajectoryTime.set(visualizedNode.getNormalizedNodeData(0));
         if (startYoVariableServer)
            treeVisualizer.update(visualizedNode);   
      }

      // ************************************************************************************************************** //
      if(updateCount.getIntegerValue() == terminateToolboxCondition)
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
      
      PrintTools.info("initialize()");
      /*
       * bring control parameters from request
       */
      numberOfExpanding = request.numberOfExpanding;

      state = CWBToolboxState.FIND_INITIAL_GUESS;
      
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

   private ConstrainedWholeBodyPlanningToolboxOutputStatus packResult()
   {
      ConstrainedWholeBodyPlanningToolboxOutputStatus result = new ConstrainedWholeBodyPlanningToolboxOutputStatus();

      return result;
   }

   private void treeInitialize()
   {
      double initialPelvisHeight = CTTaskNode.initialRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();

      rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
      rootNode.setNodeData(2, -10.0 / 180 * Math.PI);
      rootNode.setNodeData(10, -30.0 / 180 * Math.PI);
      rootNode.setConfigurationJoints(GenericTaskNode.initialRobotModel);

      PrintTools.info("initial node is " + rootNode.isValidNode());

      tree = new CTTaskNodeTree(rootNode);

      rootNode.convertDataToNormalizedData(CTTaskNode.constrainedEndEffectorTrajectory.getTaskNodeRegion());

      if (startYoVariableServer)
      {
         treeVisualizer = new CTTreeVisualizer(tree);
         treeVisualizer.initialize();
      }
   }
}
