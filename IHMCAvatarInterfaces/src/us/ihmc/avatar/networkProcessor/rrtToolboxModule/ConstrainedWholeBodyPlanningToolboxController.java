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
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConstrainedWholeBodyPlanningToolboxHelper;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel fullRobotModel;

   private final YoInteger expandingCount = new YoInteger("ExpandingCount", registry);

   private final YoDouble solutionQuality = new YoDouble("sqNewNode", registry);
   private final YoDouble scoreNewNode = new YoDouble("scoreNewNode", registry);
   private final YoBoolean isValidNewNode = new YoBoolean("isValidNewNode", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);

   private double bestInitialGuessScore;
   
   private CTTaskNode visualizedNode;
   private CTTaskNode initialGuess;
   private CTTaskNode rootNode;

   private CTTaskNodeTree tree;

   private int numberOfExpanding;

   private static int numberOfRandomTryForInitialGuess = 20;

   private static int currentTryForInitialGuess = 0;

   private boolean startYoVariableServer;
   
   
   /*
    * variables for output status
    */
   
   private int toolboxSolutionState = 0;  // 0: not initiated, 1: cannot find initial guess, 2: cannot find path, 3: ?

   /*
    * visualizer
    */
   private TreeStateVisualizer treeStateVisualizer;
   private CTTreeVisualizer treeVisualizer;

   private CWBToolboxState currentCWBToolboxState;

   enum CWBToolboxState
   {
      DO_NOTHING, FIND_INITIAL_GUESS, EXPANDING_TREE, SHORTCUT_PATH
   };

   public ConstrainedWholeBodyPlanningToolboxController(FullHumanoidRobotModel fullRobotModel, StatusMessageOutputManager statusOutputManager,
                                                        YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry, boolean startYoVariableServer)
   {
      super(statusOutputManager, registry);
      this.fullRobotModel = fullRobotModel;
      this.isDone.set(false);

      this.startYoVariableServer = startYoVariableServer;
      this.treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsRegistry, registry);
      this.currentCWBToolboxState = CWBToolboxState.DO_NOTHING;
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("updateInternal");
      
      switch (currentCWBToolboxState)
      {
      case DO_NOTHING:
         toolboxSolutionState = 0;
         
         break;

      case FIND_INITIAL_GUESS:
         ConstrainedWholeBodyPlanningToolboxHelper.setMaximumUpdateOfTester(200);

         tree.updateRandomInitialGuess();
         
         PrintTools.info("update new node with random node data done");

         scoreNewNode.set(GenericTaskNode.nodeTester.getScore(RobotSide.LEFT));
         solutionQuality.set(GenericTaskNode.nodeTester.getSolutionQuality());

         if (bestInitialGuessScore > scoreNewNode.getDoubleValue())
         {
            bestInitialGuessScore = scoreNewNode.getDoubleValue();
            initialGuess = tree.getNewNode().createNodeCopy();
         }

         currentTryForInitialGuess++;
         if (currentTryForInitialGuess == numberOfRandomTryForInitialGuess)
         {
            if(initialGuess == null)
            {
               isDone.set(true);
               toolboxSolutionState = 1;
            }
            else
            {
               isDone.set(true);
//               currentCWBToolboxState = CWBToolboxState.EXPANDING_TREE;
//               rootNode = initialGuess.createNodeCopy();
//               treeInitialize();   
            }
         }
         
         visualizedNode = tree.getNewNode().createNodeCopy();
         break;

      case EXPANDING_TREE:
//         ConstrainedWholeBodyPlanningToolboxHelper.setMaximumUpdateOfTester(100);
//
//         PrintTools.info("expandingCount " + expandingCount.getIntegerValue());
//         if (tree.expandingTree())
//         {
//            currentCWBToolboxState = CWBToolboxState.SHORTCUT_PATH;
//
//         }
//
//         
//         visualizedNode = tree.getNewNode().createNodeCopy();
//         
//         /*
//          * terminate condition.
//          */
//         if (expandingCount.getIntegerValue() == numberOfExpanding)
//            isDone.set(true);
//         expandingCount.increment();

         break;

      case SHORTCUT_PATH:

         //       ArrayList<CTTaskNode> revertedPath = new ArrayList<CTTaskNode>();
         //       CTTaskNode currentNode = tree.getNewNode();
         //       revertedPath.add(currentNode);
         //
         //       while (true)
         //       {
         //          currentNode = currentNode.getParentNode();
         //          if (currentNode != null)
         //          {
         //             revertedPath.add(currentNode);
         //          }
         //          else
         //             break;
         //       }
         //       int revertedPathSize = revertedPath.size();
         //
         //       tree.getPath().clear();
         //       for (int j = 0; j < revertedPathSize; j++)
         //          tree.getPath().add(revertedPath.get(revertedPathSize - 1 - j));
         //
         //       PrintTools.info("Constructed Tree size is " + revertedPathSize);
         //
         //       /*
         //        * shortcut.
         //        */
         //       optimalPath.clear();
         //       for (int j = 0; j < path.size(); j++)
         //       {
         //          CTTaskNode node = path.get(i).createNodeCopy();
         //          optimalPath.add(node);
         //       }

         isDone.set(true);

         break;
      }

      /*
       * update visualizer
       */
      if (visualizedNode != null)
      {
         isValidNewNode.set(visualizedNode.getIsValidNode());
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getNormalizedNodeData(0));
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.getIsValidNode());
         treeStateVisualizer.updateVisualizer();
      
         if (startYoVariableServer)
            treeVisualizer.update(visualizedNode);
      }
      
      FullHumanoidRobotModel solverRobotModel = GenericTaskNode.nodeTester.getFullRobotModelCopy();
      
      PrintTools.info("solverRobotModel");
      System.out.println(solverRobotModel.getRootJoint().getTranslationForReading());
      
      fullRobotModel.getRootJoint().setPosition(solverRobotModel.getRootJoint().getTranslationForReading());
      fullRobotModel.getRootJoint().setRotation(solverRobotModel.getRootJoint().getRotationForReading());

      for (int i = 0; i < FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel).length; i++)
         FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel)[i].setQ(FullRobotModelUtils.getAllJointsExcludingHands(solverRobotModel)[i].getQ());

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
       * root node and tree define
       */
      double initialPelvisHeight = GenericTaskNode.initialRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();

      rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
      rootNode.setConfigurationJoints(GenericTaskNode.initialRobotModel);
      rootNode.convertDataToNormalizedData(GenericTaskNode.constrainedEndEffectorTrajectory.getTaskNodeRegion());
      
      treeInitialize();

      bestInitialGuessScore = Double.MAX_VALUE;
      currentCWBToolboxState = CWBToolboxState.FIND_INITIAL_GUESS;
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
      /*
       * tree initiate
       */
      tree = new CTTaskNodeTree(rootNode);

      if (startYoVariableServer)
      {
         treeVisualizer = new CTTreeVisualizer(tree);
         treeVisualizer.initialize();
      }
   }

   private void tryRandomInitialGuess()
   {

   }
}
