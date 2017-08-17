package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.TaskRegion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   public static double handCoordinateOffsetX = -0.2;

   /*
    * essential classes
    */
   private DRCRobotModel drcRobotModelFactory;

   public static ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   private WheneverWholeBodyKinematicsSolver kinematicsSolver;

   private static ReferenceFrame midZUpFrame;

   private static ReferenceFrame worldFrame;

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

   private final YoBoolean isGoodkinematicSolution = new YoBoolean("isGoodkinematicSolution", registry);

   private final YoDouble jointlimitScore = new YoDouble("jointlimitScore", registry);

   private double bestScoreInitialGuess = 0;

   private final YoInteger cntKinematicSolver = new YoInteger("cntKinematicSolver", registry);

   /*
    * Visualizer
    */
   private boolean startYoVariableServer;

   private CTTaskNode visualizedNode;

   private OneDoFJoint[] initialOneDoFJoints;

   private Vector3D initialTranslationOfRootJoint;
   private Quaternion initialRotationOfRootJoint;

   private FullHumanoidRobotModel visualizedFullRobotModel;

   private TreeStateVisualizer treeStateVisualizer;

   private CTTreeVisualizer treeVisualizer;

   private final YoFramePose endeffectorPose;

   private final YoGraphicCoordinateSystem endeffectorFrame;

   /*
    * Configuration and Time space Tree
    */
   private CTTaskNode rootNode;

   private CTTaskNodeTree tree;

   /*
    * API
    */
   private final AtomicReference<ConstrainedWholeBodyPlanningRequestPacket> latestRequestReference = new AtomicReference<ConstrainedWholeBodyPlanningRequestPacket>(null);

   private int numberOfExpanding = 100;

   private int numberOfInitialGuess = 30;

   private static int terminateToolboxCondition = 700;
   /*
    * Toolbox state
    */
   private CWBToolboxState state;

   enum CWBToolboxState
   {
      DO_NOTHING, FIND_INITIAL_GUESS, EXPAND_TREE, SHORTCUT_PATH, GENERATE_MOTION
   }

   public ConstrainedWholeBodyPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel,
                                                        StatusMessageOutputManager statusOutputManager, YoVariableRegistry registry,
                                                        YoGraphicsListRegistry yoGraphicsRegistry, boolean startYoVariableServer)
   {
      super(statusOutputManager, registry);
      this.drcRobotModelFactory = drcRobotModel;
      this.visualizedFullRobotModel = fullRobotModel;
      this.isDone.set(false);

      this.startYoVariableServer = startYoVariableServer;
      this.treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsRegistry, registry);
      this.state = CWBToolboxState.DO_NOTHING;

      this.endeffectorPose = new YoFramePose("endeffectorPose", ReferenceFrame.getWorldFrame(), registry);
      this.endeffectorFrame = new YoGraphicCoordinateSystem("endeffectorPoseFrame", this.endeffectorPose, 0.15);
      this.endeffectorFrame.setVisible(true);

      yoGraphicsRegistry.registerYoGraphic("endeffectorPoseViz", this.endeffectorFrame);

      state = CWBToolboxState.FIND_INITIAL_GUESS;
   }

   @Override
   protected void updateInternal()
   {
      PrintTools.info("update toolbox " + updateCount.getIntegerValue() + " " + state);

      // ************************************************************************************************************** //
      TaskRegion taskRegion = constrainedEndEffectorTrajectory.getTaskRegion();
      switch (state)
      {
      case DO_NOTHING:

         break;
      case FIND_INITIAL_GUESS:

         GenericTaskNode initialGuessNode = new GenericTaskNode();

         tree.setRandomNormalizedNodeData(initialGuessNode, true);
         initialGuessNode.setNormalizedNodeData(0, 0);
         initialGuessNode.convertNormalizedDataToData(taskRegion);

         visualizedNode = initialGuessNode;

         isValidNode(visualizedNode);
         double scoreInitialGuess = kinematicsSolver.getArmJointLimitScore(constrainedEndEffectorTrajectory.getRobotSide());
         if (!visualizedNode.getIsValidNode())
            scoreInitialGuess = 0.0;

         jointlimitScore.set(scoreInitialGuess);

         if (bestScoreInitialGuess < scoreInitialGuess)
         {
            bestScoreInitialGuess = scoreInitialGuess;

            rootNode = visualizedNode.createNodeCopy();
            rootNode = new GenericTaskNode(visualizedNode);
         }

         /*
          * terminate finding initial guess.
          */
         numberOfInitialGuess--;
         if (numberOfInitialGuess == 0)
         {
            PrintTools.info("initial guess terminate");
            state = CWBToolboxState.EXPAND_TREE;

            if (true)
            {
               rootNode.convertDataToNormalizedData(taskRegion);

               tree = new CTTaskNodeTree(rootNode);
               tree.setTaskRegion(taskRegion);
            }
         }

         break;
      case EXPAND_TREE:

         tree.updateRandomConfiguration();
         tree.updateNearestNode();
         tree.updateNewConfiguration();
         tree.getNewNode().convertNormalizedDataToData(taskRegion);
         tree.getNewNode().setParentNode(tree.getNearNode());

         if (isValidNode(tree.getNewNode()))
         {
            tree.connectNewNode(true);
            if (tree.getNewNode().getTime() == constrainedEndEffectorTrajectory.getTrajectoryTime())
            {
               PrintTools.info("terminate expanding");
               numberOfExpanding = 1; // for terminate
            }
         }
         else
         {
            tree.connectNewNode(false);
         }

         visualizedNode = tree.getNewNode().createNodeCopy();
         visualizedNode.setIsValidNode(tree.getNewNode().getIsValidNode());

         /*
          * terminate expanding tree.
          */
         numberOfExpanding--;
         if (numberOfExpanding == 0)
         {
            state = CWBToolboxState.SHORTCUT_PATH;

         }

         break;
      case SHORTCUT_PATH:

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

         /*
          * terminate toolbox
          */
         isDone.set(true);
         state = CWBToolboxState.SHORTCUT_PATH;

         PrintTools.info("the size of the path is " + tree.getPath().size());

         break;
      case GENERATE_MOTION:

         break;
      }
      // ************************************************************************************************************** //

      // ************************************************************************************************************** //

      updateVisualizerRobotConfiguration(kinematicsSolver.getFullRobotModelCopy());
      updateVisualizers();
      updateYoVariables();

      // ************************************************************************************************************** //
      updateCount.increment();
      if (updateCount.getIntegerValue() == terminateToolboxCondition)
         isDone.set(true);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      ConstrainedWholeBodyPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      PrintTools.info("initialize CWB toolbox");

      /*
       * bring control parameters from request.
       */
      numberOfExpanding = request.numberOfExpanding;
      numberOfInitialGuess = request.numberOfFindInitialGuess;

      initialOneDoFJoints = request.initialOneDoFJoints;
      initialTranslationOfRootJoint = request.initialTranslationOfRootJoint;
      initialRotationOfRootJoint = request.initialRotationOfRootJoint;

      /*
       * initialize kinematicsSolver.
       */
      kinematicsSolver = new WheneverWholeBodyKinematicsSolver(drcRobotModelFactory);

      kinematicsSolver.updateRobotConfigurationData(initialOneDoFJoints, initialTranslationOfRootJoint, initialRotationOfRootJoint);

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();
      kinematicsSolver.putTrajectoryMessages();

      /*
       * start toolbox
       */
      rootNode = new GenericTaskNode();
      tree = new CTTaskNodeTree(rootNode);
      tree.setTaskRegion(constrainedEndEffectorTrajectory.getTaskRegion());

      rootNode.convertDataToNormalizedData(constrainedEndEffectorTrajectory.getTaskRegion());

      if (startYoVariableServer)
      {
         treeVisualizer = new CTTreeVisualizer(tree);
         treeVisualizer.initialize();
      }

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

   private boolean isValidNode(CTTaskNode node)
   {
      kinematicsSolver = new WheneverWholeBodyKinematicsSolver(drcRobotModelFactory);

      if (node.getParentNode() != null)
      {
         PrintTools.warn("this node has parent node.");
         kinematicsSolver.updateRobotConfigurationData(node.getParentNode().getOneDoFJoints(), node.getParentNode().getRootTranslation(),
                                                       node.getParentNode().getRootRotation());
      }
      else
      {
         PrintTools.warn("parentNode is required.");
         kinematicsSolver.updateRobotConfigurationData(initialOneDoFJoints, initialTranslationOfRootJoint, initialRotationOfRootJoint);
      }

      kinematicsSolver.initialize();

      kinematicsSolver.holdCurrentTrajectoryMessages();
      
      /*
       * set whole body tasks.
       */
      ConfigurationSpace configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(node.getNodeData(5), node.getNodeData(6), node.getNodeData(7));
      configurationSpace.setRotation(node.getNodeData(8), node.getNodeData(9), node.getNodeData(10));

      /*
       * pose from 'constrainedEndEffectorTrajectory' is considered as in MidZUp
       * frame.
       */
      Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(node.getNodeData(0), configurationSpace);
      setEndEffectorPose(desiredPose);
      
      /*
       * for kinematics solver, append offset
       */
      desiredPose.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);

      kinematicsSolver.setDesiredHandPose(constrainedEndEffectorTrajectory.getRobotSide(), desiredPose);

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(node.getNodeData(2));

      desiredChestOrientation.appendPitchRotation(node.getNodeData(3));
      desiredChestOrientation.appendRollRotation(node.getNodeData(4));
      kinematicsSolver.setDesiredChestOrientation(desiredChestOrientation);

      kinematicsSolver.setDesiredPelvisHeight(node.getNodeData(1));

      kinematicsSolver.putTrajectoryMessages();

      /*
       * result
       */
      boolean ikResult = kinematicsSolver.isSolved();
      boolean colResult = isCollisionFree();
      boolean result = false;

      if (ikResult && colResult)
         result = true;

      node.setConfigurationJoints(kinematicsSolver.getFullRobotModelCopy());
      //      node.setConfigurationJoints(kinematicsSolver.getDesiredFullRobotModel());

      node.setIsValidNode(result);

      cntKinematicSolver.set(kinematicsSolver.getCntForUpdateInternal());

      return result;
   }

   private boolean isCollisionFree()
   {
      RobotCollisionModel robotCollisionModel = new RobotCollisionModel(kinematicsSolver.getDesiredFullRobotModel());

      robotCollisionModel.update();
      boolean isCollisionFree = robotCollisionModel.getCollisionResult();

      return isCollisionFree;
   }

   /**
    * set fullRobotModel.
    */
   private void updateVisualizerRobotConfiguration(FullHumanoidRobotModel solverRobotModel)
   {
      visualizedFullRobotModel.getRootJoint().setPosition(solverRobotModel.getRootJoint().getTranslationForReading());
      visualizedFullRobotModel.getRootJoint().setRotation(solverRobotModel.getRootJoint().getRotationForReading());

      for (int i = 0; i < FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel).length; i++)
         FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel)[i].setQ(FullRobotModelUtils.getAllJointsExcludingHands(solverRobotModel)[i].getQ());
   }

   /**
    * update visualizers.
    */
   private void updateVisualizers()
   {
      if (visualizedNode != null)
      {
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getNormalizedNodeData(0));
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.getIsValidNode());
         treeStateVisualizer.updateVisualizer();

         currentIsValid.set(visualizedNode.getIsValidNode());
         currentTrajectoryTime.set(visualizedNode.getNormalizedNodeData(0));
         if (startYoVariableServer)
            treeVisualizer.update(visualizedNode);
      }
   }

   /**
    * YoVariables.
    */
   private void updateYoVariables()
   {
      isGoodkinematicSolution.set(kinematicsSolver.getIsSolved());
      solutionQuality.set(kinematicsSolver.getSolution().getSolutionQuality());
      endeffectorFrame.setVisible(true);
      endeffectorFrame.update();
   }
   
   /**
    * update end effector pose
    */
   private void setEndEffectorPose(Pose3D desiredPose)
   {
      endeffectorPose.setPosition(desiredPose.getPosition());
      endeffectorPose.setOrientation(desiredPose.getOrientation());
   }
}
