package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeFindInitialGuess;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationBuildOrder;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationSpace;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConstrainedEndEffectorTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TaskRegion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyTrajectoryToolboxController extends ToolboxController
{
   private static final int DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE = 1000;
   private static final int DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE = 50;

   public static double handCoordinateOffsetX = -0.05;//-0.2;

   private static double handOffset_NoHand_Version = -0.03;
   private static double handOffset_DualRobotiQ_Version = -0.2;

   /*
    * essential classes
    */
   private DRCRobotModel drcRobotModelFactory;

   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   private final WheneverWholeBodyKinematicsSolver kinematicsSolver;

   private final WholeBodyTrajectoryToolboxOutputStatus toolboxSolution;

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

   private final YoDouble jointlimitScore = new YoDouble("jointlimitScore", registry);

   private double bestScoreInitialGuess = 0.0;

   private final YoInteger cntKinematicSolver = new YoInteger("cntKinematicSolver", registry);

   /*
    * Visualizer
    */
   private boolean startYoVariableServer;

   private CTTaskNode visualizedNode;

   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();
   private final AtomicReference<RobotConfigurationData> currentRobotConfigurationDataReference = new AtomicReference<>(null);

   private FullHumanoidRobotModel visualizedFullRobotModel;

   private TreeStateVisualizer treeStateVisualizer;

   private CTTreeVisualizer treeVisualizer;

   private final SideDependentList<YoFramePose> endeffectorPose = new SideDependentList<>();

   private final SideDependentList<YoGraphicCoordinateSystem> endeffectorFrame = new SideDependentList<>();

   /*
    * Configuration and Time space Tree
    */
   private CTTaskNode rootNode;

   private CTTaskNodeTree tree;

   private TaskRegion taskRegion;

   /*
    * API
    */

   private int maximumExpansionSize = 1;

   private int numberOfInitialGuesses = 1;

   private int numberOfMotionPath = 1;

   private int numberOfTimesToShortcut = 5;

   private static int terminateToolboxCondition = 1000;

   // public CTTaskNodeWholeBodyTrajectoryMessageFactory ctTaskNodeWholeBodyTrajectoryMessageFactory;

   /**
    * Toolbox state
    */
   private CWBToolboxState state;

   private enum CWBToolboxState
   {
      DO_NOTHING, FIND_INITIAL_GUESS, EXPAND_TREE, SHORTCUT_PATH, GENERATE_MOTION
   }

   /*
    * parallel family.
    */

   private final CTTreeFindInitialGuess ctTreeFindInitialGuess;

   private final CommandInputManager commandInputManager;

   public WholeBodyTrajectoryToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel,
                                                        CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                                        YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry, boolean startYoVariableServer)
   {
      super(statusOutputManager, registry);
      this.drcRobotModelFactory = drcRobotModel;
      this.commandInputManager = commandInputManager;

      if (this.drcRobotModelFactory.getHandModel() == null)
      {
         handCoordinateOffsetX = handOffset_NoHand_Version;
      }
      else
         handCoordinateOffsetX = handOffset_DualRobotiQ_Version;

      this.visualizedFullRobotModel = fullRobotModel;
      this.isDone.set(false);

      this.startYoVariableServer = startYoVariableServer;
      this.treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsRegistry, registry);
      this.state = CWBToolboxState.DO_NOTHING;

      for (RobotSide robotSide : RobotSide.values)
      {
         this.endeffectorPose.put(robotSide, new YoFramePose("" + robotSide + "endeffectorPose", ReferenceFrame.getWorldFrame(), registry));

         this.endeffectorFrame.put(robotSide,
                                   new YoGraphicCoordinateSystem("" + robotSide + "endeffectorPoseFrame", this.endeffectorPose.get(robotSide), 0.25));
         this.endeffectorFrame.get(robotSide).setVisible(true);
         yoGraphicsRegistry.registerYoGraphic("" + robotSide + "endeffectorPoseViz", this.endeffectorFrame.get(robotSide));
      }

      this.state = CWBToolboxState.FIND_INITIAL_GUESS;

      this.ctTreeFindInitialGuess = new CTTreeFindInitialGuess(drcRobotModel, 4, registry);

      kinematicsSolver = new WheneverWholeBodyKinematicsSolver(drcRobotModelFactory);

      this.toolboxSolution = new WholeBodyTrajectoryToolboxOutputStatus();
      this.toolboxSolution.setDestination(-1);
   }

   @Override
   protected void updateInternal() throws InterruptedException, ExecutionException
   {
      updateCount.increment();
      // PrintTools.info("" + updateCount.getIntegerValue() + " " + state);

      // ************************************************************************************************************** //      
      switch (state)
      {
      case DO_NOTHING:

         break;
      case FIND_INITIAL_GUESS:

         // findInitialGuess();
         findInitialGuess2();

         break;
      case EXPAND_TREE:

         expandingTree();

         break;
      case SHORTCUT_PATH:

         shortcutPath();

         break;
      case GENERATE_MOTION:

         generateMotion();

         break;
      }
      // ************************************************************************************************************** //

      // ************************************************************************************************************** //

      updateVisualizerRobotConfiguration();
      updateVisualizers();
      updateYoVariables();

      // ************************************************************************************************************** //      
      if (updateCount.getIntegerValue() == terminateToolboxCondition)
         terminateToolboxController();
   }

   /**
    * state == GENERATE_MOTION
    */
   private void generateMotion()
   {
      int sizeOfPath = tree.getPath().size();

      CTTaskNode node = tree.getPath().get(sizeOfPath - numberOfMotionPath);

      visualizedNode = node;

      numberOfMotionPath--;

      if (numberOfMotionPath == 0)
      {
         /*
          * generate WholeBodyTrajectoryMessage.
          */
         terminateToolboxController();

         // TODO         
         setOutputStatus(toolboxSolution, 4);
         setOutputStatus(toolboxSolution, tree.getPath());

         toolboxSolution.setDestination(PacketDestination.BEHAVIOR_MODULE);

         reportMessage(toolboxSolution);
      }
   }

   // TODO
   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, int planningResult)
   {
      outputStatusToPack.setPlanningResult(planningResult);
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, ArrayList<CTTaskNode> path)
   {
      if (outputStatusToPack.getPlanningResult() == 4)
      {
         CTTaskNodeWholeBodyTrajectoryMessageFactory ctTaskNodeWholeBodyTrajectoryMessageFactory = new CTTaskNodeWholeBodyTrajectoryMessageFactory();
         ctTaskNodeWholeBodyTrajectoryMessageFactory.setCTTaskNodePath(tree.getPath(), constrainedEndEffectorTrajectory);

         WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = ctTaskNodeWholeBodyTrajectoryMessageFactory.getWholeBodyTrajectoryMessage();
         outputStatusToPack.setWholeBodyTrajectoryMessage(wholeBodyTrajectoryMessage);

         outputStatusToPack.setRobotConfigurations(ctTaskNodeWholeBodyTrajectoryMessageFactory.getConfigurations());
         outputStatusToPack.setTrajectoryTimes(ctTaskNodeWholeBodyTrajectoryMessageFactory.getTrajectoryTimes());
      }
      else
      {
         PrintTools.info("Planning is Failed.");
      }
   }

   /**
    * state = SHORTCUT_PATH
    */
   private void shortcutPath()
   {
      tree.getPath().clear();

      ArrayList<CTTaskNode> revertedPath = new ArrayList<CTTaskNode>();
      CTTaskNode currentNode = new CTTaskNode(tree.getNewNode());
      revertedPath.add(currentNode);

      while (true)
      {
         currentNode = currentNode.getParentNode();
         if (currentNode != null)
            revertedPath.add(new CTTaskNode(currentNode));
         else
            break;
      }

      int revertedPathSize = revertedPath.size();

      tree.addNodeOnPath(new CTTaskNode(revertedPath.get(revertedPathSize - 1)));
      int currentIndex = 0;
      for (int j = 0; j < revertedPathSize - 2; j++)
      {
         int i = revertedPathSize - 1 - j;

         double generalizedTimeGap = revertedPath.get(i - 1).getNormalizedNodeData(0) - tree.getPath().get(currentIndex).getNormalizedNodeData(0);

         if (generalizedTimeGap > tree.dismissibleTimeGap)
         {
            tree.addNodeOnPath(new CTTaskNode(revertedPath.get(i - 1)));
            currentIndex++;
         }
      }
      tree.addNodeOnPath(new CTTaskNode(revertedPath.get(0)));

      // set every parent nodes.
      for (int i = 0; i < tree.getPath().size() - 1; i++)
      {
         tree.getPath().get(i + 1).setParentNode(tree.getPath().get(i));
      }

      treeVisualizer.update(tree.getPath());

      long time1 = System.currentTimeMillis();

      for (int i = 0; i < numberOfTimesToShortcut; i++)
         updateShortcutPath(tree.getPath());

      long time2 = System.currentTimeMillis();
      PrintTools.info("shortcut time is = " + (time2 - time1) / 1000.0);
      PrintTools.info("the size of the path is " + tree.getPath().size() + " before dismissing " + revertedPathSize);

      numberOfMotionPath = tree.getPath().size();

      //if(startYoVariableServer)
      if (true)
      {
         treeVisualizer.showUpPath(tree.getPath());
      }

      /*
       * terminate state
       */
      state = CWBToolboxState.GENERATE_MOTION;
   }

   /**
    * state == EXPAND_TREE
    */
   private void expandingTree()
   {
      expandingCount.increment();

      tree.updateRandomConfiguration();

      tree.updateNearestNodeTaskTime();
      tree.updateNewConfiguration();
      tree.getNewNode().convertNormalizedDataToData(taskRegion);
      tree.getNewNode().setParentNode(tree.getNearNode());

      updateValidity(tree.getNewNode());
      if (tree.getNewNode().getValidity())
      {
         tree.connectNewNode(true);
         if (tree.getNewNode().getTime() == taskRegion.getTrajectoryTime())
         {
            PrintTools.info("terminate expanding");
            maximumExpansionSize = 1; // for terminate
         }
      }
      else
      {
         tree.connectNewNode(false);

         // Option 1 : Two way Expanding strategy.
         //         tree.updateNearestNodeTaskOnly();
         //                  
         //         tree.updateNewConfiguration();         
         //         tree.getNewNode().convertNormalizedDataToData(taskRegion);
         //                  
         //         tree.getNewNode().setParentNode(tree.getNearNode());
         //
         //         updateValidity(tree.getNewNode());
         //         if (tree.getNewNode().getValidity())
         //         {            
         //            tree.connectNewNode(true);
         //            if (tree.getNewNode().getTime() == taskRegion.getTrajectoryTime())
         //            {
         //               PrintTools.info("terminate expanding");
         //               numberOfExpanding = 1; // for terminate
         //            }
         //         }
      }

      visualizedNode = new CTTaskNode(tree.getNewNode());
      // TODO
      visualizedNode.setValidity(tree.getNewNode().getValidity());

      double jointScore = kinematicsSolver.getArmJointLimitScore();

      jointlimitScore.set(jointScore);

      /*
       * terminate expanding tree.
       */
      maximumExpansionSize--;
      if (maximumExpansionSize == 0)
      {
         if (tree.getTreeReachingTime() != 1.0)
         {
            PrintTools.info("Fail to reach end.");

            terminateToolboxController();
         }
         else
         {
            state = CWBToolboxState.SHORTCUT_PATH;

            PrintTools.info("Total update solver");
         }
      }
   }

   /**
    * state == FIND_INITIAL_GUESS
    * 
    * @throws ExecutionException
    * @throws InterruptedException
    */
   private void findInitialGuess() throws InterruptedException, ExecutionException
   {
      ctTreeFindInitialGuess.findInitialGuess(rootNode, taskRegion, initialConfiguration, constrainedEndEffectorTrajectory, handCoordinateOffsetX);

      double scoreInitialGuess = ctTreeFindInitialGuess.getBestScore();
      visualizedNode = ctTreeFindInitialGuess.getBestNode();

      jointlimitScore.set(scoreInitialGuess);

      if (bestScoreInitialGuess < scoreInitialGuess)
      {
         bestScoreInitialGuess = scoreInitialGuess;

         rootNode = new CTTaskNode(visualizedNode);
      }

      /*
       * terminate finding initial guess.
       */
      numberOfInitialGuesses -= ctTreeFindInitialGuess.getInitialGuessNodes().size();
      if (numberOfInitialGuesses < 1)
      {
         PrintTools.info("initial guess terminate");

         if (rootNode.getValidity() == false)
         {
            terminateToolboxController();
         }
         else
         {
            state = CWBToolboxState.EXPAND_TREE;

            rootNode.convertDataToNormalizedData(taskRegion);

            PrintTools.info("" + bestScoreInitialGuess);
            for (int i = 0; i < rootNode.getDimensionOfNodeData(); i++)
               PrintTools.info("" + i + " " + rootNode.getNodeData(i));

            tree = new CTTaskNodeTree(rootNode);
            tree.setTaskRegion(taskRegion);
         }
      }
   }

   private void findInitialGuess2()
   {
      CTTaskNode initialGuessNode = new GenericTaskNode();

      CTTreeTools.setRandomNormalizedNodeData(initialGuessNode, true, 0.0);
      initialGuessNode.setNormalizedNodeData(0, 0);
      initialGuessNode.convertNormalizedDataToData(taskRegion);

      updateValidity(initialGuessNode);

      visualizedNode = initialGuessNode;

      double jointScore = kinematicsSolver.getArmJointLimitScore();

      jointlimitScore.set(jointScore);

      if (bestScoreInitialGuess < jointScore && visualizedNode.getValidity() == true)
      {
         bestScoreInitialGuess = jointScore;

         rootNode = new CTTaskNode(visualizedNode);
         rootNode.setValidity(visualizedNode.getValidity());
      }

      /*
       * terminate finding initial guess.
       */
      numberOfInitialGuesses -= 1;
      if (numberOfInitialGuesses < 1)
      {
         PrintTools.info("initial guess terminate");

         if (rootNode.getValidity() == false)
         {
            PrintTools.info("Real???? ");
            terminateToolboxController();
         }
         else
         {
            state = CWBToolboxState.EXPAND_TREE;

            rootNode.convertDataToNormalizedData(taskRegion);

            PrintTools.info("" + bestScoreInitialGuess);
            for (int i = 0; i < rootNode.getDimensionOfNodeData(); i++)
               PrintTools.info("" + i + " " + rootNode.getNodeData(i));

            tree = new CTTaskNodeTree(rootNode);
            tree.setTaskRegion(taskRegion);
         }
      }
   }

   long startTime;

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      if (!commandInputManager.isNewCommandAvailable(WaypointBasedTrajectoryCommand.class))
         return false;

      List<WaypointBasedTrajectoryCommand> trajectoryCommands = commandInputManager.pollNewCommands(WaypointBasedTrajectoryCommand.class);

      constrainedEndEffectorTrajectory = convertCommands(trajectoryCommands);

      PrintTools.info("initialize CWB toolbox");

      /*
       * bring control parameters from request.
       */
      boolean success = updateConfiguration();
      if (!success)
         return false;

      /*
       * initialize kinematicsSolver.
       */
      kinematicsSolver.updateRobotConfigurationData(initialConfiguration);

      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();
      kinematicsSolver.putTrajectoryMessages();

      /*
       * start toolbox
       */
      rootNode = new GenericTaskNode();
      tree = new CTTaskNodeTree(rootNode);
      taskRegion = constrainedEndEffectorTrajectory.getTaskRegion();

      tree.setTaskRegion(taskRegion); //////////////////////////////////////////////////////////

      rootNode.convertDataToNormalizedData(taskRegion);

      /*
       * bring constrainedEndEffectorTrajectory
       */
      // if (startYoVariableServer)
      if (true)
      {
         treeVisualizer = new CTTreeVisualizer(tree);
         treeVisualizer.initialize();
      }

      startTime = System.currentTimeMillis();
      return true;
   }

   private boolean updateConfiguration()
   {
      int newMaxExpansionSize = -1;
      int newNumberOfInitialGuesses = -1;
      KinematicsToolboxOutputStatus newInitialConfiguration = null;

      if (commandInputManager.isNewCommandAvailable(WholeBodyTrajectoryToolboxConfigurationCommand.class))
      {
         WholeBodyTrajectoryToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(WholeBodyTrajectoryToolboxConfigurationCommand.class);
         newMaxExpansionSize = command.getMaximumExpansionSize();
         newNumberOfInitialGuesses = command.getNumberOfInitialGuesses();

         if (command.hasInitialConfiguration())
         {
            newInitialConfiguration = command.getInitialConfiguration();
         }
      }

      if (newMaxExpansionSize > 0)
      {
         maximumExpansionSize = newMaxExpansionSize;
      }
      else
      {
         maximumExpansionSize = DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE;
      }
      
      if (newNumberOfInitialGuesses > 0)
      {
         numberOfInitialGuesses = newNumberOfInitialGuesses;
      }
      else
      {
         numberOfInitialGuesses = DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE;
      }

      if (newInitialConfiguration != null)
      {
         initialConfiguration.set(newInitialConfiguration);
         return true;
      }

      RobotConfigurationData currentRobotConfiguration = currentRobotConfigurationDataReference.getAndSet(null);
      if (currentRobotConfiguration == null)
         return false;

      initialConfiguration.desiredRootOrientation.set(currentRobotConfiguration.getPelvisOrientation());
      initialConfiguration.desiredRootTranslation.set(currentRobotConfiguration.getPelvisTranslation());
      
      initialConfiguration.jointNameHash = currentRobotConfiguration.jointNameHash;
      int length = currentRobotConfiguration.jointAngles.length;
      initialConfiguration.desiredJointAngles = new float[length];
      System.arraycopy(currentRobotConfiguration.jointAngles, 0, initialConfiguration.desiredJointAngles, 0, length);

      return true;
   }

   private ConstrainedEndEffectorTrajectory convertCommands(List<WaypointBasedTrajectoryCommand> commands)
   {
      SideDependentList<WaypointBasedTrajectoryCommand> handTrajectories = new SideDependentList<>();

      for (int i = 0; i < commands.size(); i++)
      {
         WaypointBasedTrajectoryCommand command = commands.get(i);

         for (RobotSide robotSide : RobotSide.values)
         {
            if (command.getEndEffector().equals(visualizedFullRobotModel.getHand(robotSide)))
            {
               handTrajectories.put(robotSide, command);
               break;
            }
         }
      }

      double trajectoryTime = 0.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         trajectoryTime = Math.max(trajectoryTime, handTrajectories.get(robotSide).getLastWaypointTime());
      }

      SideDependentList<SelectionMatrix6D> controllableSelectionMatricesRe = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
         selectionMatrix6D.clearSelection();
         for (int i = 0; i < handTrajectories.get(robotSide).getNumberOfUnconstrainedDegreesOfFreedom(); i++)
         {
            ConfigurationSpaceName spaceName = handTrajectories.get(robotSide).getUnconstrainedDegreeOfFreedom(i);
            switch (spaceName)
            {
            case X:
               selectionMatrix6D.selectLinearX(true);
               break;
            case Y:
               selectionMatrix6D.selectLinearY(true);
               break;
            case Z:
               selectionMatrix6D.selectLinearZ(true);
               break;
            case ROLL:
               selectionMatrix6D.selectAngularX(true);
               break;
            case PITCH:
               selectionMatrix6D.selectAngularY(true);
               break;
            case YAW:
               selectionMatrix6D.selectAngularZ(true);
               break;
            default:
               throw new RuntimeException("Unexpected enum value: " + spaceName);
            }
         }
         controllableSelectionMatricesRe.put(robotSide, selectionMatrix6D);
      }

      // TODO Need to clean that up
      ConstrainedEndEffectorTrajectory endEffectorTrajectory = new ConstrainedEndEffectorTrajectory(trajectoryTime)
      {
         
         @Override
         public TaskRegion defineTaskRegion()
         {
            TaskRegion taskNodeRegion = new TaskRegion(GenericTaskNode.nodeDimension);

            taskNodeRegion.setRandomRegion(0, 0.0, trajectoryTime);
            
            taskNodeRegion.setRandomRegion(1, 0.75, 0.90); // Pelvis height
            taskNodeRegion.setRandomRegion(2, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI); // Chest yaw
            taskNodeRegion.setRandomRegion(3, -20.0 / 180 * Math.PI, 20.0 / 180 * Math.PI); // Chest pitch
            taskNodeRegion.setRandomRegion(4, -8.0 / 180 * Math.PI, 8.0 / 180 * Math.PI); // Chest roll

            taskNodeRegion.setRandomRegion(5, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(6, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(7, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(8, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(9, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(10, -160.0 / 180 * Math.PI, -20.0 / 180 * Math.PI);

            taskNodeRegion.setRandomRegion(11, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(12, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(13, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(14, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(15, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(16, 0.0, 0.0);

            return taskNodeRegion;
         }
         
         @Override
         public SideDependentList<SelectionMatrix6D> defineControllableSelectionMatrices()
         {
            return controllableSelectionMatricesRe;
         }
         
         @Override
         public SideDependentList<ConfigurationBuildOrder> defineConfigurationBuildOrders()
         {
            SideDependentList<ConfigurationBuildOrder> configurationBuildOrders = new SideDependentList<>();

            for (RobotSide robotSide : RobotSide.values)
               configurationBuildOrders.put(robotSide,
                                            new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                                        ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL));

            return configurationBuildOrders;
         }
         
         @Override
         public SideDependentList<ConfigurationSpace> getConfigurationSpace(double time)
         {
            SideDependentList<ConfigurationSpace> configurationSpace = new SideDependentList<>();

            for (RobotSide robotSide : RobotSide.values)
            {
               Pose3D current = new Pose3D();

               WaypointBasedTrajectoryCommand handTrajectory = handTrajectories.get(robotSide);
               Pose3D previous = null;
               Pose3D next = null;
               double t0 = Double.NaN;
               double tf = Double.NaN;

               for (int i = 1; i < handTrajectory.getNumberOfWaypoints(); i++)
               {
                  t0 = handTrajectory.getWaypointTime(i - 1);
                  tf = handTrajectory.getWaypointTime(i);
                  previous = handTrajectory.getWaypoint(i - 1);
                  next = handTrajectory.getWaypoint(i);
                  if (time < handTrajectory.getWaypointTime(i))
                     break;
               }

               double alpha = (time - t0) / (tf - t0);
               alpha = MathTools.clamp(alpha, 0, 1);               
               current.interpolate(previous, next, alpha);

               double x = current.getX();
               double y = current.getY();
               double z = current.getZ();
               RotationMatrix rot = new RotationMatrix(current.getOrientation());
               double roll  = rot.getRoll();
               double pitch = rot.getPitch();
               double yaw   = rot.getYaw();
               configurationSpace.put(robotSide, new ConfigurationSpace(x, y, z, roll, pitch, yaw));
            }

            return configurationSpace;
         }
      };
      return endEffectorTrajectory;
   }

   private void terminateToolboxController()
   {
      state = CWBToolboxState.DO_NOTHING;

      long stopTime = System.currentTimeMillis();
      long elapsedTime = stopTime - startTime;
      System.out.println("===========================================");
      System.out.println("toolbox executing time is " + elapsedTime / 1000.0 + " seconds " + updateCount.getIntegerValue());
      System.out.println("===========================================");

      isDone.set(true);
   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   /**
    * update validity of input node.
    */
   private boolean updateValidity(CTTaskNode node)
   {
      long astartTime = System.currentTimeMillis();

      if (node.getParentNode() != null)
      {
         kinematicsSolver.updateRobotConfigurationData(node.getParentNode().getConfiguration());
      }
      else
      {
         kinematicsSolver.updateRobotConfigurationData(initialConfiguration);
      }

      kinematicsSolver.initialize();

      kinematicsSolver.holdCurrentTrajectoryMessages();

      /*
       * set whole body tasks. pose from 'constrainedEndEffectorTrajectory' is considered as being
       * in MidZUpframe. for kinematicsSolver, append offset
       */
      SideDependentList<ConfigurationSpace> configurationSpaces = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         configurationSpaces.put(robotSide, CTTreeTools.getConfigurationSpace(node, robotSide));

         Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(node.getNodeData(0), robotSide, configurationSpaces.get(robotSide));
         setEndEffectorPose(robotSide, desiredPose);

         desiredPose.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);

         kinematicsSolver.setDesiredHandPose(robotSide, desiredPose);
      }

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
      kinematicsSolver.solve();
      boolean result = kinematicsSolver.getResult();

      node.setConfigurationJoints(kinematicsSolver.getSolution());

      node.setValidity(result);

      cntKinematicSolver.set(kinematicsSolver.getCntForUpdateInternal());

      long stopTime = System.currentTimeMillis();
      long elapsedTime = stopTime - astartTime;

      // System.out.println("elapsed time is " + elapsedTime / 1000.0 + " seconds " + cntKinematicSolver.getIntegerValue() +" "+ result);

      return result;
   }

   /**
    * set fullRobotModel.
    */
   private void updateVisualizerRobotConfiguration(KinematicsToolboxOutputStatus robotKinematicsConfiguration)
   {
      robotKinematicsConfiguration.getDesiredJointState(visualizedFullRobotModel.getRootJoint(),
                                                        FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   private void updateVisualizerRobotConfiguration()
   {
      updateVisualizerRobotConfiguration(visualizedNode.getConfiguration());
   }

   /**
    * update visualizers.
    */
   private void updateVisualizers()
   {
      if (visualizedNode != null)
      {
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getNormalizedNodeData(0));
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.getValidity());
         treeStateVisualizer.updateVisualizer();

         currentIsValid.set(visualizedNode.getValidity());
         currentTrajectoryTime.set(visualizedNode.getNormalizedNodeData(0));
         // if (startYoVariableServer)
         if (true)
            treeVisualizer.update(visualizedNode);
      }
   }

   /**
    * YoVariables.
    */
   private void updateYoVariables()
   {
      solutionQuality.set(kinematicsSolver.getSolution().getSolutionQuality());

      for (RobotSide robotSide : RobotSide.values)
      {
         endeffectorFrame.get(robotSide).setVisible(true);
         endeffectorFrame.get(robotSide).update();
      }
   }

   /**
    * update end effector pose
    */
   private void setEndEffectorPose(RobotSide robotSide, Pose3D desiredPose)
   {
      endeffectorPose.get(robotSide).setPosition(desiredPose.getPosition());
      endeffectorPose.get(robotSide).setOrientation(desiredPose.getOrientation());
   }

   /**
    * oneTime shortcut : try to make a shortcut from index to index+2
    */
   private void updateShortcutPath(ArrayList<CTTaskNode> path, int index)
   {
      // check out when index is over the size.
      if (index > path.size() - 3)
      {
         return;
      }

      CTTaskNode nodeFrom = new CTTaskNode(path.get(index));
      CTTaskNode nodeGoal = new CTTaskNode(path.get(index + 2));

      CTTaskNode nodeShortcut = path.get(index + 1);
      CTTaskNode nodeDummy = new CTTaskNode(nodeShortcut);

      for (int i = 0; i < nodeShortcut.getDimensionOfNodeData(); i++)
      {
         double dataFrom = nodeFrom.getNodeData(i);
         double dataGoal = nodeGoal.getNodeData(i);

         double dataShortcut = (dataGoal + dataFrom) / 2;
         nodeDummy.setNodeData(i, dataShortcut);
      }

      if (nodeDummy.getParentNode() == null)
      {
         PrintTools.info("no parent!");
      }
      updateValidity(nodeDummy);
      if (nodeDummy.getValidity())
      {
         for (int i = 0; i < nodeShortcut.getDimensionOfNodeData(); i++)
         {
            nodeShortcut.setNodeData(i, nodeDummy.getNodeData(i));
         }
      }
   }

   private void updateShortcutPath(ArrayList<CTTaskNode> path)
   {
      for (int i = 0; i < path.size(); i++)
      {
         updateShortcutPath(path, i);
         path.get(i).convertDataToNormalizedData(taskRegion);
      }
   }

   void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      currentRobotConfigurationDataReference.set(newConfigurationData);
   }

   FullHumanoidRobotModel getSolverFullRobotModel()
   {
      return kinematicsSolver.getDesiredFullRobotModel();
   }
}