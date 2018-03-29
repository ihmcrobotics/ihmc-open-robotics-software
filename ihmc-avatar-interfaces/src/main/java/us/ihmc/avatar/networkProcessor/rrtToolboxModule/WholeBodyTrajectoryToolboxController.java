package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxSettings;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyTrajectoryToolboxController extends ToolboxController
{
   private static final boolean VERBOSE = true;
   private static final int DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION = 10;
   private static final int DEFAULT_MAXIMUM_NUMBER_OF_ITERATIONS = 3000;
   private static final int DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE = 1000;
   private static final int DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE = 200;
   private static final int TERMINAL_CONDITION_NUMBER_OF_VALID_INITIAL_GUESSES = 20;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Random randomManager = new Random(1);

   private final HumanoidKinematicsSolver humanoidKinematicsSolver;

   private final WholeBodyTrajectoryToolboxOutputStatus toolboxSolution;

   private WholeBodyTrajectoryToolboxData toolboxData;

   private List<RigidBodyExplorationConfigurationCommand> rigidBodyCommands = null;
   private List<WaypointBasedTrajectoryCommand> trajectoryCommands = null;
   private List<ReachingManifoldCommand> manifoldCommands = null;

   /*
    * YoVariables
    */
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
   private final YoInteger currentNumberOfIterations = new YoInteger("currentNumberOfIterations", registry);
   private final YoInteger terminalConditionNumberOfValidInitialGuesses = new YoInteger("terminalConditionNumberOfValidInitialGuesses", registry);

   // check the tree reaching the normalized time from 0.0 to 1.0.
   private final YoDouble currentTrajectoryTime = new YoDouble("currentNormalizedTime", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final YoDouble jointlimitScore = new YoDouble("jointlimitScore", registry);

   private final YoDouble bestScoreInitialGuess = new YoDouble("bestScoreInitialGuess", registry);

   private final YoBoolean isValidNode = new YoBoolean("isValidNode", registry);

   /*
    * Visualizer
    */
   private boolean visualize;

   private SpatialNode visualizedNode;

   private final KinematicsToolboxOutputConverter configurationConverter;
   private final KinematicsToolboxOutputStatus initialConfiguration = new KinematicsToolboxOutputStatus();
   private final AtomicReference<RobotConfigurationData> currentRobotConfigurationDataReference = new AtomicReference<>(null);

   private FullHumanoidRobotModel visualizedFullRobotModel;

   private TreeStateVisualizer treeStateVisualizer;

   private SpatialNodePlotter nodePlotter;

   private final SideDependentList<YoFramePose> endeffectorPose = new SideDependentList<>();

   private final SideDependentList<YoGraphicCoordinateSystem> endeffectorFrame = new SideDependentList<>();

   private final YoFramePose testFramePose;
   private final YoGraphicCoordinateSystem testFrameViz;
   private final YoDouble minimumDistanceFromManifold = new YoDouble("minimumDistanceFromManifold", registry);

   /*
    * Configuration and Time space Tree
    */
   private SpatialNode rootNode = null;
   private SpatialNodeTree tree;
   private final List<SpatialNode> path = new ArrayList<>();
   private final double minTimeInterval = 0.05;

   private int numberOfValidPosture = 0;
   private int numberOfInvalidPosture = 0;
   private final YoInteger currentExpansionSize = new YoInteger("currentExpansionSize", registry);
   private final YoInteger maximumExpansionSize = new YoInteger("maximumExpansionSize", registry);

   private final YoInteger currentNumberOfValidInitialGuesses = new YoInteger("currentNumberOfValidInitialGuesses", registry);
   private final YoInteger currentNumberOfInitialGuesses = new YoInteger("currentNumberOfInitialGuesses", registry);
   private final YoInteger desiredNumberOfInitialGuesses = new YoInteger("desiredNumberOfInitialGuesses", registry);

   private YoInteger numberOfIterationForShortcutOptimization = new YoInteger("numberOfIterationForShortcutOptimization", registry);

   /**
    * Toolbox state
    */
   private final YoEnum<CWBToolboxState> state = new YoEnum<>("state", registry, CWBToolboxState.class);

   private enum CWBToolboxState
   {
      DO_NOTHING, FIND_INITIAL_GUESS, EXPAND_TREE, SHORTCUT_PATH, GENERATE_MOTION
   }

   private final YoDouble initialGuessComputationTime = new YoDouble("initialGuessComputationTime", registry);
   private final YoDouble treeExpansionComputationTime = new YoDouble("treeExpansionComputationTime", registry);
   private final YoDouble shortcutPathComputationTime = new YoDouble("shortcutPathComputationTime", registry);
   private final YoDouble motionGenerationComputationTime = new YoDouble("motionGenerationComputationTime", registry);
   private final YoDouble totalComputationTime = new YoDouble("totalComputationTime", registry);

   private long initialGuessStartTime;
   private long treeExpansionStartTime;
   private long shortcutStartTime;
   private long motionGenerationStartTime;

   private final CommandInputManager commandInputManager;

   public WholeBodyTrajectoryToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                               StatusMessageOutputManager statusOutputManager, YoVariableRegistry registry,
                                               YoGraphicsListRegistry yoGraphicsListRegistry, boolean visualize)
   {
      super(statusOutputManager, registry);
      this.commandInputManager = commandInputManager;

      visualizedFullRobotModel = fullRobotModel;
      isDone.set(false);

      this.visualize = visualize;
      if (visualize)
      {
         treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsListRegistry, registry);
      }
      else
      {
         treeStateVisualizer = null;
      }
      state.set(CWBToolboxState.DO_NOTHING);

      for (RobotSide robotSide : RobotSide.values)
      {
         endeffectorPose.put(robotSide, new YoFramePose("" + robotSide + "endeffectorPose", ReferenceFrame.getWorldFrame(), registry));

         endeffectorFrame.put(robotSide, new YoGraphicCoordinateSystem("" + robotSide + "endeffectorPoseFrame", endeffectorPose.get(robotSide), 0.25));
         endeffectorFrame.get(robotSide).setVisible(true);
         yoGraphicsListRegistry.registerYoGraphic("" + robotSide + "endeffectorPoseViz", endeffectorFrame.get(robotSide));
      }

      numberOfIterationForShortcutOptimization.set(DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION);
      maximumNumberOfIterations.set(DEFAULT_MAXIMUM_NUMBER_OF_ITERATIONS);
      terminalConditionNumberOfValidInitialGuesses.set(TERMINAL_CONDITION_NUMBER_OF_VALID_INITIAL_GUESSES);

      humanoidKinematicsSolver = new HumanoidKinematicsSolver(drcRobotModel, yoGraphicsListRegistry, registry);

      toolboxSolution = new WholeBodyTrajectoryToolboxOutputStatus();
      toolboxSolution.setDestination(-1);

      configurationConverter = new KinematicsToolboxOutputConverter(drcRobotModel);

      testFramePose = new YoFramePose("testFramePose", ReferenceFrame.getWorldFrame(), registry);
      testFrameViz = new YoGraphicCoordinateSystem("testFrameViz", testFramePose, 0.25);
      yoGraphicsListRegistry.registerYoGraphic("testFrameYoGraphic", testFrameViz);
   }

   @Override
   protected void updateInternal() throws InterruptedException, ExecutionException
   {
      currentNumberOfIterations.increment();

      // ************************************************************************************************************** //
      switch (state.getEnumValue())
      {
      case DO_NOTHING:

         break;
      case FIND_INITIAL_GUESS:

         findInitialGuess();

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
      if (currentNumberOfIterations.getIntegerValue() == maximumNumberOfIterations.getIntegerValue())
      {
         terminateToolboxController();
      }
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, int planningResult)
   {
      outputStatusToPack.setPlanningResult(planningResult);
   }

   private void setOutputStatus(WholeBodyTrajectoryToolboxOutputStatus outputStatusToPack, List<SpatialNode> path)
   {
      if (outputStatusToPack.getPlanningResult() == 4)
      {
         MessageTools.copyData(path.stream().map(SpatialNode::getConfiguration).toArray(size -> new KinematicsToolboxOutputStatus[size]), outputStatusToPack.robotConfigurations);
         outputStatusToPack.trajectoryTimes.reset();
         outputStatusToPack.trajectoryTimes.add(path.stream().mapToDouble(SpatialNode::getTime).toArray());

         //if (VERBOSE)
         //   for (int i = 0; i < path.size(); i++)
         //      PrintTools.info("FINAL RESULT IS " + i + " " + path.get(i).isValid());

      }
      else
      {
         if (VERBOSE)
            PrintTools.info("Planning has Failed.");
      }
   }

   /**
    * state == GENERATE_MOTION
    */
   private void generateMotion()
   {
      // TODO : WholeBodyTrajectoryBehavior should be created.
      // TODO : WholeBodyTrajectoryToolboxOutputConverter should be used.
      
      updateTimer(motionGenerationComputationTime, motionGenerationStartTime);

      /*
       * terminate generateMotion.
       */
      if (true)
      {
         setOutputStatus(toolboxSolution, 4);
         setOutputStatus(toolboxSolution, path);

         terminateToolboxController();
      }
   }

   /**
    * state = SHORTCUT_PATH
    */
   private void shortcutPath()
   {
      path.clear();

      List<SpatialNode> revertedPath = new ArrayList<SpatialNode>();
      SpatialNode currentNode = new SpatialNode(tree.getLastNodeAdded());
      revertedPath.add(currentNode);

      while (true)
      {
         currentNode = currentNode.getParent();
         if (currentNode != null)
            revertedPath.add(new SpatialNode(currentNode));
         else
            break;
      }

      int revertedPathSize = revertedPath.size();

      path.add(new SpatialNode(revertedPath.get(revertedPathSize - 1)));

      // filtering out too short time gap node.
      // test ahead adding and attach only valid filtering.
      // TODO : but still has problem. To test ahead whole shortcut would be too heavy.      
      int currentIndex = 0;
      int latestAddedIndexOfRevertedPath = revertedPathSize - 1;
      for (int j = revertedPathSize - 2; j > 0; j--)
      {
         double timeGapWithLatestNode = revertedPath.get(j).getTime() - path.get(currentIndex).getTime();

         if (timeGapWithLatestNode > minTimeInterval)
         {
            SpatialNode dummyNode = new SpatialNode(revertedPath.get(j));
            dummyNode.setParent(path.get(currentIndex));
            updateValidity(dummyNode);
            if (dummyNode.isValid())
            {
               path.add(new SpatialNode(revertedPath.get(j)));
               latestAddedIndexOfRevertedPath = j;
               currentIndex++;
            }
            else
            {
               j = latestAddedIndexOfRevertedPath - 1;

               path.add(new SpatialNode(revertedPath.get(j)));
               latestAddedIndexOfRevertedPath = j;

               currentIndex++;
            }
         }
      }
      path.add(new SpatialNode(revertedPath.get(0)));

      // set every parent nodes.
      for (int i = 0; i < path.size() - 1; i++)
         path.get(i + 1).setParent(path.get(i));

      // plotting before smoothing.
      for (int i = 0; i < path.size(); i++)
         nodePlotter.update(path.get(i), 2);

      // smoothing over one mile stone node.
      int numberOfShortcut = 0;
      for (int i = 0; i < numberOfIterationForShortcutOptimization.getIntegerValue(); i++)
      {
         numberOfShortcut = i;
         if (updateShortcutPath(path) < 0.001)
            break;
      }

      // plotting final result.
      for (int i = 0; i < path.size(); i++)
         nodePlotter.update(path.get(i), 3);

      motionGenerationStartTime = updateTimer(shortcutPathComputationTime, shortcutStartTime);

      if (VERBOSE)
         PrintTools.info("the size of the path is " + path.size() + " before dismissing " + revertedPathSize + " shortcut " + numberOfShortcut);

      /*
       * terminate state
       */
      state.set(CWBToolboxState.GENERATE_MOTION);
   }

   /**
    * state == EXPAND_TREE
    */
   private void expandingTree()
   {
      currentExpansionSize.increment();
      boolean isExpandingTerminalCondition = false;

      boolean randomNodeHasParentNode = false;
      int maximumPatientCounter = 1000;
      while (!randomNodeHasParentNode)
      {
         SpatialNode randomNode;

         SpatialData randomData = toolboxData.createRandomSpatialData();
         double nextDouble = WholeBodyTrajectoryToolboxSettings.randomManager.nextDouble();
         double randomTime = nextDouble * (1.0 + WholeBodyTrajectoryToolboxSettings.timeCoefficient * tree.getMostAdvancedTime());

         randomNode = new SpatialNode(randomTime, randomData);

         tree.setRandomNode(randomNode);
         if (trajectoryCommands != null)
            randomNodeHasParentNode = tree.findNearestValidNodeToCandidate(true);
         if (manifoldCommands != null)
            randomNodeHasParentNode = tree.findNearestValidNodeToCandidate(false);

         if (randomNodeHasParentNode)
         {
            tree.limitCandidateDistanceFromParent(toolboxData.getTrajectoryTime());
            SpatialNode candidate = tree.getCandidate();
            updateValidity(candidate);

            /*
             * visualize
             */
            visualizedNode = new SpatialNode(candidate);
            nodePlotter.update(candidate, 1);

            if (candidate.isValid())
            {
               tree.attachCandidate();
               numberOfValidPosture++;

               // TODO: generic terminal conditions.
               if (trajectoryCommands != null)
               {
                  if (tree.getMostAdvancedTime() >= toolboxData.getTrajectoryTime())
                     isExpandingTerminalCondition = true;
               }
               else if (manifoldCommands != null)
               {  
                  Pose3D testFrame = toolboxData.getTestFrame(tree.getLastNodeAdded());

                  testFramePose.setPosition(testFrame.getPosition());
                  testFramePose.setOrientation(testFrame.getOrientation());
                  testFrameViz.setVisible(true);
                  testFrameViz.update();

                  // TODO : terminal condition for manifold command.
                  double maximumDistanceFromManifolds = toolboxData.getMaximumDistanceFromManifolds(tree.getLastNodeAdded());
                  minimumDistanceFromManifold.set(maximumDistanceFromManifolds);
                  if(maximumDistanceFromManifolds < 0.05)
                     isExpandingTerminalCondition = true;
               }
               else
               {
                  if (VERBOSE)
                     PrintTools.warn("any command is available");
               }
            }
            else
            {
               tree.dismissCandidate();
               numberOfInvalidPosture++;
            }
         }
         else
         {
            maximumPatientCounter--;
            if (maximumPatientCounter == 0)
               continue;
         }
      }

      /*
       * terminate expanding tree.
       */
      if (currentExpansionSize.getIntegerValue() >= maximumExpansionSize.getIntegerValue() || isExpandingTerminalCondition)
      {
         if (!isExpandingTerminalCondition)
         {
            if (VERBOSE)
               PrintTools.info("Failed to complete trajectory." + " " + numberOfValidPosture + " " + numberOfInvalidPosture);

            setOutputStatus(toolboxSolution, 2);
            terminateToolboxController();
         }
         else
         {
            if (VERBOSE)
               PrintTools.info("Successfully finished tree expansion. " + numberOfValidPosture + " " + numberOfInvalidPosture);
            state.set(CWBToolboxState.SHORTCUT_PATH);
         }
      }
      shortcutStartTime = updateTimer(treeExpansionComputationTime, treeExpansionStartTime);
   }

   /**
    * state == FIND_INITIAL_GUESS
    */
   private void findInitialGuessSub() // TODO : to enable this method, refine holding method.
   {
      SpatialData initialGuessData = toolboxData.createRandomInitialSpatialData();
      //SpatialData initialGuessData = toolboxData.createRandomSpatialData();

      SpatialNode initialGuessNode = new SpatialNode(initialGuessData);
      updateValidity(initialGuessNode);

      visualizedNode = initialGuessNode;

      double jointScore;
      if (visualizedNode.isValid())
         jointScore = computeArmJointsLimitScore(humanoidKinematicsSolver.getDesiredFullRobotModel());
      else
         jointScore = 0.0;

      jointlimitScore.set(jointScore);

      if (bestScoreInitialGuess.getDoubleValue() < jointScore && initialGuessNode.isValid())
      {
         bestScoreInitialGuess.set(jointScore);

         toolboxData.holdConfiguration(getSolverFullRobotModel());

         SpatialData dummyData = toolboxData.createRandomSpatialData();
         rootNode = new SpatialNode(dummyData);
         rootNode.setConfiguration(initialGuessNode.getConfiguration());
         rootNode.initializeSpatialData();

      }

      if (initialGuessNode.isValid())
         currentNumberOfValidInitialGuesses.increment();

      /*
       * terminate finding initial guess.
       */
      currentNumberOfInitialGuesses.increment();

      if (currentNumberOfInitialGuesses.getIntegerValue() >= desiredNumberOfInitialGuesses.getIntegerValue()
            || currentNumberOfValidInitialGuesses.getIntegerValue() >= terminalConditionNumberOfValidInitialGuesses.getIntegerValue())
      {
         if (rootNode == null || !rootNode.isValid())
         {
            if (VERBOSE)
               PrintTools.info("Did not find a single valid root node.");
            setOutputStatus(toolboxSolution, 1);
            terminateToolboxController();
         }
         else
         {
            if (VERBOSE)
               PrintTools.info("Successfully finished initial guess stage. " + currentNumberOfInitialGuesses.getIntegerValue() + " "
                     + currentNumberOfValidInitialGuesses.getIntegerValue());
            state.set(CWBToolboxState.EXPAND_TREE);

            toolboxData.updateInitialConfiguration();
            tree = new SpatialNodeTree(rootNode);
         }
      }
      treeExpansionStartTime = updateTimer(initialGuessComputationTime, initialGuessStartTime);
   }

   private void findInitialGuess()
   {
      SpatialData initialGuessData = toolboxData.createRandomSpatialData();

      SpatialNode initialGuessNode = new SpatialNode(initialGuessData);
      updateValidity(initialGuessNode);

      visualizedNode = initialGuessNode;

      double jointScore = 0.0;
      if (initialGuessNode.isValid())
      {
         tree.addInitialNode(initialGuessNode);
         currentNumberOfValidInitialGuesses.increment();
         jointScore = computeArmJointsLimitScore(humanoidKinematicsSolver.getDesiredFullRobotModel());
      }
      jointlimitScore.set(jointScore);

      nodePlotter.update(initialGuessNode, 1);

      /*
       * terminate finding initial guess.
       */
      currentNumberOfInitialGuesses.increment();

      if (currentNumberOfInitialGuesses.getIntegerValue() >= desiredNumberOfInitialGuesses.getIntegerValue()
            || currentNumberOfValidInitialGuesses.getIntegerValue() >= terminalConditionNumberOfValidInitialGuesses.getIntegerValue())
      {
         if (tree.getValidNodes().size() == 0)
         {
            if (VERBOSE)
               PrintTools.info("Did not find a single valid root node.");
            setOutputStatus(toolboxSolution, 1);
            terminateToolboxController();
         }
         else
         {
            if (VERBOSE)
               PrintTools.info("Successfully finished initial guess stage. " + currentNumberOfInitialGuesses.getIntegerValue() + " "
                     + currentNumberOfValidInitialGuesses.getIntegerValue());
            state.set(CWBToolboxState.EXPAND_TREE);
         }
      }
      treeExpansionStartTime = updateTimer(initialGuessComputationTime, initialGuessStartTime);
   }

   private long updateTimer(YoDouble currentTimer, long currentTimerStartTime)
   {
      long endTime = System.nanoTime();
      currentTimer.set(Conversions.nanosecondsToSeconds(endTime - currentTimerStartTime));
      return endTime;
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);

      /*
       * bring control parameters from request.
       */
      boolean success = updateConfiguration();
      if (!success)
      {
         return false;
      }

      if (!commandInputManager.isNewCommandAvailable(RigidBodyExplorationConfigurationCommand.class))
      {
         return false;
      }

      rigidBodyCommands = commandInputManager.pollNewCommands(RigidBodyExplorationConfigurationCommand.class);

      // ******************************************************************************** //
      // Convert command into WholeBodyTrajectoryToolboxData.
      // ******************************************************************************** //

      if (VERBOSE)
         PrintTools.info("initialize CWB toolbox");

      configurationConverter.updateFullRobotModel(initialConfiguration);

      toolboxData = new WholeBodyTrajectoryToolboxData(configurationConverter.getFullRobotModel(), trajectoryCommands, manifoldCommands, rigidBodyCommands);

      bestScoreInitialGuess.set(0.0);

      initialGuessStartTime = System.nanoTime();

      initialGuessComputationTime.setToNaN();
      treeExpansionComputationTime.setToNaN();
      shortcutPathComputationTime.setToNaN();
      motionGenerationComputationTime.setToNaN();

      numberOfValidPosture = 0;
      numberOfInvalidPosture = 0;

      rootNode = null;
      nodePlotter = new SpatialNodePlotter(toolboxData, visualize);

      // Initiate tree.
      // TODO : findInitialGuessSub()
      tree = new SpatialNodeTree();

      if (trajectoryCommands != null)
      {
         state.set(CWBToolboxState.FIND_INITIAL_GUESS);
      }
      else if (manifoldCommands != null)
      {
         state.set(CWBToolboxState.EXPAND_TREE);
         SpatialData rootData = toolboxData.createRandomSpatialData();
         SpatialNode rootNode = new SpatialNode(rootData);
         rootNode.setConfiguration(initialConfiguration);
         rootNode.initializeSpatialData();

         nodePlotter.update(rootNode, 1);
         tree.addInitialNode(rootNode);

         treeExpansionStartTime = updateTimer(initialGuessComputationTime, initialGuessStartTime);
      }
      else
      {
         return false;
      }

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

         trajectoryCommands = null;
         manifoldCommands = null;
         if (commandInputManager.isNewCommandAvailable(WaypointBasedTrajectoryCommand.class))
         {
            trajectoryCommands = commandInputManager.pollNewCommands(WaypointBasedTrajectoryCommand.class);
            if (trajectoryCommands.size() < 1)
               return false;
         }
         else if (commandInputManager.isNewCommandAvailable(ReachingManifoldCommand.class))
         {
            manifoldCommands = commandInputManager.pollNewCommands(ReachingManifoldCommand.class);
            if (manifoldCommands.size() < 1)
               return false;
         }
         else
         {
            if (VERBOSE)
               PrintTools.info("wrong type received");
            return false;
         }

         newMaxExpansionSize = command.getMaximumExpansionSize();
         newNumberOfInitialGuesses = command.getNumberOfInitialGuesses();

         if (command.hasInitialConfiguration())
         {
            newInitialConfiguration = command.getInitialConfiguration();
         }
      }

      if (newMaxExpansionSize > 0)
      {
         maximumExpansionSize.set(newMaxExpansionSize);
      }
      else
      {
         maximumExpansionSize.set(DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE);
      }

      if (newNumberOfInitialGuesses > 0)
      {
         desiredNumberOfInitialGuesses.set(newNumberOfInitialGuesses);
      }
      else
      {
         desiredNumberOfInitialGuesses.set(DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE);
      }

      if (newInitialConfiguration != null)
      {
         initialConfiguration.set(newInitialConfiguration);
         return true;
      }

      RobotConfigurationData currentRobotConfiguration = currentRobotConfigurationDataReference.getAndSet(null);
      if (currentRobotConfiguration == null)
      {
         return false;
      }

      initialConfiguration.desiredRootOrientation.set(currentRobotConfiguration.getRootOrientation());
      initialConfiguration.desiredRootTranslation.set(currentRobotConfiguration.getRootTranslation());

      initialConfiguration.jointNameHash = currentRobotConfiguration.jointNameHash;
      int length = currentRobotConfiguration.jointAngles.size();
      MessageTools.copyData(currentRobotConfiguration.jointAngles, initialConfiguration.desiredJointAngles);

      PrintTools.info("update config done");
      return true;
   }

   private void terminateToolboxController()
   {
      toolboxSolution.setDestination(PacketDestination.BEHAVIOR_MODULE);

      reportMessage(toolboxSolution);

      //nodePlotter.closeAll();
      state.set(CWBToolboxState.DO_NOTHING);

      double totalTime = 0.0;
      if (!initialGuessComputationTime.isNaN())
         totalTime += initialGuessComputationTime.getDoubleValue();
      if (!treeExpansionComputationTime.isNaN())
         totalTime += treeExpansionComputationTime.getDoubleValue();
      if (!shortcutPathComputationTime.isNaN())
         totalTime += shortcutPathComputationTime.getDoubleValue();
      if (!motionGenerationComputationTime.isNaN())
         totalTime += motionGenerationComputationTime.getDoubleValue();
      totalComputationTime.set(totalTime);

      if (VERBOSE)
      {
         PrintTools.info("===========================================");
         PrintTools.info("initialGuessComputationTime is " + initialGuessComputationTime.getDoubleValue());
         PrintTools.info("treeExpansionComputationTime is " + treeExpansionComputationTime.getDoubleValue());
         PrintTools.info("shortcutPathComputationTime is " + shortcutPathComputationTime.getDoubleValue());
         PrintTools.info("motionGenerationComputationTime is " + motionGenerationComputationTime.getDoubleValue());
         PrintTools.info("toolbox executing time is " + totalComputationTime.getDoubleValue() + " seconds " + currentNumberOfIterations.getIntegerValue());
         PrintTools.info("===========================================");
      }

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
   private boolean updateValidity(SpatialNode node)
   {
      if (node.getParent() != null && node.getParent().getConfiguration() != null)
      {
         humanoidKinematicsSolver.setInitialConfiguration(node.getParent().getConfiguration());
      }
      else
      {
         humanoidKinematicsSolver.setInitialConfiguration(initialConfiguration);
      }

      humanoidKinematicsSolver.initialize();
      humanoidKinematicsSolver.submit(toolboxData.createMessages(node));
      /*
       * result
       */
      boolean success = humanoidKinematicsSolver.solve();

      node.setConfiguration(humanoidKinematicsSolver.getSolution());
      node.setValidity(success);

      return success;
   }

   /**
    * set fullRobotModel.
    */
   private void updateVisualizerRobotConfiguration(KinematicsToolboxOutputStatus robotKinematicsConfiguration)
   {
      MessageTools.unpackDesiredJointState(robotKinematicsConfiguration, visualizedFullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   private void updateVisualizerRobotConfiguration()
   {
      MessageTools.unpackDesiredJointState(visualizedNode.getConfiguration(), visualizedFullRobotModel.getRootJoint(), FullRobotModelUtils.getAllJointsExcludingHands(visualizedFullRobotModel));
   }

   /**
    * update visualizers.
    */
   private void updateVisualizers()
   {
      currentTrajectoryTime.set(visualizedNode.getTime());
      isValidNode.set(visualizedNode.isValid());

      if (visualize && visualizedNode != null)
      {
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getTime() / toolboxData.getTrajectoryTime());
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.isValid());
         treeStateVisualizer.updateVisualizer();

         //FIXME TODO
         //         treeVisualizer.update(visualizedNode);
      }
   }

   /**
    * YoVariables.
    */
   private void updateYoVariables()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         endeffectorFrame.get(robotSide).setVisible(true);
         endeffectorFrame.get(robotSide).update();
      }
   }

   /**
    * oneTime shortcut : try to make a shortcut from index to index+2
    */
   private boolean updateShortcutPath(List<SpatialNode> path, int index)
   {
      // check out when index is over the size.
      if (index > path.size() - 3)
      {
         return false;
      }

      SpatialNode nodeDummy = new SpatialNode(path.get(index + 1));

      nodeDummy.setParent(path.get(index));
      nodeDummy.interpolate(path.get(index), path.get(index + 2), 0.5);

      updateValidity(nodeDummy);

      if (nodeDummy.isValid())
      {
         path.get(index + 1).interpolate(path.get(index), path.get(index + 2), 0.5);
         path.get(index + 1).setConfiguration(nodeDummy.getConfiguration());

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * return distance of the paths before and after shortcut.
    */
   private double updateShortcutPath(List<SpatialNode> path)
   {
      ArrayList<SpatialNode> pathBeforeShortcut = new ArrayList<SpatialNode>();

      for (int i = 0; i < path.size(); i++)
      {
         pathBeforeShortcut.add(new SpatialNode(path.get(i)));
      }

      for (int i = 0; i < path.size(); i++)
      {
         if (updateShortcutPath(path, i))
            ;
      }

      double distance = 0.0;
      for (int i = 0; i < path.size(); i++)
      {
         distance += pathBeforeShortcut.get(i).computeDistance(0.0, tree.getPositionWeight(), tree.getOrientationWeight(), path.get(i));
      }

      return distance / path.size();
   }

   private double computeArmJointsLimitScore(FullHumanoidRobotModel fullRobotModel)
   {
      double score = 0.0;
      RigidBody chest = fullRobotModel.getChest();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         score += WholeBodyTrajectoryToolboxHelper.kinematicsChainLimitScore(chest, hand);
      }
      return score;
   }

   void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      currentRobotConfigurationDataReference.set(newConfigurationData);
   }

   FullHumanoidRobotModel getSolverFullRobotModel()
   {
      return humanoidKinematicsSolver.getDesiredFullRobotModel();
   }
}