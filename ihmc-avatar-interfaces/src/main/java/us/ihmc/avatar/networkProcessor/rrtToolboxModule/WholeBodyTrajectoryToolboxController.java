package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeTools;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationBuildOrder;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationSpace;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConstrainedEndEffectorTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TaskRegion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyTrajectoryToolboxController extends ToolboxController
{
   private static final boolean VERBOSE = false;
   private static final int DEFAULT_NUMBER_OF_ITERATIONS_FOR_SHORTCUT_OPTIMIZATION = 5;
   private static final int DEFAULT_MAXIMUM_NUMBER_OF_ITERATIONS = 1000;
   private static final int DEFAULT_MAXIMUM_EXPANSION_SIZE_VALUE = 1000;
   private static final int DEFAULT_NUMBER_OF_INITIAL_GUESSES_VALUE = 50;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private ConstrainedEndEffectorTrajectory constrainedEndEffectorTrajectory;

   private final HumanoidKinematicsSolver humanoidKinematicsSolver;

   private final WholeBodyTrajectoryToolboxOutputStatus toolboxSolution;

   /*
    * YoVariables
    */
   private final YoInteger maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", registry);
   private final YoInteger currentNumberOfIterations = new YoInteger("currentNumberOfIterations", registry);

   // check the tree reaching the normalized time from 0.0 to 1.0.
   private final YoDouble currentTrajectoryTime = new YoDouble("currentNormalizedTime", registry);

   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final YoDouble jointlimitScore = new YoDouble("jointlimitScore", registry);

   private final YoDouble bestScoreInitialGuess = new YoDouble("bestScoreInitialGuess", registry);

   /*
    * Visualizer
    */
   private boolean visualize;

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

   private final YoInteger currentExpansionSize = new YoInteger("currentExpansionSize", registry);
   private final YoInteger maximumExpansionSize = new YoInteger("maximumExpansionSize", registry);

   private final YoInteger currentNumberOfInitialGuesses = new YoInteger("currentNumberOfInitialGuesses", registry);
   private final YoInteger desiredNumberOfInitialGuesses = new YoInteger("desiredNumberOfInitialGuesses", registry);

   private int numberOfMotionPath = 1;

   private YoInteger numberOfIterationForShortcutOptimization = new YoInteger("numberOfIterationForShortcutOptimization", registry);

   /**
    * Toolbox state
    */
   private final YoEnum<CWBToolboxState> state = new YoEnum<>("", registry, CWBToolboxState.class);

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

      humanoidKinematicsSolver = new HumanoidKinematicsSolver(drcRobotModel, yoGraphicsListRegistry, registry);

      toolboxSolution = new WholeBodyTrajectoryToolboxOutputStatus();
      toolboxSolution.setDestination(-1);
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

      long endTime = System.nanoTime();
      motionGenerationComputationTime.set(Conversions.nanosecondsToSeconds(endTime - motionGenerationStartTime));
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
         if (VERBOSE)
            PrintTools.info("Planning has Failed.");
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
         {
            revertedPath.add(new CTTaskNode(currentNode));
         }
         else
         {
            break;
         }
      }

      int revertedPathSize = revertedPath.size();

      tree.addNodeOnPath(new CTTaskNode(revertedPath.get(revertedPathSize - 1)));
      int currentIndex = 0;
      for (int j = 0; j < revertedPathSize - 2; j++)
      {
         int i = revertedPathSize - 1 - j;

         double generalizedTimeGap = revertedPath.get(i - 1).getNormalizedNodeData(0) - tree.getPath().get(currentIndex).getNormalizedNodeData(0);

         if (generalizedTimeGap > CTTaskNodeTree.dismissibleTimeGap)
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

      if (visualize)
         treeVisualizer.update(tree.getPath());

      for (int i = 0; i < numberOfIterationForShortcutOptimization.getIntegerValue(); i++)
      {
         updateShortcutPath(tree.getPath());
      }
      long endTime = System.nanoTime();

      shortcutPathComputationTime.set(Conversions.nanosecondsToSeconds(endTime - shortcutStartTime));
      motionGenerationStartTime = endTime;

      if (VERBOSE)
      {
         PrintTools.info("Shortcut computation time = " + shortcutPathComputationTime.getDoubleValue());
         PrintTools.info("the size of the path is " + tree.getPath().size() + " before dismissing " + revertedPathSize);
      }

      numberOfMotionPath = tree.getPath().size();

      if (visualize)
      {
         treeVisualizer.showUpPath(tree.getPath());
      }

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
            if (VERBOSE)
               PrintTools.info("Successfully finished tree expansion.");
            currentExpansionSize.set(maximumExpansionSize.getIntegerValue()); // for terminate
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

      double jointScore = computeArmJointsLimitScore(humanoidKinematicsSolver.getDesiredFullRobotModel());

      jointlimitScore.set(jointScore);

      /*
       * terminate expanding tree.
       */

      if (currentExpansionSize.getIntegerValue() >= maximumExpansionSize.getIntegerValue())
      {
         if (tree.getTreeReachingTime() != 1.0)
         {
            if (VERBOSE)
               PrintTools.info("Failed to complete trajectory.");

            terminateToolboxController();
         }
         else
         {
            state.set(CWBToolboxState.SHORTCUT_PATH);
            long endTime = System.nanoTime();
            treeExpansionComputationTime.set(Conversions.nanosecondsToSeconds(endTime - treeExpansionStartTime));
            shortcutStartTime = endTime;
         }
      }
   }

   /**
    * state == FIND_INITIAL_GUESS
    *
    * @throws ExecutionException
    * @throws InterruptedException
    */
   private void findInitialGuess()
   {
      CTTaskNode initialGuessNode = new GenericTaskNode();

      CTTreeTools.setRandomNormalizedNodeData(initialGuessNode, true, 0.0);
      initialGuessNode.setNormalizedNodeData(0, 0);
      initialGuessNode.convertNormalizedDataToData(taskRegion);

      updateValidity(initialGuessNode);

      visualizedNode = initialGuessNode;

      double jointScore = computeArmJointsLimitScore(humanoidKinematicsSolver.getDesiredFullRobotModel());

      jointlimitScore.set(jointScore);

      if (bestScoreInitialGuess.getDoubleValue() < jointScore && visualizedNode.getValidity() == true)
      {
         bestScoreInitialGuess.set(jointScore);

         rootNode = new CTTaskNode(visualizedNode);
         rootNode.setValidity(visualizedNode.getValidity());
      }

      /*
       * terminate finding initial guess.
       */
      currentNumberOfInitialGuesses.increment();

      if (currentNumberOfInitialGuesses.getIntegerValue() >= desiredNumberOfInitialGuesses.getIntegerValue())
      {
         if (VERBOSE)
            PrintTools.info("Successfully finished initial guess stage.");

         if (rootNode.getValidity() == false)
         {
            terminateToolboxController();
         }
         else
         {
            state.set(CWBToolboxState.EXPAND_TREE);

            rootNode.convertDataToNormalizedData(taskRegion);

            tree = new CTTaskNodeTree(rootNode);
            tree.setTaskRegion(taskRegion);

            long endTime = System.nanoTime();
            initialGuessComputationTime.set(Conversions.nanosecondsToSeconds(endTime - initialGuessStartTime));
            treeExpansionStartTime = endTime;
         }
      }
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      if (!commandInputManager.isNewCommandAvailable(WaypointBasedTrajectoryCommand.class))
      {
         return false;
      }

      List<WaypointBasedTrajectoryCommand> trajectoryCommands = commandInputManager.pollNewCommands(WaypointBasedTrajectoryCommand.class);
      if(trajectoryCommands.size() < 1)
         return false;

      List<RigidBodyExplorationConfigurationCommand> rigidBodyCommands = commandInputManager.pollNewCommands(RigidBodyExplorationConfigurationCommand.class);
      
      WholeBodyTrajectoryToolboxConfigurationCommand configurationCommand = commandInputManager.pollNewestCommand(WholeBodyTrajectoryToolboxConfigurationCommand.class);
            
      PrintTools.info("received ! WaypointBasedTrajectoryCommand");
      System.out.println(trajectoryCommands.size());
      for(int i=0;i<trajectoryCommands.size();i++)
      {
         System.out.println(trajectoryCommands.get(i).getEndEffector().getName());
         int numberOfUnconstrainedDegreesOfFreedom = trajectoryCommands.get(i).getNumberOfUnconstrainedDegreesOfFreedom();
         for(int j=0;j<numberOfUnconstrainedDegreesOfFreedom;j++)
            System.out.println(trajectoryCommands.get(i).getUnconstrainedDegreeOfFreedom(j));         
      }
      
      PrintTools.info("received ! RigidBodyExplorationConfigurationCommand");      
      System.out.println(rigidBodyCommands.size());
      for(int i=0;i<rigidBodyCommands.size();i++)
      {
         System.out.println(rigidBodyCommands.get(i).getRigidBody().getName());
         int numberOfDegreesOfFreedomToExplore = rigidBodyCommands.get(i).getNumberOfDegreesOfFreedomToExplore();
         for(int j=0;j<numberOfDegreesOfFreedomToExplore;j++)
         {
            System.out.println(rigidBodyCommands.get(i).getDegreeOfFreedomToExplore(j));
            // System.out.println(rigidBodyCommands.get(i).getExplorationUpperLimit(j));
            // System.out.println(rigidBodyCommands.get(i).getExplorationLowerLimit(j));
         }   
      }
      
      PrintTools.info("received ! WholeBodyTrajectoryToolboxConfigurationCommand");
      System.out.println(configurationCommand.getMaximumExpansionSize());
      
      
      // ******************************************************************************** //
      // Convert command into WholeBodyTrajectoryToolboxData.
      // ******************************************************************************** //
      
      
      WholeBodyTrajectoryToolboxData toolboxData = new WholeBodyTrajectoryToolboxData(this.visualizedFullRobotModel, configurationCommand, trajectoryCommands, rigidBodyCommands);
      
      PrintTools.info("initialize success");
      
      
      
      
      
      
      // ******************************************************************************** //
      // Convert WholeBodyTrajectoryToolboxData into constrainedEndEffectorTrajectory.
      // ******************************************************************************** //      
      //constrainedEndEffectorTrajectory = convertCommands(trajectoryCommands);

      if (VERBOSE)
         PrintTools.info("initialize CWB toolbox");

      /*
       * bring control parameters from request.
       */
      boolean success = updateConfiguration();
      if (!success)
      {
         return false;
      }

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
      if (visualize)
      {
         treeVisualizer = new CTTreeVisualizer(tree);
         treeVisualizer.initialize();
      }

      bestScoreInitialGuess.set(0.0);

      state.set(CWBToolboxState.FIND_INITIAL_GUESS);
      initialGuessStartTime = System.nanoTime();

      initialGuessComputationTime.setToNaN();
      treeExpansionComputationTime.setToNaN();
      shortcutPathComputationTime.setToNaN();
      motionGenerationComputationTime.setToNaN();

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
            taskNodeRegion.setRandomRegion(8, -Math.PI, Math.PI);
            taskNodeRegion.setRandomRegion(9, -Math.PI, Math.PI);
            taskNodeRegion.setRandomRegion(10, -Math.PI, Math.PI);

            taskNodeRegion.setRandomRegion(11, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(12, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(13, 0.0, 0.0);
            taskNodeRegion.setRandomRegion(14, -Math.PI, Math.PI);
            taskNodeRegion.setRandomRegion(15, -Math.PI, Math.PI);
            taskNodeRegion.setRandomRegion(16, -Math.PI, Math.PI);

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
            {
               configurationBuildOrders.put(robotSide,
                                            new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                                        ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL));
            }

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
                  {
                     break;
                  }
               }

               double alpha = (time - t0) / (tf - t0);
               alpha = MathTools.clamp(alpha, 0, 1);
               current.interpolate(previous, next, alpha);

               double x = current.getX();
               double y = current.getY();
               double z = current.getZ();
               RotationMatrix rot = new RotationMatrix(current.getOrientation());
               double roll = rot.getRoll();
               double pitch = rot.getPitch();
               double yaw = rot.getYaw();
               configurationSpace.put(robotSide, new ConfigurationSpace(x, y, z, roll, pitch, yaw));
            }

            return configurationSpace;
         }
      };
      return endEffectorTrajectory;
   }

   private void terminateToolboxController()
   {
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
   private boolean updateValidity(CTTaskNode node)
   {
      if (node.getParentNode() != null && node.getParentNode().getConfiguration() != null)
      {
         humanoidKinematicsSolver.setInitialConfiguration(node.getParentNode().getConfiguration());
      }
      else
      {
         humanoidKinematicsSolver.setInitialConfiguration(initialConfiguration);
      }

      humanoidKinematicsSolver.initialize();

      /*
       * set whole body tasks. pose from 'constrainedEndEffectorTrajectory' is considered as being
       * in MidZUpframe. for kinematicsSolver, append offset
       */
      SideDependentList<ConfigurationSpace> configurationSpaces = new SideDependentList<>();
      FullHumanoidRobotModel desiredFullRobotModel = humanoidKinematicsSolver.getDesiredFullRobotModel();
      desiredFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      referenceFrames.updateFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();

      for (RobotSide robotSide : RobotSide.values)
      {
         configurationSpaces.put(robotSide, CTTreeTools.getConfigurationSpace(node, robotSide));

         Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(node.getNodeData(0), robotSide, configurationSpaces.get(robotSide));
         setEndEffectorPose(robotSide, desiredPose);

         humanoidKinematicsSolver.submit(createHandMessage(robotSide, new FramePose(midFootZUpGroundFrame, desiredPose)));
      }

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.setYawPitchRoll(node.getNodeData(2), node.getNodeData(3), node.getNodeData(4));
      humanoidKinematicsSolver.submit(createChestMessage(new FrameOrientation(midFootZUpGroundFrame, desiredChestOrientation)));

      humanoidKinematicsSolver.submit(createPelvisMessage(new FramePoint3D(midFootZUpGroundFrame, 0.0, 0.0, node.getNodeData(1))));

      /*
       * result
       */
      boolean success = humanoidKinematicsSolver.solve();

      node.setConfigurationJoints(humanoidKinematicsSolver.getSolution());

      node.setValidity(success);

      return success;
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
      currentTrajectoryTime.set(visualizedNode.getNormalizedNodeData(0));

      if (visualize && visualizedNode != null)
      {
         treeStateVisualizer.setCurrentNormalizedTime(visualizedNode.getNormalizedNodeData(0));
         treeStateVisualizer.setCurrentCTTaskNodeValidity(visualizedNode.getValidity());
         treeStateVisualizer.updateVisualizer();

         treeVisualizer.update(visualizedNode);
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
         if (VERBOSE)
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

   private final double handWeight = 50.0;
   private final double chestWeight = 10.0;
   private final double pelvisWeight = 10.0;

   private KinematicsToolboxRigidBodyMessage createHandMessage(RobotSide robotSide, FramePose desiredPose)
   {
      RigidBody hand = humanoidKinematicsSolver.getDesiredFullRobotModel().getHand(robotSide);

      KinematicsToolboxRigidBodyMessage message;
      if (desiredPose == null)
      {
         message = KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(hand);
      }
      else
      {
         desiredPose = new FramePose(desiredPose);
         desiredPose.changeFrame(worldFrame);
         message = new KinematicsToolboxRigidBodyMessage(hand);
         message.setDesiredPose(desiredPose);
      }

      message.setWeight(handWeight);

      return message;
   }

   private KinematicsToolboxRigidBodyMessage createPelvisMessage(FramePoint3D desiredHeight)
   {
      desiredHeight = new FramePoint3D(desiredHeight);
      desiredHeight.changeFrame(worldFrame);

      RigidBody pelvis = humanoidKinematicsSolver.getDesiredFullRobotModel().getPelvis();
      RigidBodyTransform pelvisTransform = pelvis.getParentJoint().getFrameAfterJoint().getTransformToWorldFrame();
      Quaternion desiredOrientation = new Quaternion(pelvisTransform.getRotationMatrix());
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(pelvis, desiredHeight, desiredOrientation);
      message.setWeight(pelvisWeight);
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
      selectionMatrix.setLinearAxisSelection(false, false, true);
      message.setSelectionMatrix(selectionMatrix);
      return message;
   }

   private KinematicsToolboxRigidBodyMessage createChestMessage(FrameOrientation desiredOrientation)
   {
      desiredOrientation = new FrameOrientation(desiredOrientation);
      desiredOrientation.changeFrame(worldFrame);

      RigidBody chest = humanoidKinematicsSolver.getDesiredFullRobotModel().getChest();
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(chest);
      message.setDesiredOrientation(desiredOrientation);
      message.setSelectionMatrixForAngularControl();
      message.setWeight(chestWeight);

      return message;
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