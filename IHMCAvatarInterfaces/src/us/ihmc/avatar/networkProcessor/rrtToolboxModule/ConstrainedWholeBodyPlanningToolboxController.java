package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedEndEffectorTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConstrainedWholeBodyPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.TreeStateVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ConstrainedWholeBodyPlanningToolboxController extends ToolboxController
{
   private static int terminateToolboxCondition = 10;
   
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

   /*
    * Visualizer
    */
   private boolean startYoVariableServer;

   private CTTaskNode visualizedNode;

   private FullHumanoidRobotModel initialRobotModel;
   
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

   public ConstrainedWholeBodyPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, StatusMessageOutputManager statusOutputManager,
                                                        YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsRegistry, boolean startYoVariableServer)
   {
      super(statusOutputManager, registry);
      this.drcRobotModelFactory = drcRobotModel;      
      this.visualizedFullRobotModel = fullRobotModel;
      this.isDone.set(false);

      this.startYoVariableServer = startYoVariableServer;
      this.treeStateVisualizer = new TreeStateVisualizer("TreeStateVisualizer", "VisualizerGraphicsList", yoGraphicsRegistry, registry);
      this.state = CWBToolboxState.DO_NOTHING;
      
      this.endeffectorPose = new YoFramePose("endeffectorFramePose", ReferenceFrame.getWorldFrame(), registry);
      this.endeffectorFrame = new YoGraphicCoordinateSystem("endeffectorPose", this.endeffectorPose, 0.5);
      this.endeffectorFrame.setVisible(true);
      
      yoGraphicsRegistry.registerYoGraphic("endeffectorPoseViz", this.endeffectorFrame);
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
//         visualizedNode = new GenericTaskNode(0.02, 0.8, Math.PI*20/180, Math.PI*10/180, Math.PI*5/180);
         
         visualizedNode = new GenericTaskNode(0.0, 0.75, -Math.PI*20/180, Math.PI*10/180, Math.PI*5/180);
         
         PrintTools.info(""+isValidNode(visualizedNode));
//         visualizedFullRobotModel = initialRobotModel;
         
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
      FullHumanoidRobotModel solverRobotModel = kinematicsSolver.getFullRobotModelCopy();
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
      
      isGoodkinematicSolution.set(kinematicsSolver.getIsSolved());
      solutionQuality.set(kinematicsSolver.getSolution().getSolutionQuality());
      endeffectorFrame.update();

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

//      constrainedEndEffectorTrajectory = request.constrainedEndEffectorTrajectory;
            
      kinematicsSolver = new WheneverWholeBodyKinematicsSolver(drcRobotModelFactory, visualizedFullRobotModel);
      
      kinematicsSolver.updateRobotConfigurationData(request.initialOneDoFJoints, request.initialTranslationOfRootJoint, request.initialRotationOfRootJoint);
      
      kinematicsSolver.initialize();
      kinematicsSolver.holdCurrentTrajectoryMessages();
      kinematicsSolver.putTrajectoryMessages();
      kinematicsSolver.isSolved();
      
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(kinematicsSolver.getDesiredFullRobotModel());
      
      referenceFrames.updateFrames();
      midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();      
      worldFrame = referenceFrames.getWorldFrame();
      
      initialRobotModel = kinematicsSolver.getFullRobotModelCopy();
            
      /*
       * start toolbox
       */
      state = CWBToolboxState.FIND_INITIAL_GUESS;
      
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
   
   public static double handCoordinateOffsetX = -0.2;
   
   private boolean isValidNode(CTTaskNode node)
   {
      if (node.getParentNode() != null)
      {
         kinematicsSolver.updateRobotConfigurationDataJointsOnly(node.getParentNode().getOneDoFJoints());
         for (int i = 0; i < node.getParentNode().getOneDoFJoints().length; i++)
         {
            double jointPosition = node.getParentNode().getOneDoFJoints()[i].getQ();
         }
      }
      else
      {
         PrintTools.warn("parentNode is required.");
         //kinematicsSolver.updateRobotConfigurationDataJointsOnly(FullRobotModelUtils.getAllJointsExcludingHands(initialRobotModel));         
         kinematicsSolver.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(initialRobotModel), new Vector3D(initialRobotModel.getRootJoint().getTranslationForReading())
                                                       , new Quaternion(initialRobotModel.getRootJoint().getRotationForReading()));
      }

      kinematicsSolver.initialize();

      kinematicsSolver.holdCurrentTrajectoryMessages();
      /*
       * set whole body tasks.
       */      
      ConfigurationSpace configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(node.getNodeData(5), node.getNodeData(6), node.getNodeData(7));
      configurationSpace.setRotation(node.getNodeData(8), node.getNodeData(9), node.getNodeData(10));
      
      Pose3D desiredPose = constrainedEndEffectorTrajectory.getEndEffectorPose(node.getNodeData(0), configurationSpace);
//      desiredPose.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);
      
      FramePoint desiredPointToWorld = new FramePoint(worldFrame, desiredPose.getPosition());
      FrameOrientation desiredOrientationToWorld = new FrameOrientation(worldFrame, desiredPose.getOrientation());

      FramePose desiredPoseToWorld = new FramePose(desiredPointToWorld, desiredOrientationToWorld);
     
      desiredPoseToWorld.changeFrame(midZUpFrame);

      Pose3D desiredPoseToMidZUp = new Pose3D(new Point3D(desiredPoseToWorld.getPosition()), new Quaternion(desiredPoseToWorld.getOrientation()));
      desiredPoseToMidZUp.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);      
      
      kinematicsSolver.setDesiredHandPose(constrainedEndEffectorTrajectory.getRobotSide(), desiredPoseToMidZUp);
//      kinematicsSolver.setHandSelectionMatrixFree(constrainedEndEffectorTrajectory.getAnotherRobotSide());
      
      

      desiredPoseToWorld.changeFrame(worldFrame);
      endeffectorPose.set(desiredPoseToWorld);
      endeffectorFrame.setPose(desiredPoseToWorld);
      System.out.println(desiredPoseToWorld);
      

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(node.getNodeData(2));

      desiredChestOrientation.appendPitchRotation(node.getNodeData(3));
      desiredChestOrientation.appendRollRotation(node.getNodeData(4));
      kinematicsSolver.setDesiredChestOrientation(desiredChestOrientation);

      kinematicsSolver.setDesiredPelvisHeight(node.getNodeData(1));

      kinematicsSolver.putTrajectoryMessages();

      boolean result = kinematicsSolver.isSolved();
      
      PrintTools.info(""+result);

      node.setConfigurationJoints(kinematicsSolver.getFullRobotModelCopy());
            
      return result;
   }

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
//   private void treeInitialize()
//   {
//      double initialPelvisHeight = CTTaskNode.initialRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
//
//      rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
//      rootNode.setNodeData(2, -10.0 / 180 * Math.PI);
//      rootNode.setNodeData(10, -30.0 / 180 * Math.PI);
//      rootNode.setConfigurationJoints(GenericTaskNode.initialRobotModel);
//
//      PrintTools.info("initial node is " + rootNode.isValidNode());
//
//      tree = new CTTaskNodeTree(rootNode);
//
//      rootNode.convertDataToNormalizedData(CTTaskNode.constrainedEndEffectorTrajectory.getTaskNodeRegion());
//
//      if (startYoVariableServer)
//      {
//         treeVisualizer = new CTTreeVisualizer(tree);
//         treeVisualizer.initialize();
//      }
//   }
}
