package us.ihmc.humanoidBehaviors.behaviors.scripts;

import java.io.InputStream;
import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.EndEffectorLoadBearingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HighLevelStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisHeightTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PrimitiveBehaviorType;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngine;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateWrapper;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.script.ScriptBehaviorStatusPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateChangedListener;
import us.ihmc.robotics.stateMachines.StateMachineTools;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionAction;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.tools.io.printing.PrintTools;

public class ScriptBehavior extends AbstractBehavior
{
   private static final boolean DEBUG = false;

   private final BooleanYoVariable scriptImported = new BooleanYoVariable("scriptImported", registry);
   private final BooleanYoVariable scriptFinished = new BooleanYoVariable("scriptFinished", registry);

   private final ScriptEngine scriptEngine;
   private RigidBodyTransform behaviorOriginTransformToWorld = null;
   private ArrayList<ScriptObject> childInputPackets = null;
   private ScriptBehaviorStatusEnum scriptStatus;
   private int scriptIndex = 0;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;

   private final BehaviorStateMachine<PrimitiveBehaviorType> stateMachine;
   private final EnumYoVariable<PrimitiveBehaviorType> requestedState = new EnumYoVariable<>("requestedScriptBehaviorState", registry,
         PrimitiveBehaviorType.class, true);

   private final FootstepListBehavior footstepListBehavior;
   private final HandTrajectoryBehavior handTrajectoryBehavior;
   private final EndEffectorLoadBearingBehavior endEffectorLoadBearingBehavior;
   private final HeadTrajectoryBehavior headTrajectoryBehavior;
   private final PelvisHeightTrajectoryBehavior pelvisHeightTrajectoryBehavior;
   private final FootTrajectoryBehavior footTrajectoryBehavior;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;
   private final ChestTrajectoryBehavior chestTrajectoryBehavior;
   private final HighLevelStateBehavior highLevelStateBehavior;
   public final HandDesiredConfigurationBehavior handDesiredConfigurationBehavior;

   public ScriptBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      this(outgoingCommunicationBridge, fullRobotModel, yoTime, null, null);

      PrintTools.debug(this, "Warning: FootPosePackets and FootstepDataList packets are not supported when using this constructor!");
   }

   public ScriptBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime,
         BooleanYoVariable doubleSupport, WalkingControllerParameters walkingControllerParameters)
   {
      super(outgoingCommunicationBridge);

      scriptEngine = new ScriptEngine(null);

      if (walkingControllerParameters != null)
      {
         footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge, walkingControllerParameters);
      }
      else
      {
         footstepListBehavior = null;
      }
      handTrajectoryBehavior = new HandTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      endEffectorLoadBearingBehavior = new EndEffectorLoadBearingBehavior(outgoingCommunicationBridge);
      headTrajectoryBehavior = new HeadTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      pelvisHeightTrajectoryBehavior = new PelvisHeightTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      if (doubleSupport != null)
      {
         footTrajectoryBehavior = new FootTrajectoryBehavior(outgoingCommunicationBridge, yoTime, doubleSupport);

      }
      else
      {
         footTrajectoryBehavior = null;
      }
      pelvisTrajectoryBehavior = new PelvisTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      chestTrajectoryBehavior = new ChestTrajectoryBehavior(outgoingCommunicationBridge, yoTime);
      highLevelStateBehavior = new HighLevelStateBehavior(outgoingCommunicationBridge);
      handDesiredConfigurationBehavior = new HandDesiredConfigurationBehavior(outgoingCommunicationBridge, yoTime);
      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);

      stateMachine = new BehaviorStateMachine<>("scriptBehaviorState", "scriptBehaviorSwitchTime", PrimitiveBehaviorType.class, yoTime, registry);
      addBehaviorsToStateMachine(stateMachine);

      if (DEBUG)
      {
         stateMachine.attachStateChangedListener(new StateChangedListener<PrimitiveBehaviorType>()
         {
            @Override
            public void stateChanged(State<PrimitiveBehaviorType> oldState, State<PrimitiveBehaviorType> newState, double time)
            {
               PrintTools.debug(this, "Switching from: " + oldState.getStateEnum() + " to: " + newState.getStateEnum() + " at t = " + time);
            }
         });
      }
      requestedState.set(null);
   }

   private void addBehaviorsToStateMachine(BehaviorStateMachine<PrimitiveBehaviorType> stateMachine)
   {
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.IDLE, new SimpleDoNothingBehavior(outgoingCommunicationBridge));
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FOOTSTEP_LIST, footstepListBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HAND_TRAJECTORY, handTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.END_EFFECTOR_LOAD_BEARING, endEffectorLoadBearingBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HEAD_ORIENTATION, headTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.PELVIS_HEIGHT, pelvisHeightTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FOOT_POSE, footTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.PELVIS_POSE, pelvisTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.CHEST_TRAJECTORY, chestTrajectoryBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HIGH_LEVEL_STATE, highLevelStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FINGER_STATE, handDesiredConfigurationBehavior);

      stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);
   }

   private void wrapBehaviorAndSetupTransitions(BehaviorStateMachine<PrimitiveBehaviorType> stateMachine, PrimitiveBehaviorType scriptObjectType,
         AbstractBehavior behavior)
   {
      boolean initBehaviorOnStateTransitionIntoAction = false; // FIXME: this is a hack to prevent initialize() from being called *AFTER* input has been set
      final BehaviorStateWrapper<PrimitiveBehaviorType> behaviorState = new BehaviorStateWrapper<PrimitiveBehaviorType>(scriptObjectType, behavior,
            initBehaviorOnStateTransitionIntoAction);

      stateMachine.addState(behaviorState);
      registry.addChild(behavior.getYoVariableRegistry());

      if (scriptObjectType != PrimitiveBehaviorType.IDLE)
      {
         addTransitionToIdleState(stateMachine, behaviorState);
         addTransitionFromIdleToRequestedState(stateMachine, behaviorState);
      }
   }

   private void addTransitionToIdleState(BehaviorStateMachine<PrimitiveBehaviorType> stateMachine,
         final BehaviorStateWrapper<PrimitiveBehaviorType> behaviorState)
   {
      StateTransitionCondition behaviorIsDone = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            boolean isDone = behaviorState.isDone();
            if (isDone)
            {
               PrintTools.debug(this, behaviorState.getBehavior().getName() + " is Done");
            }
            return isDone;
         }
      };

      StateTransitionAction requestNextBehavior = requestNextBehaviorAction(behaviorState);
      StateTransition<PrimitiveBehaviorType> toIdleStateTransition = new StateTransition<>(PrimitiveBehaviorType.IDLE, behaviorIsDone, requestNextBehavior);
      behaviorState.addStateTransition(toIdleStateTransition);
   }

   private StateTransitionAction requestNextBehaviorAction(final BehaviorStateWrapper<PrimitiveBehaviorType> behaviorState)
   {
      StateTransitionAction ret = new StateTransitionAction()
      {
         @Override
         public void doTransitionAction()
         {
            requestNextBehavior(childInputPackets);
            sendScriptStatusPacketToNetworkProcessor(scriptStatus, scriptIndex);
            setChildBehaviorInput(childInputPackets.remove(0));
         }
      };
      return ret;
   }

   private void addTransitionFromIdleToRequestedState(BehaviorStateMachine<PrimitiveBehaviorType> stateMachine,
         final BehaviorStateWrapper<PrimitiveBehaviorType> behaviorState)
   {
      BehaviorStateWrapper<PrimitiveBehaviorType> idleState = stateMachine.getState(PrimitiveBehaviorType.IDLE);
      boolean waitUntilDone = true;

      StateMachineTools.addRequestedStateTransition(requestedState, waitUntilDone, idleState, behaviorState);
   }

   @Override
   public void doControl()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable())
      {
         ScriptBehaviorInputPacket receivedPacket = scriptBehaviorInputPacketListener.poll();
         importScriptInputPacket(receivedPacket);
      }

      if (scriptImported.getBooleanValue() && !scriptFinished.getBooleanValue())
      {
         stateMachine.checkTransitionConditions();
         stateMachine.doAction();
      }
      else
      {
         return;
      }

   }

   public void importScriptInputPacket(ScriptBehaviorInputPacket inputPacket)
   {
      String scriptFileName = inputPacket.getScriptName();
      RigidBodyTransform behaviorOriginTransformToWorld = (inputPacket.getReferenceTransform());

      importScriptFile(scriptFileName, behaviorOriginTransformToWorld);
   }

   public void importScriptFile(String scriptFileName, RigidBodyTransform transformToWorld)
   {
      InputStream scriptResourceStream = getClass().getClassLoader().getResourceAsStream(scriptFileName);

      importChildInputPackets(scriptFileName, scriptResourceStream, transformToWorld);
   }

   public void importChildInputPackets(String scriptFileName, InputStream scriptResourceStream, RigidBodyTransform transformToWorld)
   {
      if (scriptResourceStream != null && transformToWorld != null)
      {
         PrintTools.debug(this, "Starting Importing " + scriptFileName);

         if (childInputPackets == null)
            childInputPackets = new ArrayList<ScriptObject>();

         populateChildBehaviorInputPackets(scriptEngine.getScriptObjects(scriptResourceStream));

         this.behaviorOriginTransformToWorld = transformToWorld;

         PrintTools.debug(this, "Finishing Importing " + scriptFileName);

         scriptIndex = 0;
         scriptStatus = ScriptBehaviorStatusEnum.SCRIPT_LOADED;

         requestNextBehavior(childInputPackets);
         sendScriptStatusPacketToNetworkProcessor(scriptStatus, scriptIndex);
         setChildBehaviorInput(childInputPackets.remove(0));

         scriptImported.set(true);
      }
      else
      {

         PrintTools.debug(this, "Script Resource Stream is null. Can't load script!");
         scriptImported.set(false);
      }
   }

   private void populateChildBehaviorInputPackets(ArrayList<ScriptObject> packetsToAdd)
   {
      int numberOfEndOfScriptCommands = 0;
      for (ScriptObject inputPacket : packetsToAdd)
      {
         PrintTools.debug(this, " Importing child : " + inputPacket.toString() + " to script behavior");

         if (inputPacket.getScriptObject() instanceof EndOfScriptCommand)
         {
            numberOfEndOfScriptCommands++;
         }

         if (isChildPacketValid(inputPacket))
            this.childInputPackets.add(inputPacket);

      }

      if (numberOfEndOfScriptCommands != 1)
      {
         throw new RuntimeException("Number of EndOfScriptCommand() packets = " + numberOfEndOfScriptCommands + "!  Must equal one.");
      }
   }

   private boolean isChildPacketValid(ScriptObject inputPacket)
   {
      boolean ret = true;

      if (footTrajectoryBehavior == null && getPrimitiveBehaviorType(inputPacket) == PrimitiveBehaviorType.FOOT_POSE)
      {
         PrintTools.debug(this, "Must use more elaborate ScriptBehavior constructor in order to import FootPosePackets!");
         return false;
      }

      if (footstepListBehavior == null && getPrimitiveBehaviorType(inputPacket) == PrimitiveBehaviorType.FOOTSTEP_LIST)
      {
         PrintTools.debug(this, "Must use more elaborate ScriptBehavior constructor in order to import FootstepDataList packets!");
         return false;
      }

      return ret;
   }

   private void sendScriptStatusPacketToNetworkProcessor(ScriptBehaviorStatusEnum scriptBehaviorStatusEnum, int scriptIndexToSend)
   {
      //TODO: get rid of "true" when footstep data lists are downlinked
      if (true) //scriptObject == null || !(scriptObject.getScriptObject() instanceof FootstepDataList))
      {
         outgoingCommunicationBridge.sendPacketToNetworkProcessor(new ScriptBehaviorStatusPacket(scriptStatus, scriptIndex));
      }
   }

   private void requestNextBehavior(ArrayList<ScriptObject> behaviorInputPackets)
   {
      if (behaviorInputPackets.size() == 0)
      {
         scriptStatus = ScriptBehaviorStatusEnum.SCRIPT_LOAD_FAILED;
         return;
      }
      else
      {
         ScriptObject inputPacket = childInputPackets.get(0);

         if (inputPacket.getScriptObject() instanceof EndOfScriptCommand)
         {
            scriptFinished.set(true);
            scriptStatus = ScriptBehaviorStatusEnum.FINISHED;

            PrintTools.debug(this, "End of script");
         }
         else
         {
            scriptIndex++;
            scriptStatus = ScriptBehaviorStatusEnum.INDEX_CHANGED;

            PrimitiveBehaviorType behaviorType = getPrimitiveBehaviorType(inputPacket);
            requestedState.set(behaviorType);

            PrintTools.debug(this, "Requesting Behavior State : " + behaviorType);
         }
      }
   }

   private PrimitiveBehaviorType getPrimitiveBehaviorType(ScriptObject inputPacket)
   {
      PrimitiveBehaviorType ret = PrimitiveBehaviorType.IDLE;

      Object scriptObject = inputPacket.getScriptObject();

      if (scriptObject instanceof FootstepDataListMessage)
      {
         ret = PrimitiveBehaviorType.FOOTSTEP_LIST;
      }
      else if (scriptObject instanceof HandTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.HAND_TRAJECTORY;
      }
      else if (scriptObject instanceof EndEffectorLoadBearingMessage)
      {
         ret = PrimitiveBehaviorType.END_EFFECTOR_LOAD_BEARING;
      }
      else if (scriptObject instanceof HeadTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.HEAD_ORIENTATION;
      }
      else if (scriptObject instanceof PelvisHeightTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.PELVIS_HEIGHT;
      }
      else if (scriptObject instanceof FootTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.FOOT_POSE;
         //         scriptIndex--; //TODO:  <-- Why is this here?  Also, shouldn't scriptStatus != ScriptBehaviorStatusEnum.INDEX_CHANGED?
      }
      else if (scriptObject instanceof PelvisTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.PELVIS_POSE;
      }
      else if (scriptObject instanceof ChestTrajectoryMessage)
      {
         ret = PrimitiveBehaviorType.CHEST_TRAJECTORY;
      }
      else if (scriptObject instanceof HighLevelStateMessage)
      {
         ret = PrimitiveBehaviorType.HIGH_LEVEL_STATE;
      }
      else if (scriptObject instanceof HandDesiredConfigurationMessage)
      {
         ret = PrimitiveBehaviorType.FINGER_STATE;
      }

      return ret;
   }

   private void setChildBehaviorInput(ScriptObject inputPacket)
   {
      inputPacket.applyTransform(behaviorOriginTransformToWorld);

      PrimitiveBehaviorType behaviorType = getPrimitiveBehaviorType(inputPacket);

      if (behaviorType.equals(PrimitiveBehaviorType.FOOTSTEP_LIST))
      {
         footstepListBehavior.initialize();
         footstepListBehavior.set((FootstepDataListMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HAND_TRAJECTORY))
      {
         handTrajectoryBehavior.initialize();
         handTrajectoryBehavior.setInput((HandTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.END_EFFECTOR_LOAD_BEARING))
      {
         endEffectorLoadBearingBehavior.initialize();
         endEffectorLoadBearingBehavior.setInput((EndEffectorLoadBearingMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HEAD_ORIENTATION))
      {
         headTrajectoryBehavior.initialize();
         headTrajectoryBehavior.setInput((HeadTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.PELVIS_HEIGHT))
      {
         pelvisHeightTrajectoryBehavior.initialize();
         pelvisHeightTrajectoryBehavior.setInput((PelvisHeightTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.FOOT_POSE))
      {
         footTrajectoryBehavior.initialize();
         footTrajectoryBehavior.setInput((FootTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.PELVIS_POSE))
      {
         pelvisTrajectoryBehavior.initialize();
         pelvisTrajectoryBehavior.setInput((PelvisTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.CHEST_TRAJECTORY))
      {
         chestTrajectoryBehavior.initialize();
         chestTrajectoryBehavior.setInput((ChestTrajectoryMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HIGH_LEVEL_STATE))
      {
         highLevelStateBehavior.initialize();
         highLevelStateBehavior.setInput((HighLevelStateMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.FINGER_STATE))
      {
         handDesiredConfigurationBehavior.initialize();
         handDesiredConfigurationBehavior.setInput((HandDesiredConfigurationMessage) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.IDLE))
      {
         // nothing to set here
      }
      else
      {

         PrintTools.debug(this, "FAILED TO SET BEHAVIOR INPUT");
      }
   }

   @Override
   public void initialize()
   {
      PrintTools.debug(this, "Initializing");
      scriptImported.set(false);
      scriptFinished.set(false);
      behaviorOriginTransformToWorld = null;

      scriptIndex = 0;
      childInputPackets = null;
      stateMachine.initialize();
      stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);

   }

   @Override
   public void pause()
   {
      PrintTools.debug(this, "Pausing");
      stateMachine.pause();
   }

   @Override
   public void resume()
   {
      PrintTools.debug(this, "Resuming");
      stateMachine.resume();
   }

   @Override
   public void stop()
   {
      PrintTools.debug(this, "Stopping");
      stateMachine.stop();
      scriptFinished.set(true);
      doPostBehaviorCleanup();
      if (childInputPackets != null)
      {
         childInputPackets.clear();
      }
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      PrintTools.debug(this, "Finalizing");
      scriptImported.set(false);
      scriptFinished.set(false);
      behaviorOriginTransformToWorld = null;
      //      scriptResourceStream = null;
      scriptIndex = 0;
      childInputPackets = null;
      stateMachine.doPostBehaviorCleanup();
      stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);
   }

   @Override
   public void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      footstepListBehavior.consumeObjectFromNetworkProcessor(object);
      handTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      endEffectorLoadBearingBehavior.consumeObjectFromNetworkProcessor(object);
      headTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      pelvisHeightTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      footTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      pelvisTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      chestTrajectoryBehavior.consumeObjectFromNetworkProcessor(object);
      highLevelStateBehavior.consumeObjectFromNetworkProcessor(object);
      handDesiredConfigurationBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   public void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      footstepListBehavior.consumeObjectFromController(object);
      handTrajectoryBehavior.consumeObjectFromController(object);
      endEffectorLoadBearingBehavior.consumeObjectFromController(object);
      headTrajectoryBehavior.consumeObjectFromController(object);
      pelvisHeightTrajectoryBehavior.consumeObjectFromController(object);
      footTrajectoryBehavior.consumeObjectFromController(object);
      pelvisTrajectoryBehavior.consumeObjectFromController(object);
      chestTrajectoryBehavior.consumeObjectFromController(object);
      highLevelStateBehavior.consumeObjectFromController(object);
      handDesiredConfigurationBehavior.consumeObjectFromController(object);
   }

   @Override
   public boolean isDone()
   {
      return scriptFinished.getBooleanValue();
   }

   @Override
   public void enableActions()
   {
      stateMachine.enableActions();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      boolean ret = scriptImported.getBooleanValue();

      PrimitiveBehaviorType currentBehaviorState = stateMachine.getCurrentState().getStateEnum();
      if (currentBehaviorState != PrimitiveBehaviorType.IDLE)
      {
         ret &= stateMachine.getCurrentState().getBehavior().hasInputBeenSet();
      }
      return ret;
   }
}
