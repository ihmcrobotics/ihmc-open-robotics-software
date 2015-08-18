package us.ihmc.humanoidBehaviors.behaviors.scripts;

import java.io.InputStream;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.BumStatePacket;
import us.ihmc.communication.packets.HighLevelStatePacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorStatusEnum;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorStatusPacket;
import us.ihmc.communication.packets.manipulation.FingerStatePacket;
import us.ihmc.communication.packets.manipulation.HandLoadBearingPacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandStatePacket;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.communication.packets.walking.FootStatePacket;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.communication.packets.walking.ThighStatePacket;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.BumStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ChestOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandLoadBearingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HeadOrientationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HighLevelStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.PrimitiveBehaviorType;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ThighStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngine;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateWrapper;
import us.ihmc.humanoidRobotics.model.BaseFullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateChangedListener;
import us.ihmc.yoUtilities.stateMachines.StateMachineTools;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public class ScriptBehavior extends BehaviorInterface
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
   private final HandPoseBehavior handPoseBehavior;
   private final FootStateBehavior footStateBehavior;
   private final HandStateBehavior handStateBehavior;
   private final HeadOrientationBehavior headOrientationBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final FootPoseBehavior footPoseBehavior;
   private final PelvisPoseBehavior pelvisPoseBehavior;
   private final ChestOrientationBehavior chestOrientationBehavior;
   private final HandLoadBearingBehavior handLoadBearingBehavior;
   private final BumStateBehavior bumStateBehavior;
   private final ThighStateBehavior thighStateBehavior;
   private final HighLevelStateBehavior highLevelStateBehavior;
   public final FingerStateBehavior fingerStateBehavior;

   public ScriptBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, BaseFullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      this(outgoingCommunicationBridge, fullRobotModel, yoTime, null, null);

      PrintTools.debug(this, "Warning: FootPosePackets and FootstepDataList packets are not supported when using this constructor!");
   }

   public ScriptBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, BaseFullRobotModel fullRobotModel, DoubleYoVariable yoTime,
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
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      footStateBehavior = new FootStateBehavior(outgoingCommunicationBridge);
      handStateBehavior = new HandStateBehavior(outgoingCommunicationBridge, yoTime);
      headOrientationBehavior = new HeadOrientationBehavior(outgoingCommunicationBridge, yoTime);
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      if (doubleSupport != null)
      {
         footPoseBehavior = new FootPoseBehavior(outgoingCommunicationBridge, yoTime, doubleSupport);

      }
      else
      {
         footPoseBehavior = null;
      }
      pelvisPoseBehavior = new PelvisPoseBehavior(outgoingCommunicationBridge, yoTime);
      chestOrientationBehavior = new ChestOrientationBehavior(outgoingCommunicationBridge, yoTime);
      handLoadBearingBehavior = new HandLoadBearingBehavior(outgoingCommunicationBridge);
      bumStateBehavior = new BumStateBehavior(outgoingCommunicationBridge);
      thighStateBehavior = new ThighStateBehavior(outgoingCommunicationBridge);
      highLevelStateBehavior = new HighLevelStateBehavior(outgoingCommunicationBridge);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
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
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HAND_POSE, handPoseBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FOOT_STATE, footStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HAND_STATE, handStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HEAD_ORIENTATION, headOrientationBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.COM_HEIGHT, comHeightBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FOOT_POSE, footPoseBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.PELVIS_POSE, pelvisPoseBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.CHEST_ORIENTATION, chestOrientationBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HAND_LOAD, handLoadBearingBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.BUM_STATE, bumStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.THIGH_STATE, thighStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.HIGH_LEVEL_STATE, highLevelStateBehavior);
      wrapBehaviorAndSetupTransitions(stateMachine, PrimitiveBehaviorType.FINGER_STATE, fingerStateBehavior);

      stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);
   }

   private void wrapBehaviorAndSetupTransitions(BehaviorStateMachine<PrimitiveBehaviorType> stateMachine, PrimitiveBehaviorType scriptObjectType,
         BehaviorInterface behavior)
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
         ScriptBehaviorInputPacket receivedPacket = scriptBehaviorInputPacketListener.getNewestPacket();
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

      if (footPoseBehavior == null && getPrimitiveBehaviorType(inputPacket) == PrimitiveBehaviorType.FOOT_POSE)
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

      if (scriptObject instanceof FootstepDataList)
      {
         ret = PrimitiveBehaviorType.FOOTSTEP_LIST;
      }
      else if (scriptObject instanceof HandPosePacket)
      {
         ret = PrimitiveBehaviorType.HAND_POSE;
      }
      else if (scriptObject instanceof FootStatePacket)
      {
         ret = PrimitiveBehaviorType.FOOT_STATE;
      }
      else if (scriptObject instanceof HandStatePacket)
      {
         ret = PrimitiveBehaviorType.HAND_STATE;
      }
      else if (scriptObject instanceof HeadOrientationPacket)
      {
         ret = PrimitiveBehaviorType.HEAD_ORIENTATION;
      }
      else if (scriptObject instanceof ComHeightPacket)
      {
         ret = PrimitiveBehaviorType.COM_HEIGHT;
      }
      else if (scriptObject instanceof FootPosePacket)
      {
         ret = PrimitiveBehaviorType.FOOT_POSE;
         //         scriptIndex--; //TODO:  <-- Why is this here?  Also, shouldn't scriptStatus != ScriptBehaviorStatusEnum.INDEX_CHANGED?
      }
      else if (scriptObject instanceof PelvisPosePacket)
      {
         ret = PrimitiveBehaviorType.PELVIS_POSE;
      }
      else if (scriptObject instanceof ChestOrientationPacket)
      {
         ret = PrimitiveBehaviorType.CHEST_ORIENTATION;
      }
      else if (scriptObject instanceof HandLoadBearingPacket)
      {
         ret = PrimitiveBehaviorType.HAND_LOAD;
      }
      else if (scriptObject instanceof BumStatePacket)
      {
         ret = PrimitiveBehaviorType.BUM_STATE;
      }
      else if (scriptObject instanceof ThighStatePacket)
      {
         ret = PrimitiveBehaviorType.THIGH_STATE;
      }
      else if (scriptObject instanceof HighLevelStatePacket)
      {
         ret = PrimitiveBehaviorType.HIGH_LEVEL_STATE;
      }
      else if (scriptObject instanceof FingerStatePacket)
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
         footstepListBehavior.set((FootstepDataList) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HAND_POSE))
      {
         handPoseBehavior.initialize();
         handPoseBehavior.setInput((HandPosePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.FOOT_STATE))
      {
         footStateBehavior.initialize();
         footStateBehavior.setInput((FootStatePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HAND_STATE))
      {
         handStateBehavior.initialize();
         handStateBehavior.setInput((HandStatePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HEAD_ORIENTATION))
      {
         headOrientationBehavior.initialize();
         headOrientationBehavior.setInput((HeadOrientationPacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.COM_HEIGHT))
      {
         comHeightBehavior.initialize();
         comHeightBehavior.setInput((ComHeightPacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.FOOT_POSE))
      {
         footPoseBehavior.initialize();
         footPoseBehavior.setInput((FootPosePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.PELVIS_POSE))
      {
         pelvisPoseBehavior.initialize();
         pelvisPoseBehavior.setInput((PelvisPosePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.CHEST_ORIENTATION))
      {
         chestOrientationBehavior.initialize();
         chestOrientationBehavior.setInput((ChestOrientationPacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HAND_LOAD))
      {
         handLoadBearingBehavior.initialize();
         handLoadBearingBehavior.setInput((HandLoadBearingPacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.BUM_STATE))
      {
         bumStateBehavior.initialize();
         bumStateBehavior.setInput((BumStatePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.THIGH_STATE))
      {
         thighStateBehavior.initialize();
         thighStateBehavior.setInput((ThighStatePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.HIGH_LEVEL_STATE))
      {
         highLevelStateBehavior.initialize();
         highLevelStateBehavior.setInput((HighLevelStatePacket) inputPacket.getScriptObject());
      }
      else if (behaviorType.equals(PrimitiveBehaviorType.FINGER_STATE))
      {
         fingerStateBehavior.initialize();
         fingerStateBehavior.setInput((FingerStatePacket) inputPacket.getScriptObject());
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
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
      footStateBehavior.consumeObjectFromNetworkProcessor(object);
      handStateBehavior.consumeObjectFromNetworkProcessor(object);
      headOrientationBehavior.consumeObjectFromNetworkProcessor(object);
      comHeightBehavior.consumeObjectFromNetworkProcessor(object);
      footPoseBehavior.consumeObjectFromNetworkProcessor(object);
      pelvisPoseBehavior.consumeObjectFromNetworkProcessor(object);
      chestOrientationBehavior.consumeObjectFromNetworkProcessor(object);
      handLoadBearingBehavior.consumeObjectFromNetworkProcessor(object);
      bumStateBehavior.consumeObjectFromNetworkProcessor(object);
      thighStateBehavior.consumeObjectFromNetworkProcessor(object);
      highLevelStateBehavior.consumeObjectFromNetworkProcessor(object);
      fingerStateBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   public void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      footstepListBehavior.consumeObjectFromController(object);
      handPoseBehavior.consumeObjectFromController(object);
      footStateBehavior.consumeObjectFromController(object);
      handStateBehavior.consumeObjectFromController(object);
      headOrientationBehavior.consumeObjectFromController(object);
      comHeightBehavior.consumeObjectFromController(object);
      footPoseBehavior.consumeObjectFromController(object);
      pelvisPoseBehavior.consumeObjectFromController(object);
      chestOrientationBehavior.consumeObjectFromController(object);
      handLoadBearingBehavior.consumeObjectFromController(object);
      bumStateBehavior.consumeObjectFromController(object);
      thighStateBehavior.consumeObjectFromController(object);
      highLevelStateBehavior.consumeObjectFromController(object);
      fingerStateBehavior.consumeObjectFromController(object);
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
