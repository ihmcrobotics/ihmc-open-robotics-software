package us.ihmc.humanoidBehaviors.behaviors.scripts;

import java.io.InputStream;
import java.util.ArrayList;

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
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
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
	
	private final BooleanYoVariable scriptLoaded = new BooleanYoVariable("scriptLoaded", registry);
	private final BooleanYoVariable scriptFinished = new BooleanYoVariable("scriptFinished", registry);
	
	private final ScriptEngine scriptEngine;
	private InputStream scriptResourceStream = null;
	private RigidBodyTransform scriptObjectTransformToWorld = null;
	private ArrayList<ScriptObject> scriptObjects = null;
	private ScriptBehaviorStatusEnum scriptStatus;
	private int scriptIndex = 0;
	private ScriptObject scriptObject = null;
	
	private final FullRobotModel fullRobotModel;
	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	
	private final BehaviorStateMachine<PrimitiveBehaviorType> stateMachine;
	private final EnumYoVariable<PrimitiveBehaviorType> requestedState = new EnumYoVariable<>("requestedScriptBehaviorState", registry, PrimitiveBehaviorType.class, true);
	
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
	private final FingerStateBehavior fingerStateBehavior;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
	
	public ScriptBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);
		
		this.fullRobotModel = fullRobotModel;
		
		scriptEngine = new ScriptEngine(null);
		
		footstepListBehavior = new FootstepListBehavior(outgoingCommunicationBridge);
		handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
		footStateBehavior = new FootStateBehavior(outgoingCommunicationBridge);
		handStateBehavior = new HandStateBehavior(outgoingCommunicationBridge, yoTime);
		headOrientationBehavior = new HeadOrientationBehavior(outgoingCommunicationBridge, yoTime);
		comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
		footPoseBehavior = new FootPoseBehavior(outgoingCommunicationBridge, yoTime);
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
		setupStateMachine(yoTime);
		
		if (DEBUG)
		{
			stateMachine.attachStateChangedListener(new StateChangedListener<PrimitiveBehaviorType>()
					{
				@Override
				public void stateChanged(State<PrimitiveBehaviorType> oldState, State<PrimitiveBehaviorType> newState, double time)
				{
					System.out.println("Switching from: " + oldState.getStateEnum() + " to: " + newState.getStateEnum() + " at t = " + time);
				}
					});
		}
		requestedState.set(null);
	}
	
	private void setupStateMachine(DoubleYoVariable yoTime)
	{
		addScriptBehavior(PrimitiveBehaviorType.IDLE, new SimpleDoNothingBehavior(outgoingCommunicationBridge));
		addScriptBehavior(PrimitiveBehaviorType.FOOTSTEP_LIST, footstepListBehavior);
		addScriptBehavior(PrimitiveBehaviorType.HAND_POSE, handPoseBehavior);
		addScriptBehavior(PrimitiveBehaviorType.FOOT_STATE, footStateBehavior);
		addScriptBehavior(PrimitiveBehaviorType.HAND_STATE, handStateBehavior);
		addScriptBehavior(PrimitiveBehaviorType.HEAD_ORIENTATION, headOrientationBehavior);
		addScriptBehavior(PrimitiveBehaviorType.COM_HEIGHT, comHeightBehavior);
		addScriptBehavior(PrimitiveBehaviorType.FOOT_POSE, footPoseBehavior);
		addScriptBehavior(PrimitiveBehaviorType.PELVIS_POSE, pelvisPoseBehavior);
		addScriptBehavior(PrimitiveBehaviorType.CHEST_ORIENTATION, chestOrientationBehavior);
		addScriptBehavior(PrimitiveBehaviorType.HAND_LOAD, handLoadBearingBehavior);
		addScriptBehavior(PrimitiveBehaviorType.BUM_STATE, bumStateBehavior);
		addScriptBehavior(PrimitiveBehaviorType.THIGH_STATE, thighStateBehavior);
		addScriptBehavior(PrimitiveBehaviorType.HIGH_LEVEL_STATE, highLevelStateBehavior);
		addScriptBehavior(PrimitiveBehaviorType.FINGER_STATE, fingerStateBehavior);
		
		
		stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);
	}
	
	private void addScriptBehavior(PrimitiveBehaviorType scriptObjectType, BehaviorInterface newBehavior)
	{
		final BehaviorStateWrapper<PrimitiveBehaviorType> newBehaviorState = new BehaviorStateWrapper<PrimitiveBehaviorType>(scriptObjectType, newBehavior);
		
		stateMachine.addState(newBehaviorState);
		registry.addChild(newBehavior.getYoVariableRegistry());
		
		if (scriptObjectType == PrimitiveBehaviorType.IDLE)
			return;
		
		BehaviorStateWrapper<PrimitiveBehaviorType> idleState = stateMachine.getState(PrimitiveBehaviorType.IDLE);
		
		// Enable transition from any state to IDLE
		StateTransitionCondition toWaitStateTransitionCondition = new StateTransitionCondition()
		{
			@Override
			public boolean checkCondition()
			{
				return newBehaviorState.isDone();
			}
		};
		
		// Load the next script object when switching to IDLE. If no new script object, stay in IDLE the behavior is done.
		StateTransitionAction loadNextScriptObjectAction = new StateTransitionAction()
		{
			@Override
			public void doTransitionAction()
			{
				loadNextScriptObject();
				outgoingCommunicationBridge.sendPacketToNetworkProcessor(new ScriptBehaviorStatusPacket(scriptStatus,scriptIndex));
			}
		};
		
		StateTransition<PrimitiveBehaviorType> toWaitStateTransition = new StateTransition<>(PrimitiveBehaviorType.IDLE, toWaitStateTransitionCondition, loadNextScriptObjectAction);
		newBehaviorState.addStateTransition(toWaitStateTransition);
		
		// State transition using the requestedState enum value that is set in loadNextScriptObject.
		StateMachineTools.addRequestedStateTransition(requestedState, false, idleState, newBehaviorState);
	}
	
	@Override
	public void doControl()
	{
		checkIfScriptBehaviorInputPacketReceived();
		if (!inputsSupplied() || scriptFinished.getBooleanValue())
		{
			return;
		}
		
		
		if (!scriptLoaded.getBooleanValue())
			loadScript();
		
		stateMachine.checkTransitionConditions();
		if (!hasInputBeenSet())
			setInputForNextScriptObject();
		stateMachine.doAction();
	}
	
	private void checkIfScriptBehaviorInputPacketReceived() 
	{
		if (scriptBehaviorInputPacketListener.isNewPacketAvailable()) 
		{
			receivedScriptBehavior = scriptBehaviorInputPacketListener.getNewestPacket();
			scriptObjectTransformToWorld = (receivedScriptBehavior.getReferenceTransform());
			scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());
			
			if (scriptResourceStream == null)
				System.out.println("Script Resource Stream is null. Can't load script!");
			else
				System.out.println("Script " + receivedScriptBehavior.getScriptName() + " loaded.");
		}
	}
	
	private boolean inputsSupplied()
	{
		return (scriptResourceStream != null && scriptObjectTransformToWorld != null);
	}
	
	private void loadScript()
	{
		scriptIndex = 0;
		scriptObjects = scriptEngine.getScriptObjects(scriptResourceStream);
		scriptLoaded.set(true);
		scriptStatus = scriptStatus.SCRIPT_LOADED;
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(new ScriptBehaviorStatusPacket(scriptStatus,scriptIndex));
		loadNextScriptObject();
		outgoingCommunicationBridge.sendPacketToNetworkProcessor(new ScriptBehaviorStatusPacket(scriptStatus,scriptIndex));
	}
	
	private void loadNextScriptObject()
	{
		
		if (scriptObjects.size() == 0)
		{
			scriptStatus = scriptStatus.SCRIPT_LOAD_FAILED;
			return;
		}
		
		scriptIndex++;
		scriptStatus = scriptStatus.INDEX_CHANGED;
		
		scriptObject = scriptObjects.remove(0);
		
		scriptObject.applyTransform(scriptObjectTransformToWorld);
		
		if (scriptObject.getScriptObject() instanceof EndOfScriptCommand)
		{
			if (DEBUG) System.out.println("End of script");
			scriptFinished.set(true);
			scriptIndex--;
			scriptStatus = scriptStatus.FINISHED;
		}
		else if (scriptObject.getScriptObject() instanceof FootstepDataList)
		{
			if (DEBUG) System.out.println("Loaded footsteps");
			requestedState.set(PrimitiveBehaviorType.FOOTSTEP_LIST);
		}
		else if (scriptObject.getScriptObject() instanceof HandPosePacket)
		{
			if (DEBUG) System.out.println("Loaded hand pose");
			requestedState.set(PrimitiveBehaviorType.HAND_POSE);
		}
		else if (scriptObject.getScriptObject() instanceof FootStatePacket)
		{
			if (DEBUG) System.out.println("Loaded foot state");
			requestedState.set(PrimitiveBehaviorType.FOOT_STATE);
		}
		else if (scriptObject.getScriptObject() instanceof HandStatePacket)
		{
			if (DEBUG) System.out.println("Loaded hand state");
			requestedState.set(PrimitiveBehaviorType.HAND_STATE);
		}
		else if (scriptObject.getScriptObject() instanceof HeadOrientationPacket)
		{
			if (DEBUG) System.out.println("Loaded head orientation");
			requestedState.set(PrimitiveBehaviorType.HEAD_ORIENTATION);
		}
		else if (scriptObject.getScriptObject() instanceof ComHeightPacket)
		{
			if (DEBUG) System.out.println("Loaded COM height");
			requestedState.set(PrimitiveBehaviorType.COM_HEIGHT);
		}
		else if (scriptObject.getScriptObject() instanceof FootPosePacket)
		{
			if (DEBUG) System.out.println("Loaded foot pose");
			requestedState.set(PrimitiveBehaviorType.FOOT_POSE);
			scriptIndex--;
		}
		else if (scriptObject.getScriptObject() instanceof PelvisPosePacket)
		{
			if (DEBUG) System.out.println("Loaded pelvis pose");
			requestedState.set(PrimitiveBehaviorType.PELVIS_POSE);
		}
		else if (scriptObject.getScriptObject() instanceof ChestOrientationPacket)
		{
			if (DEBUG) System.out.println("Loaded chest orientation");
			requestedState.set(PrimitiveBehaviorType.CHEST_ORIENTATION);
		}
		else if (scriptObject.getScriptObject() instanceof HandLoadBearingPacket)
		{
			if (DEBUG) System.out.println("Loaded hand load bearing");
			requestedState.set(PrimitiveBehaviorType.HAND_LOAD);
		}
		else if (scriptObject.getScriptObject() instanceof BumStatePacket)
		{
			if (DEBUG) System.out.println("Loaded bum state");
			requestedState.set(PrimitiveBehaviorType.BUM_STATE);
		}
		else if (scriptObject.getScriptObject() instanceof ThighStatePacket)
		{
			if (DEBUG) System.out.println("Loaded thigh state");
			requestedState.set(PrimitiveBehaviorType.THIGH_STATE);
		}
		else if (scriptObject.getScriptObject() instanceof HighLevelStatePacket)
		{
			if (DEBUG) System.out.println("Loaded high level state");
			requestedState.set(PrimitiveBehaviorType.HIGH_LEVEL_STATE);
		}
		else if (scriptObject.getScriptObject() instanceof FingerStatePacket)
		{
			if (DEBUG) System.out.println("Loaded head orientation");
			requestedState.set(PrimitiveBehaviorType.FINGER_STATE);
		}
		else
		{
			loadNextScriptObject();
			scriptIndex--;
		}
	}
	
	public void setInputForNextScriptObject()
	{
		if (scriptObject.getScriptObject() instanceof FootstepDataList)
		{
			footstepListBehavior.set((FootstepDataList) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof HandPosePacket)
		{
			handPoseBehavior.setInput((HandPosePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof FootStatePacket)
		{
			footStateBehavior.setInput((FootStatePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof HandStatePacket)
		{
			handStateBehavior.setInput((HandStatePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof HeadOrientationPacket)
		{
			headOrientationBehavior.setInput((HeadOrientationPacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof ComHeightPacket)
		{
			comHeightBehavior.setInput((ComHeightPacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof FootPosePacket)
		{
			footPoseBehavior.setInput((FootPosePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof PelvisPosePacket)
		{
			pelvisPoseBehavior.setInput((PelvisPosePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof ChestOrientationPacket)
		{
			chestOrientationBehavior.setInput((ChestOrientationPacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof HandLoadBearingPacket)
		{
			handLoadBearingBehavior.setInput((HandLoadBearingPacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof BumStatePacket)
		{
			bumStateBehavior.setInput((BumStatePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof ThighStatePacket)
		{
			thighStateBehavior.setInput((ThighStatePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof HighLevelStatePacket)
		{
			highLevelStateBehavior.setInput((HighLevelStatePacket) scriptObject.getScriptObject());
		}
		else if (scriptObject.getScriptObject() instanceof FingerStatePacket)
		{
			fingerStateBehavior.setInput((FingerStatePacket) scriptObject.getScriptObject());
		}
		else
		{
		}
	}
	
	@Override
	public void initialize()
	{
		// TODO Need to get the file from the UI maybe
//      getScriptFileFromDialog();
		
		// Hardcoded for testing
//      scriptFile = new File("C:\\Users\\Sylvain\\Desktop\\workspace\\IHMCHumanoidBehaviors\\resources\\scripts\\shakeout_prime.xml");
//      scriptFile = new File("C:\\Users\\Sylvain\\Desktop\\workspace\\DarpaRoboticsChallenge\\test\\us\\ihmc\\darpaRoboticsChallenge\\obstacleCourseTests\\scripts\\ExerciseAndJUnitScripts\\SimpleFlatGroundScript.xml");
		
		// TODO Need to get the transform from the UI
//      if (scriptObjectTransformToWorld == null)
//      {
//         scriptObjectTransformToWorld = fullRobotModel.getSoleFrame(RobotSide.LEFT).getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
//      }
	}
	
	@Override
	public void stop()
	{
		scriptFinished.set(true);
		finalize();
		if(scriptObjects != null)
		{
			scriptObjects.clear();
		}
	}
	
	@Override
	public void pause()
	{
		stateMachine.pause();
	}
	
	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
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
	protected void passReceivedControllerObjectToChildBehaviors(Object object)
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
	public void finalize()
	{
		scriptFinished.set(false);
		scriptLoaded.set(false);
		scriptObjectTransformToWorld = null;
		scriptResourceStream = null;
		scriptObjects = null;
		stateMachine.finalize();
		stateMachine.setCurrentState(PrimitiveBehaviorType.IDLE);
		scriptIndex = 0;
	}
	
	@Override
	public void resume()
	{
		stateMachine.resume();
	}
	
	@Override
	public boolean hasInputBeenSet() 
	{
		if (scriptObject.getScriptObject() instanceof FootstepDataList)
		{
			return footstepListBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof HandPosePacket)
		{
			return handPoseBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof FootStatePacket)
		{
			return footStateBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof HandStatePacket)
		{
			return handStateBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof HeadOrientationPacket)
		{
			return headOrientationBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof ComHeightPacket)
		{
			return comHeightBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof FootPosePacket)
		{
			return footPoseBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof PelvisPosePacket)
		{
			return pelvisPoseBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof ChestOrientationPacket)
		{
			return chestOrientationBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof HandLoadBearingPacket)
		{
			return handLoadBearingBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof BumStatePacket)
		{
			return bumStateBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof ThighStatePacket)
		{
			return thighStateBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof HighLevelStatePacket)
		{
			return highLevelStateBehavior.hasInputBeenSet();
		}
		else if (scriptObject.getScriptObject() instanceof FingerStatePacket)
		{
			return fingerStateBehavior.hasInputBeenSet();
		}
		else
		{
			return false;
		}
	}
}
