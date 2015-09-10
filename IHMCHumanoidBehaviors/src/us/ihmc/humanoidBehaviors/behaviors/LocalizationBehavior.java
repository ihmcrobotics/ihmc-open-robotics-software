package us.ihmc.humanoidBehaviors.behaviors;

import java.io.InputStream;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class LocalizationBehavior extends BehaviorInterface {
	
	private final FullRobotModel fullRobotModel;
	private ScriptBehavior scriptBehavior;
	//private String scriptName = "us/ihmc/atlas/scripts/TestLocalization.xml";
	//private String scriptName = "us/ihmc/atlas/scripts/ShortScriptForLooping.xml";
	private int i = 0;
	private InputStream scriptResourceStream = null;
	private RigidBodyTransform scriptObjectTransformToWorld = null;
	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
	private boolean firstRun = false;
	
	public LocalizationBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, WalkingControllerParameters walkingControllerParameters)
	   {
	      super(outgoingCommunicationBridge);
	      
	      this.fullRobotModel = fullRobotModel;
	      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport, walkingControllerParameters);
	      registry.addChild(scriptBehavior.getYoVariableRegistry());
	      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
	      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
	   }

	@Override
	public void doControl() 
	{
		checkIfScriptBehaviorInputPacketReceived();
		
		if ((scriptBehavior.isDone() && i < 10) || firstRun)
		{
			System.out.println("Starting iteration " + i + " of the localization test.");
			i++;
			reloadScriptBehavior();
			firstRun = false; 
		}

		scriptBehavior.doControl();
	}
	
	private void checkIfScriptBehaviorInputPacketReceived() 
	{
		if (scriptBehaviorInputPacketListener.isNewPacketAvailable()) 
		{
			firstRun = true;
			System.out.println("New packet available. Loading inputs...");
			receivedScriptBehavior = scriptBehaviorInputPacketListener.getNewestPacket();
			scriptObjectTransformToWorld = (receivedScriptBehavior.getReferenceTransform());
			scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());
			
			if (scriptResourceStream == null)
				System.out.println("Script Resource Stream is null. Can't load script!");
			else
				System.out.println("Script " + receivedScriptBehavior.getScriptName() + " loaded.");
		}
	}

	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(
			Object object) {

	}

	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object) {
		scriptBehavior.consumeObjectFromController(object);
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub

	}

	@Override
	public void enableActions() {
		// TODO Auto-generated method stub

	}

	@Override
	public void pause() {
		// TODO Auto-generated method stub

	}

	@Override
	public void resume() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean isDone() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void doPostBehaviorCleanup() {
		// TODO Auto-generated method stub

	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean hasInputBeenSet() {
		// TODO Auto-generated method stub
		return false;
	}
	
	private void reloadScriptBehavior() {
		scriptBehavior.doPostBehaviorCleanup();
		scriptBehavior.initialize();
		scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());
		scriptBehavior.importChildInputPackets(getClass().getSimpleName(), scriptResourceStream,scriptObjectTransformToWorld);
		
	}

}
