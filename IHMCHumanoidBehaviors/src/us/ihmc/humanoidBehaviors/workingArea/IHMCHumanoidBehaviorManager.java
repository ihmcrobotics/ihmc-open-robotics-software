package us.ihmc.humanoidBehaviors.workingArea;

import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidBehaviors.workingArea.communication.BehaviorCommunicationBridge;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class IHMCHumanoidBehaviorManager {
	private final BehaviorCommunicationBridge communicationBridge;
	private final BehaviorDisptacher dispatcher;
	private final BehaviorFactory behaviorFactory;
	
	public IHMCHumanoidBehaviorManager(ObjectCommunicator networkProcessorCommunicator, ObjectCommunicator controllerCommunicator, YoVariableRegistry parentRegistry)
	{
		communicationBridge = new BehaviorCommunicationBridge(networkProcessorCommunicator, controllerCommunicator, parentRegistry);
		dispatcher = new BehaviorDisptacher(communicationBridge, parentRegistry);
		behaviorFactory = new BehaviorFactory(dispatcher, communicationBridge, parentRegistry);
		networkProcessorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, behaviorFactory);
	}
}
