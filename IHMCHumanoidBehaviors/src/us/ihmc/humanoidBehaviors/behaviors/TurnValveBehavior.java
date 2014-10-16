package us.ihmc.humanoidBehaviors.behaviors;

import java.io.InputStream;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class TurnValveBehavior extends BehaviorInterface
{
	private final ScriptBehavior scriptBehavior;
	private final WalkToLocationBehavior walkToLocationBehavior;
	
	private Vector3d temp;
	private Point3d valveLocation;
	private Point3d targetWalkLocation;
	private YoFrameOrientation valveOrientation;

	private InputStream scriptResourceStream = null;
	private RigidBodyTransform scriptObjectTransformToWorldNoTranslation = null;
	private RigidBodyTransform worldTransformToScriptObject = new RigidBodyTransform();

	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
	
	private final FullRobotModel fullRobotModel;
	private final DoubleYoVariable yoTime;
		
	private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.65, -0.1, 0.0);
	private Vector3d valveInteractionOffsetInWorld = new Vector3d();

	
//	private final ModifiableValveModel valveModel;


	public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
			ReferenceFrames referenceFrames, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);

		this.fullRobotModel = fullRobotModel;
		this.yoTime = yoTime;
				
		temp = new Vector3d();
		valveLocation = new Point3d();
		targetWalkLocation = new Point3d();
		valveOrientation = new YoFrameOrientation(behaviorName + "ValveOrientation", ReferenceFrame.getWorldFrame(), registry);

		scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
		walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames);

		scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<>();
		super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);

//		valveModel = new SixteenInchValveModel(basicJMEInterface, thirdPersonNode)
		
	}

	@Override
	public void doControl() 
	{

		checkIfScriptBehaviorInputPacketReceived();
		walkToLocationBehavior.doControl();
		
		if ( walkToLocationBehavior.isDone() )
		{
//			System.out.println("TurnValveBehavior: Done Walking, now starting Script.");
//			scriptBehavior.doControl();
		}

		if (!inputsSupplied())
		{
			return;
		}


	}

	private void checkIfScriptBehaviorInputPacketReceived() 
	{
		if (scriptBehaviorInputPacketListener.isNewPacketAvailable()) 
		{
			receivedScriptBehavior = scriptBehaviorInputPacketListener.getNewestPacket();
			scriptObjectTransformToWorldNoTranslation = (receivedScriptBehavior.getReferenceTransform());  

			scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());
			
			scriptObjectTransformToWorldNoTranslation.get(temp); valveLocation.set(temp);			
			valveOrientation.set(scriptObjectTransformToWorldNoTranslation);
						
			convertValveOffsetFromValveFrameToWorldFrame();
			
			targetWalkLocation.set(valveLocation);
			targetWalkLocation.add(valveInteractionOffsetInWorld);

			walkToLocationBehavior.setTarget(targetWalkLocation, valveOrientation);
//			walkToLocationBehavior.setTarget(valveLocation, valveOrientation);
			
			
			System.out.println("Turn Valve Location Updated:" + valveLocation);
			System.out.println("Target Walk to Location Updated:" + targetWalkLocation);

		}
	}

	
	private void convertValveOffsetFromValveFrameToWorldFrame()
	{
		scriptObjectTransformToWorldNoTranslation.invert(worldTransformToScriptObject);
		scriptObjectTransformToWorldNoTranslation.setTranslation(0, 0, 0);
		
		
		valveInteractionOffsetInWorld.set(valveInteractionOffsetInValveFrame);
		scriptObjectTransformToWorldNoTranslation.transform(valveInteractionOffsetInValveFrame, valveInteractionOffsetInWorld);
		
		
		System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveInteractionOffsetInWorld);
	}
	
	
	private boolean inputsSupplied()
	{
		return (scriptResourceStream != null && scriptObjectTransformToWorldNoTranslation != null);
	}

	@Override
	protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object) 
	{
		walkToLocationBehavior.passReceivedNetworkProcessorObjectToChildBehaviors(object);
	}

	@Override
	protected void passReceivedControllerObjectToChildBehaviors(Object object)
	{
		walkToLocationBehavior.passReceivedControllerObjectToChildBehaviors(object);
//				scriptBehavior.passReceivedControllerObjectToChildBehaviors(object);
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
	public void finalize() {
		// TODO Auto-generated method stub

	}

	@Override
	public void initialize()
	{
		if ( scriptBehavior != null)
		{
			scriptBehavior.initialize();
		}
		
		if ( walkToLocationBehavior != null)
		{
			walkToLocationBehavior.initialize();
		}

	}

	@Override
	public boolean hasInputBeenSet() {
		// TODO Auto-generated method stub
		return false;
	}



}
