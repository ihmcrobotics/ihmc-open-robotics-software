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
import us.ihmc.utilities.math.geometry.FrameVector;
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
	private RigidBodyTransform scriptObjectTransformToWorld = null;
	private RigidBodyTransform worldTransformToScriptObject = new RigidBodyTransform();

	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
	
	private final FullRobotModel fullRobotModel;
	private final DoubleYoVariable yoTime;
	
	private final DoubleYoVariable startWalkTime;
	
	private final Vector3d valveInteractionOffset = new Vector3d(0.65, 0.1, 1.016);
	private FrameVector valveInteractionOffsetInWorld = new FrameVector(ReferenceFrame.getWorldFrame());
	
//	private final ModifiableValveModel valveModel;


	public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
			ReferenceFrames referenceFrames, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);

		this.fullRobotModel = fullRobotModel;
		this.yoTime = yoTime;
		
		startWalkTime = new DoubleYoVariable(behaviorName + "StartWalkTime", registry); startWalkTime.set(Double.POSITIVE_INFINITY);
		
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
		
		if (yoTime.getDoubleValue() > (startWalkTime.getDoubleValue() + 10.0) )
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
			scriptObjectTransformToWorld = (receivedScriptBehavior.getReferenceTransform());  

			scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());
			
			scriptObjectTransformToWorld.get(temp); valveLocation.set(temp);			
			valveOrientation.set(scriptObjectTransformToWorld);
						
//			setValveOffsetInValveFrameAndConvertToWorldFrame();
			
			targetWalkLocation.set(valveLocation);
//			targetWalkLocation.add(valveInteractionOffsetInWorld.getPointCopy());

//			walkToLocationBehavior.setTarget(targetWalkLocation, valveOrientation);
			walkToLocationBehavior.setTarget(valveLocation, valveOrientation);

			startWalkTime.set(yoTime.getDoubleValue());
			
			
			System.out.println("Turn Valve Location Updated:" + valveLocation);
			System.out.println("Target Walk to Location Updated:" + targetWalkLocation);

		}
	}

	
	private void setValveOffsetInValveFrameAndConvertToWorldFrame()
	{
		scriptObjectTransformToWorld.invert(worldTransformToScriptObject);
		
		valveInteractionOffsetInWorld.set(valveInteractionOffset);
		valveInteractionOffsetInWorld.changeFrameUsingTransform(ReferenceFrame.getWorldFrame(), scriptObjectTransformToWorld);
		System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveInteractionOffsetInWorld);
	}
	
	
	private boolean inputsSupplied()
	{
		return (scriptResourceStream != null && scriptObjectTransformToWorld != null);
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
