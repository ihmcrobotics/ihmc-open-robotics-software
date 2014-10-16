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
	
	private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.65, 0.2, 0.0);
//	private final Vector3d valveInteractionOffsetInValveFrame = new Vector3d(-0.65, 2.0, 0.0);


	private Point3d targetWalkLocation;
	private final YoFrameOrientation targetWalkOrientation;

	private InputStream scriptResourceStream;

	private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
	private ScriptBehaviorInputPacket receivedScriptBehavior;
		
	
//	private final ModifiableValveModel valveModel;


	public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
			ReferenceFrames referenceFrames, DoubleYoVariable yoTime)
	{
		super(outgoingCommunicationBridge);
				
		targetWalkLocation = new Point3d();
		valveOrientation = new YoFrameOrientation(behaviorName + "ValveOrientation", ReferenceFrame.getWorldFrame(), registry);
		targetWalkOrientation = new YoFrameOrientation(behaviorName + "WalkToOrientation", ReferenceFrame.getWorldFrame(), registry);
		
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

	private RigidBodyTransform worldToValveTransform;

	
	private void checkIfScriptBehaviorInputPacketReceived() 
	{
		if (scriptBehaviorInputPacketListener.isNewPacketAvailable()) 
		{
			receivedScriptBehavior = scriptBehaviorInputPacketListener.getNewestPacket();
			worldToValveTransform = receivedScriptBehavior.getReferenceTransform();	

			
			setValveLocationAndOrientation( worldToValveTransform );
												
			targetWalkLocation.set(valveLoction);
//			convertVectorFromValveFrameToWorldFrame(valveInteractionOffsetInValveFrame, valveOffsetInWorldFrame);
			convertVectorFromValveFrameToWorldFrame_Hack(valveInteractionOffsetInValveFrame, valveOffsetInWorldFrame);

			System.out.println("TurnValveBehavior:  ValveOffset in World Frame:" + valveOffsetInWorldFrame);		

			targetWalkLocation.add(valveOffsetInWorldFrame);
			
			targetWalkOrientation.setYawPitchRoll(
					valveOrientation.getYaw().getDoubleValue() + 0.5*Math.PI,
					valveOrientation.getPitch().getDoubleValue(),
					valveOrientation.getRoll().getDoubleValue());

			walkToLocationBehavior.setTarget(targetWalkLocation, targetWalkOrientation);
			

			System.out.println("Turn Valve Location Updated:" + valveLoction);
			System.out.println("Target Walk to Location Updated:" + targetWalkLocation);
			
			scriptResourceStream = getClass().getClassLoader().getResourceAsStream(receivedScriptBehavior.getScriptName());

		}
	}
	
	
	private Vector3d valveLoction = new Vector3d();
	private YoFrameOrientation valveOrientation;

	private void setValveLocationAndOrientation(RigidBodyTransform worldToValveTransform) 
	{
		worldToValveTransform.get(valveLoction);
		valveOrientation.set(worldToValveTransform);
	}
	
	
	private RigidBodyTransform valveToWorldRotation = new RigidBodyTransform();
	private Vector3d valveOffsetInWorldFrame = new Vector3d();
	
	private void convertVectorFromValveFrameToWorldFrame(Vector3d vecInValveFrame, Vector3d vecInWorldFrameToPack) 
	{
		valveToWorldRotation.set(worldToValveTransform);
		valveToWorldRotation.invert();
		valveToWorldRotation.setTranslation(0, 0, 0);

		valveToWorldRotation.transform(vecInValveFrame, vecInWorldFrameToPack);
	}


	private void convertVectorFromValveFrameToWorldFrame_Hack(Vector3d vecInValveFrame, Vector3d vecInWorldFrameToPack)
	{
		double x = vecInValveFrame.x;
		double y = vecInValveFrame.y;
		double yaw = valveOrientation.getYaw().getDoubleValue() + 0.5*Math.PI;
		vecInWorldFrameToPack.set(x*Math.cos(yaw) - y*Math.sin(yaw), x*Math.sin(yaw) + y*Math.cos(yaw), 0.0);
	}
	
	
	private boolean inputsSupplied()
	{
		return (scriptResourceStream != null && worldToValveTransform != null);
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
