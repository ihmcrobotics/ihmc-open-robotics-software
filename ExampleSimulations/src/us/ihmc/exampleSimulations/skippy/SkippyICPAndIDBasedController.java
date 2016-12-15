package us.ihmc.exampleSimulations.skippy;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class SkippyICPAndIDBasedController extends SimpleRobotController {
	private final SkippyRobotV2 skippy;
	private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

	private final FramePoint com = new FramePoint(worldFrame);
	private final FramePoint icp = new FramePoint(worldFrame);
	private final FramePoint footLocation = new FramePoint(worldFrame);
	private final FramePoint desiredCMP = new FramePoint(worldFrame);
	private final FrameVector comVelocity = new FrameVector(worldFrame);
	private final FrameVector angularMomentum = new FrameVector(worldFrame);
	private final FrameVector groundReaction = new FrameVector(worldFrame);
	private final FrameVector desiredGroundReaction = new FrameVector(worldFrame);

	public SkippyICPAndIDBasedController(SkippyRobotV2 skippy) {
		this.skippy = skippy;

	}

	@Override
	public void doControl() {

		// --- compute force to apply

		skippy.updateInverseDynamicsStructureFromSimulation();
		skippy.computeComAndICP(com, comVelocity, icp, angularMomentum);
		skippy.computeFootContactForce(groundReaction.getVector());
		footLocation.set(skippy.computeFootLocation());
		skippy.cmpFromIcpDynamics(icp, footLocation, desiredCMP);
		desiredGroundReaction.sub(com, desiredCMP);
		desiredGroundReaction.normalize();
		double reactionModulus = Math.abs(skippy.getGravity()) * skippy.getMass() / desiredGroundReaction.getZ();
		desiredGroundReaction.scale(reactionModulus);

		ReferenceFrame endEffectorFrame = skippy.getEndEffectorFrame();
		RigidBody endEffectorBody = skippy.getEndEffectorBody();
		ReferenceFrame endEffectorBodyFrame = endEffectorBody.getBodyFixedFrame();

//	      endEffectorPosition.setToZero(endEffectorFrame);
//	      endEffectorPosition.changeFrame(worldFrame);
//	      errorVector.setIncludingFrame(targetPosition.getFrameTuple());
//	      errorVector.sub(endEffectorPosition);
//	      errorVector.changeFrame(endEffectorFrame);

	}

}
