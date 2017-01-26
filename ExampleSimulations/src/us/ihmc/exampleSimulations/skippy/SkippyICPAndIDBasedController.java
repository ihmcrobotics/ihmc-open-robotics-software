package us.ihmc.exampleSimulations.skippy;

import java.util.ArrayList;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class SkippyICPAndIDBasedController extends SimpleRobotController
{
   private final SkippyRobotV2 skippy;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

//   private final FramePoint com = new FramePoint(worldFrame);
//   private final FramePoint icp = new FramePoint(worldFrame);
//   private final FramePoint footLocation = new FramePoint(worldFrame);
//   private final FramePoint desiredCMP = new FramePoint(worldFrame);
//   private final FrameVector comVelocity = new FrameVector(worldFrame);
//   private final FrameVector angularMomentum = new FrameVector(worldFrame);
//   private final FrameVector groundReaction = new FrameVector(worldFrame);
//   private final FrameVector desiredGroundReaction = new FrameVector(worldFrame);

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final TwistCalculator twistCalculator;

   private final ArrayList<YoGraphicReferenceFrame> referenceFrameGraphics = new ArrayList<>();

   public SkippyICPAndIDBasedController(SkippyRobotV2 skippy, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.skippy = skippy;

      twistCalculator = new TwistCalculator(worldFrame, skippy.getElevator());
      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, -skippy.getGravityZ());

      setupGraphics(graphicsListRegistry);
   }

   private void setupGraphics(YoGraphicsListRegistry graphicsListRegistry)
   {
      RigidBody[] bodies = ScrewTools.computeRigidBodiesAfterThisJoint(skippy.getTorso().getParentJoint());
      for (RigidBody body : bodies)
      {
         YoGraphicReferenceFrame referenceFrameBody = new YoGraphicReferenceFrame(body.getBodyFixedFrame(), registry, 0.4);
         graphicsListRegistry.registerYoGraphic(body.getName() + "BodyFrame", referenceFrameBody);
         referenceFrameGraphics.add(referenceFrameBody);
      }

      YoGraphicReferenceFrame referenceFrameElevator = new YoGraphicReferenceFrame(skippy.getElevator().getBodyFixedFrame(), registry, 0.4);
      graphicsListRegistry.registerYoGraphic(skippy.getElevator().getName() + "BodyFrame", referenceFrameElevator);
      referenceFrameGraphics.add(referenceFrameElevator);

      YoGraphicReferenceFrame referenceFrameEndEffector = new YoGraphicReferenceFrame(skippy.getEndEffectorFrame(), registry, 0.4);
      graphicsListRegistry.registerYoGraphic("EndEffectorFrame", referenceFrameEndEffector);
      referenceFrameGraphics.add(referenceFrameEndEffector);
   }

   @Override
   public void doControl()
   {

      // --- compute force to apply

//      skippy.computeComAndICP(com, comVelocity, icp, angularMomentum);
//      skippy.computeFootContactForce(groundReaction.getVector());
//      footLocation.set(skippy.computeFootLocation());
//      skippy.cmpFromIcpDynamics(icp, footLocation, desiredCMP);
//      desiredGroundReaction.sub(com, desiredCMP);
//      desiredGroundReaction.normalize();
//      double reactionModulus = Math.abs(skippy.getGravity()) * skippy.getMass() / desiredGroundReaction.getZ();
//      desiredGroundReaction.scale(reactionModulus);
//
//      ReferenceFrame endEffectorFrame = skippy.getEndEffectorFrame();
//      RigidBody endEffectorBody = skippy.getEndEffectorBody();
//      ReferenceFrame endEffectorBodyFrame = endEffectorBody.getBodyFixedFrame();

      //	      endEffectorPosition.setToZero(endEffectorFrame);
      //	      endEffectorPosition.changeFrame(worldFrame);
      //	      errorVector.setIncludingFrame(targetPosition.getFrameTuple());
      //	      errorVector.sub(endEffectorPosition);
      //	      errorVector.changeFrame(endEffectorFrame);


      skippy.updateInverseDynamicsStructureFromSimulation();
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
      skippy.updateSimulationFromInverseDynamicsTorques();

      updateGraphics();
   }

   private void updateGraphics()
   {
      for (YoGraphicReferenceFrame frameGraphics : referenceFrameGraphics)
         frameGraphics.update();
   }

}
