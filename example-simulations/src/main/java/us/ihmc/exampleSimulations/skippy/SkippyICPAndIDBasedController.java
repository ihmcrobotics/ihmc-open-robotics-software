package us.ihmc.exampleSimulations.skippy;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class SkippyICPAndIDBasedController extends SimpleRobotController
{
   private final SkippyRobotV2 skippy;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble kCapture = new YoDouble("kCapture", registry);
   private final YoDouble totalMass = new YoDouble("totalMass", registry);

   private final FramePoint3D com = new FramePoint3D(worldFrame);
   private final FramePoint3D icp = new FramePoint3D(worldFrame);
   private final FrameVector3D comVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D angularMomentum = new FrameVector3D(worldFrame);
   private final FrameVector3D actualGroundReaction = new FrameVector3D(worldFrame);
   private final FramePoint3D footLocation = new FramePoint3D(worldFrame);
   private final FramePoint3D desiredCMP = new FramePoint3D(worldFrame);
   private final FrameVector3D desiredGroundReaction = new FrameVector3D(worldFrame);

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final Wrench endEffectorWrench = new Wrench();
   private final FrameVector3D errorVector = new FrameVector3D();
   private final YoFramePoint targetPosition;
   private final FramePoint3D endEffectorPosition = new FramePoint3D();
   private final YoDouble kp;

   private final ArrayList<YoGraphicReferenceFrame> referenceFrameGraphics = new ArrayList<>();

   public SkippyICPAndIDBasedController(SkippyRobotV2 skippy, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.skippy = skippy;

      inverseDynamicsCalculator = new InverseDynamicsCalculator(skippy.getElevator(), -skippy.getGravityZ());

      setupGraphics(graphicsListRegistry);
      totalMass.set(skippy.computeCenterOfMass(new Point3D()));

      kp = new YoDouble("kpTaskspace", registry);
      kp.set(0.5);

      targetPosition = new YoFramePoint("targetPosition", skippy.getRightShoulderFrame(), registry);
      targetPosition.set(0.0, 0.1, 0.0);

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

      YoGraphicReferenceFrame referenceFrameEndEffector = new YoGraphicReferenceFrame(skippy.getFootFrame(), registry, 0.4);
      graphicsListRegistry.registerYoGraphic("EndEffectorFrame", referenceFrameEndEffector);
      referenceFrameGraphics.add(referenceFrameEndEffector);
   }

   @Override
   public void doControl()
   {
      skippy.updateInverseDynamicsStructureFromSimulation();

      // --- compute force to apply

      //      cmpToComReaction();

      //End effector on the right shoulder
      ReferenceFrame rightShoulderFrame = skippy.getRightShoulderFrame();
      RigidBody shoulderBody = skippy.getShoulderBody();
      ReferenceFrame rightShoulderBodyFrame = shoulderBody.getBodyFixedFrame();

      // --- compute force to pull the end effector towards the target position
      ReferenceFrame endEffectorFrame = skippy.getRightShoulderFrame();
      RigidBody endEffectorBody = skippy.getShoulderBody();
      ReferenceFrame endEffectorBodyFrame = endEffectorBody.getBodyFixedFrame();

      endEffectorPosition.setToZero(endEffectorFrame); //set(targetPosition.getFrameTuple());  //

      errorVector.setIncludingFrame(targetPosition);
      errorVector.sub(endEffectorPosition);
      errorVector.changeFrame(endEffectorFrame);

      endEffectorWrench.setToZero(endEffectorBodyFrame, endEffectorFrame);
      endEffectorWrench.setLinearPart(errorVector);
      endEffectorWrench.changeFrame(endEffectorBodyFrame);

      endEffectorWrench.scale(-kp.getDoubleValue());
      inverseDynamicsCalculator.setExternalWrench(endEffectorBody, endEffectorWrench);
      // ---

      inverseDynamicsCalculator.compute();

      skippy.updateSimulationFromInverseDynamicsTorques();

      skippy.updateInverseDynamicsStructureFromSimulation();

      inverseDynamicsCalculator.compute();
      skippy.updateSimulationFromInverseDynamicsTorques();

      updateGraphics();
   }

   /**
    * Compute desiredGroundReaction according to Skippy report Eq. 10 
    */
   public void cmpToComReaction()
   {
      // --- NOT READY!!!
      computeComAndICP(com, comVelocity, icp, angularMomentum);
      skippy.computeFootContactForce(actualGroundReaction);
      footLocation.set(skippy.computeFootLocation());
      cmpFromIcpDynamics(icp, footLocation, desiredCMP);
      desiredGroundReaction.sub(com, desiredCMP);
      desiredGroundReaction.normalize();
      double reactionModulus = Math.abs(skippy.getGravity()) * totalMass.getDoubleValue() / desiredGroundReaction.getZ();
      desiredGroundReaction.scale(reactionModulus);
   }

   private void updateGraphics()
   {
      for (YoGraphicReferenceFrame frameGraphics : referenceFrameGraphics)
         frameGraphics.update();
   }

   private final Point3D tempCOMPosition = new Point3D();
   private final Vector3D tempLinearMomentum = new Vector3D();
   private final Vector3D tempAngularMomentum = new Vector3D();

   public void computeComAndICP(FramePoint3D comToPack, FrameVector3D comVelocityToPack, FramePoint3D icpToPack, FrameVector3D angularMomentumToPack)
   {
      totalMass.set(skippy.computeCOMMomentum(tempCOMPosition, tempLinearMomentum, tempAngularMomentum));
      angularMomentumToPack.set(tempAngularMomentum);

      comToPack.set(tempCOMPosition);
      tempLinearMomentum.scale(1.0 / totalMass.getDoubleValue());
      comVelocityToPack.set(tempLinearMomentum);

      double omega0 = Math.sqrt(comToPack.getZ() / Math.abs(skippy.getGravityZ()));

      icpToPack.scaleAdd(omega0, comVelocityToPack, comToPack);
      icpToPack.setZ(0.0);
   }

   public void cmpFromIcpDynamics(FramePoint3D icp, FramePoint3D footLocation, FramePoint3D desiredCMPToPack)
   {
      FrameVector3D icpToFoot = new FrameVector3D();
      icpToFoot.sub(icp, footLocation);
      desiredCMPToPack.scaleAdd(kCapture.getDoubleValue(), icpToFoot, icp);
      desiredCMPToPack.setZ(0.0);
   }

}
