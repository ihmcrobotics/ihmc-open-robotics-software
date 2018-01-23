package us.ihmc.exampleSimulations.skippy;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class SkippyICPBasedController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SkippyRobot skippy;

   private final double dt;

   private final YoDouble kCapture = new YoDouble("kCapture", registry);
   private final YoDouble kMomentum = new YoDouble("kMomentum", registry);
   private final YoDouble kAngle = new YoDouble("kAngle", registry);
   private final YoDouble hipSetpoint = new YoDouble("hipSetpoint", registry);
   private final YoDouble shoulderSetpoint = new YoDouble("shoulderSetpoint", registry);

   private final YoInteger tickCounter = new YoInteger("tickCounter", registry);
   private final YoInteger ticksForDesiredForce = new YoInteger("ticksForDesiredForce", registry);
   private final PIDController hipAngleController = new PIDController("hipAngleController", registry);
   private final PIDController shoulderAngleController = new PIDController("shoulderAngleController", registry);
   private final FrameVector3D angularMomentum = new FrameVector3D(worldFrame);

   private final FramePoint3D com = new FramePoint3D(worldFrame);
   private final FrameVector3D comVelocity = new FrameVector3D(worldFrame);
   private final FramePoint3D icp = new FramePoint3D(worldFrame);
   private final FramePoint3D footLocation = new FramePoint3D(worldFrame);
   private final FramePoint3D desiredCMP = new FramePoint3D(worldFrame);
   private final FrameVector3D desiredGroundReaction = new FrameVector3D(worldFrame);
   private final FrameVector3D groundReaction = new FrameVector3D(worldFrame);
   private final FrameVector3D worldToHip = new FrameVector3D(worldFrame);
   private final FrameVector3D worldToShoulder = new FrameVector3D(worldFrame);
   private final FrameVector3D hipToFootDirection = new FrameVector3D(worldFrame);
   private final FrameVector3D shoulderToFootDirection = new FrameVector3D(worldFrame);
   private final FrameVector3D hipAxis = new FrameVector3D(worldFrame);
   private final FrameVector3D shoulderAxis = new FrameVector3D(worldFrame);

   private final YoFramePoint comViz = new YoFramePoint("CoM", worldFrame, registry);
   private final YoFramePoint icpViz = new YoFramePoint("ICP", worldFrame, registry);
   private final YoFramePoint desiredCMPViz = new YoFramePoint("DesiredCMP", worldFrame, registry);
   private final YoFramePoint footLocationViz = new YoFramePoint("FootLocation", worldFrame, registry);
   private final YoFramePoint hipLocationViz = new YoFramePoint("HipLocation", worldFrame, registry);
   private final YoFramePoint shoulderLocationViz = new YoFramePoint("ShoulderLocation", worldFrame, registry);
   private final YoFrameVector desiredGroundReactionViz = new YoFrameVector("DesiredGroundReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector groundReachtionViz = new YoFrameVector("GroundReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootDirectionViz = new YoFrameVector("HipToFootDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipAxisViz = new YoFrameVector("HipAxis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderAxisViz = new YoFrameVector("ShoulderAxis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector angularMomentumViz = new YoFrameVector("AngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   public SkippyICPBasedController(SkippyRobot skippy, double dt, YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      this.skippy = skippy;
      this.dt = dt;

      kCapture.set(7.5);
      kMomentum.set(0.3);
      kAngle.set(0.1);

      ticksForDesiredForce.set(10);
      hipAngleController.setProportionalGain(20.0);
      hipAngleController.setIntegralGain(10.0);
      shoulderAngleController.setProportionalGain(0.0);
      shoulderAngleController.setIntegralGain(0.0);
      tickCounter.set(ticksForDesiredForce.getIntegerValue() + 1);

      hipSetpoint.set(0.0);
      shoulderSetpoint.set(0.0);

      makeViz(yoGraphicsListRegistries);
   }

   private void makeViz(YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      String listName = getClass().getSimpleName();

      YoGraphicPosition comPositionYoGraphic = new YoGraphicPosition("CoM", comViz, 0.05, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistries.registerYoGraphic(listName, comPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, comPositionYoGraphic.createArtifact());

      YoGraphicPosition icpPositionYoGraphic = new YoGraphicPosition("ICP", icpViz, 0.05, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistries.registerYoGraphic(listName, icpPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, icpPositionYoGraphic.createArtifact());

      YoGraphicPosition desiredCMPPositionYoGraphic = new YoGraphicPosition("DesiredCMP", desiredCMPViz, 0.05, YoAppearance.Magenta(),
                                                                            GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistries.registerYoGraphic(listName, desiredCMPPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, desiredCMPPositionYoGraphic.createArtifact());

      YoGraphicPosition footPositionYoGraphic = new YoGraphicPosition("FootLocation", footLocationViz, 0.025, YoAppearance.Black(), GraphicType.SOLID_BALL);
      yoGraphicsListRegistries.registerYoGraphic(listName, footPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, footPositionYoGraphic.createArtifact());

      YoGraphicVector desiredGRFYoGraphic = new YoGraphicVector("desiredGRFYoGraphic", footLocationViz, desiredGroundReactionViz, 0.05, YoAppearance.Orange(),
                                                                true);
      yoGraphicsListRegistries.registerYoGraphic("desiredReactionForce", desiredGRFYoGraphic);

      YoGraphicVector actualGRFYoGraphic = new YoGraphicVector("actualGRFYoGraphic", footLocationViz, groundReachtionViz, 0.05, YoAppearance.DarkGreen(), true);
      yoGraphicsListRegistries.registerYoGraphic("actualReactionForce", actualGRFYoGraphic);

      YoGraphicVector hipToFootPositionVectorYoGraphic = new YoGraphicVector("hipToFootPositionVector", hipLocationViz, hipToFootDirectionViz, 1.0,
                                                                             YoAppearance.Red(), true);
      yoGraphicsListRegistries.registerYoGraphic("hipToFootPositionVector", hipToFootPositionVectorYoGraphic);

      YoGraphicVector hipAxisVectorYoGraphic = new YoGraphicVector("hipAxis", hipLocationViz, hipAxisViz, 0.4, YoAppearance.Red(), true);
      yoGraphicsListRegistries.registerYoGraphic("hipAxis", hipAxisVectorYoGraphic);

      YoGraphicVector shoulderAxisVectorYoGraphic = new YoGraphicVector("shoulderAxis", shoulderLocationViz, shoulderAxisViz, 0.4, YoAppearance.Red(), true);
      yoGraphicsListRegistries.registerYoGraphic("shoulderAxis", shoulderAxisVectorYoGraphic);
   }

   @Override
   public void doControl()
   {
      skippy.computeComAndICP(com, comVelocity, icp, angularMomentum);
      skippy.computeFootContactForce(groundReaction);
      footLocation.set(skippy.computeFootLocation());
      cmpFromIcpDynamics(icp, footLocation, desiredCMP);

      if (tickCounter.getIntegerValue() > ticksForDesiredForce.getIntegerValue())
      {
         double qHip = skippy.getHipJoint().getQ();
         double q_dHip = skippy.getHipJoint().getQ() > 0.0 ? hipSetpoint.getDoubleValue() : -hipSetpoint.getDoubleValue();
         double hipSetpointFeedback = kAngle.getDoubleValue() * (qHip - q_dHip);
         double shoulderSetpointFeedback = kAngle.getDoubleValue() * (skippy.getShoulderJoint().getQ() - shoulderSetpoint.getDoubleValue());

         RotationMatrix rotationMatrix = new RotationMatrix();
         skippy.getRootJoints().get(0).getRotationToWorld(rotationMatrix);
         double yaw = rotationMatrix.getYaw();
         rotationMatrix.setYawPitchRoll(yaw, 0.0, 0.0);
         Point3D angleFeedback = new Point3D(hipSetpointFeedback, shoulderSetpointFeedback, 0.0);
         rotationMatrix.invert();
         rotationMatrix.transform(angleFeedback);

         desiredCMP.setY(desiredCMP.getY() - kMomentum.getDoubleValue() * angularMomentum.getX() + angleFeedback.getX());
         desiredCMP.setX(desiredCMP.getX() + kMomentum.getDoubleValue() * angularMomentum.getY() + angleFeedback.getY());

         desiredGroundReaction.sub(com, desiredCMP);
         desiredGroundReaction.normalize();
         double reactionModulus = Math.abs(skippy.getGravity()) * skippy.getMass() / desiredGroundReaction.getZ();
         desiredGroundReaction.scale(reactionModulus);
         tickCounter.set(0);
      }
      tickCounter.increment();

      skippy.getHipJoint().getTranslationToWorld(worldToHip);
      skippy.getShoulderJoint().getTranslationToWorld(worldToShoulder);
      skippy.getShoulderJointAxis(shoulderAxis);
      skippy.getHipJointAxis(hipAxis);

      // hip specific:
      hipToFootDirection.sub(footLocation, worldToHip);
      hipToFootDirection.normalize();
      double balanceTorque = computeJointTorque(desiredGroundReaction, hipToFootDirection, hipAxis);
      double angleFeedback = computeAngleFeedbackHip();

      if (Double.isNaN(angleFeedback + balanceTorque))
         skippy.getHipJoint().setTau(0.0);
      else
         skippy.getHipJoint().setTau(angleFeedback + balanceTorque);

      // shoulder specific:
      shoulderToFootDirection.sub(footLocation, worldToShoulder);
      shoulderToFootDirection.normalize();
      balanceTorque = -computeJointTorque(desiredGroundReaction, hipToFootDirection, shoulderAxis);
      angleFeedback = -computeAngleFeedbackShoulder();

      if (Double.isNaN(angleFeedback + balanceTorque))
         skippy.getShoulderJoint().setTau(0.0);
      else
         skippy.getShoulderJoint().setTau(angleFeedback + balanceTorque);

      updateViz();
   }

   /**
    * CMP computed from ICP and CMP coupled dynamics according to:
    * CMP = ICP + kCapture * (ICP - foot)
    */
   private void cmpFromIcpDynamics(FramePoint3D icp, FramePoint3D footLocation, FramePoint3D desiredCMPToPack)
   {
      FrameVector3D icpToFoot = new FrameVector3D();
      icpToFoot.sub(icp, footLocation);
      desiredCMPToPack.scaleAdd(kCapture.getDoubleValue(), icpToFoot, icp);
      desiredCMPToPack.setZ(0.0);
   }

   private double computeJointTorque(FrameVector3D groundReactionForce, FrameVector3D jointToFoot, FrameVector3D jointAxis)
   {
      FrameVector3D torque = new FrameVector3D(worldFrame);
      torque.cross(jointToFoot, groundReactionForce);
      return -jointAxis.dot(torque);
   }

   private void updateViz()
   {
      comViz.set(com);
      icpViz.set(icp);
      desiredCMPViz.set(desiredCMP);
      footLocationViz.set(footLocation);
      desiredGroundReactionViz.set(desiredGroundReaction);
      groundReachtionViz.set(groundReaction);
      hipLocationViz.set(worldToHip);
      shoulderLocationViz.set(worldToShoulder);
      hipToFootDirectionViz.set(hipToFootDirection);
      hipAxisViz.set(hipAxis);
      shoulderAxisViz.set(shoulderAxis);
      angularMomentumViz.set(angularMomentum);
   }

   /**
    * Controller on the angle between desired and achieved ground reaction force in the hip plane.
    */
   private double computeAngleFeedbackHip()
   {
      double hipAngleDifference = computeAngleDifferenceInPlane(desiredGroundReaction, groundReaction, hipAxis);
      return hipAngleController.compute(hipAngleDifference, 0.0, 0.0, 0.0, dt);
   }

   /**
    * Controller on the angle between desired and achieved ground reaction force in the shoulder plane.
    */
   private double computeAngleFeedbackShoulder()
   {
      double shoulderAngleDifference = computeAngleDifferenceInPlane(desiredGroundReaction, groundReaction, shoulderAxis);
      return shoulderAngleController.compute(shoulderAngleDifference, 0.0, 0.0, 0.0, dt);
   }

   private double computeAngleDifferenceInPlane(FrameVector3D vectorA, FrameVector3D vectorB, FrameVector3D planeNormal)
   {
      FrameVector3D projectedVectorA = new FrameVector3D();
      FrameVector3D projectedVectorB = new FrameVector3D();
      projectVectorInPlane(vectorA, planeNormal, projectedVectorA);
      projectVectorInPlane(vectorB, planeNormal, projectedVectorB);

      FrameVector3D crosProduct = new FrameVector3D();
      crosProduct.cross(planeNormal, projectedVectorA);
      double sign = Math.signum(crosProduct.dot(projectedVectorB));

      double angle = sign * projectedVectorA.angle(projectedVectorB);
      if (Double.isNaN(angle))
         return 0.0;
      return angle;
   }

   private void projectVectorInPlane(FrameVector3D vectorToProject, FrameVector3D planeNormal, FrameVector3D projectionToPack)
   {
      double modulus = vectorToProject.dot(planeNormal);
      projectionToPack.set(planeNormal);
      projectionToPack.scale(-modulus);
      projectionToPack.add(vectorToProject);
   }
}
