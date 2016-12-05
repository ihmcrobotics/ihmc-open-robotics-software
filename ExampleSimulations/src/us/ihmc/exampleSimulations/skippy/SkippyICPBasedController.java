package us.ihmc.exampleSimulations.skippy;

import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class SkippyICPBasedController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SkippyRobot skippy;

   private final double dt;

   private final DoubleYoVariable kCapture = new DoubleYoVariable("kCapture", registry);
   private final DoubleYoVariable kMomentum = new DoubleYoVariable("kMomentum", registry);

   private final IntegerYoVariable tickCounter = new IntegerYoVariable("tickCounter", registry);
   private final IntegerYoVariable ticksForDesiredForce = new IntegerYoVariable("ticksForDesiredForce", registry);
   private final DoubleYoVariable angleBetweenDesiredAndActual = new DoubleYoVariable("angleBetweenDesiredAndActual", registry);
   private final PIDController angleController = new PIDController("angleController", registry);
   private final FrameVector angularMomentum = new FrameVector(worldFrame);

   private final FramePoint com = new FramePoint(worldFrame);
   private final FrameVector comVelocity = new FrameVector(worldFrame);
   private final FramePoint icp = new FramePoint(worldFrame);
   private final FramePoint footLocation = new FramePoint(worldFrame);
   private final FramePoint desiredCMP = new FramePoint(worldFrame);
   private final FrameVector desiredGroundReaction = new FrameVector(worldFrame);
   private final FrameVector groundReaction = new FrameVector(worldFrame);
   private final FrameVector worldToHip = new FrameVector(worldFrame);
   private final FrameVector hipToFootDirection = new FrameVector(worldFrame);
   private final FrameVector hipAxis = new FrameVector(worldFrame);

   private final YoFramePoint comViz = new YoFramePoint("CoM", worldFrame, registry);
   private final YoFramePoint icpViz = new YoFramePoint("ICP", worldFrame, registry);
   private final YoFramePoint desiredCMPViz = new YoFramePoint("DesiredCMP", worldFrame, registry);
   private final YoFramePoint footLocationViz = new YoFramePoint("FootLocation", worldFrame, registry);
   private final YoFramePoint hipLocationViz = new YoFramePoint("HipLocation", worldFrame, registry);
   private final YoFrameVector desiredGroundReactionViz = new YoFrameVector("DesiredGroundReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector groundReachtionViz = new YoFrameVector("GroundReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootDirectionViz = new YoFrameVector("HipToFootDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipAxisViz = new YoFrameVector("HipAxis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector angularMomentumViz = new YoFrameVector("AngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   public SkippyICPBasedController(SkippyRobot skippy, double dt, YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      this.skippy = skippy;
      this.dt = dt;

      kCapture.set(7.5);
      kMomentum.set(0.3);

      ticksForDesiredForce.set(100);
      angleController.setProportionalGain(20.0);
      angleController.setIntegralGain(10.0);
      tickCounter.set(ticksForDesiredForce.getIntegerValue());

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

      YoGraphicPosition desiredCMPPositionYoGraphic = new YoGraphicPosition("DesiredCMP", desiredCMPViz, 0.05, YoAppearance.Magenta(), GraphicType.BALL_WITH_ROTATED_CROSS);
      yoGraphicsListRegistries.registerYoGraphic(listName, desiredCMPPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, desiredCMPPositionYoGraphic.createArtifact());

      YoGraphicPosition footPositionYoGraphic = new YoGraphicPosition("FootLocation", footLocationViz, 0.025, YoAppearance.Black(), GraphicType.SOLID_BALL);
      yoGraphicsListRegistries.registerYoGraphic(listName, footPositionYoGraphic);
      yoGraphicsListRegistries.registerArtifact(listName, footPositionYoGraphic.createArtifact());

      YoGraphicVector desiredGRFYoGraphic = new YoGraphicVector("desiredGRFYoGraphic", footLocationViz, desiredGroundReactionViz, 0.05, YoAppearance.Orange(), true);
      yoGraphicsListRegistries.registerYoGraphic("desiredReactionForce", desiredGRFYoGraphic);

      YoGraphicVector actualGRFYoGraphic = new YoGraphicVector("actualGRFYoGraphic", footLocationViz, groundReachtionViz, 0.05, YoAppearance.DarkGreen(), true);
      yoGraphicsListRegistries.registerYoGraphic("actualReactionForce", actualGRFYoGraphic);

      YoGraphicVector hipToFootPositionVectorYoGraphic = new YoGraphicVector("hipToFootPositionVector", hipLocationViz, hipToFootDirectionViz, 1.0, YoAppearance.Red(), true);
      yoGraphicsListRegistries.registerYoGraphic("hipToFootPositionVector", hipToFootPositionVectorYoGraphic);

      YoGraphicVector hipAxisVectorYoGraphic = new YoGraphicVector("hipAxis", hipLocationViz, hipAxisViz, 0.2, YoAppearance.Red(), true);
      yoGraphicsListRegistries.registerYoGraphic("hipAxis", hipAxisVectorYoGraphic);
   }

   @Override
   public void doControl()
   {
      skippy.computeComAndICP(com, comVelocity, icp, angularMomentum);
      skippy.computeFootContactForce(groundReaction.getVector());
      footLocation.set(skippy.computeFootLocation());
      cmpFromIcpDynamics(icp, footLocation, desiredCMP);

      if (tickCounter.getIntegerValue() > ticksForDesiredForce.getIntegerValue())
      {
         desiredCMP.setY(desiredCMP.getY() - kMomentum.getDoubleValue() * angularMomentum.getX());
         
         desiredGroundReaction.sub(com, desiredCMP);
         desiredGroundReaction.normalize();
         double reactionModulus = Math.abs(skippy.getGravity()) * skippy.getMass() / desiredGroundReaction.getZ();
         desiredGroundReaction.scale(reactionModulus);
         tickCounter.set(0);
      }
      tickCounter.increment();

      skippy.getHipJoint().getTranslationToWorld(worldToHip.getVector());
      hipToFootDirection.sub(footLocation, worldToHip);
      hipToFootDirection.normalize();

      skippy.getHipJointAxis(hipAxis);
      double balanceTorque = computeJointTorque(desiredGroundReaction, hipToFootDirection, hipAxis);
      double angleFeedback = computeAngleFeedback();
      skippy.getHipJoint().setTau(angleFeedback + balanceTorque);

      updateViz();
   }

   /**
    * CMP computed from ICP and CMP coupled dynamics according to:
    * CMP = ICP + kCapture * (ICP - foot)
    */
   private void cmpFromIcpDynamics(FramePoint icp, FramePoint footLocation, FramePoint desiredCMPToPack)
   {
      FrameVector icpToFoot = new FrameVector();
      icpToFoot.sub(icp, footLocation);
      desiredCMPToPack.scaleAdd(kCapture.getDoubleValue(), icpToFoot, icp);
      desiredCMPToPack.setZ(0.0);
   }

   private double computeJointTorque(FrameVector groundReactionForce, FrameVector jointToFoot, FrameVector jointAxis)
   {
      FrameVector torque = new FrameVector(worldFrame);
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
      hipToFootDirectionViz.set(hipToFootDirection);
      hipAxisViz.set(hipAxis);
      angularMomentumViz.set(angularMomentum);
   }

   /**
    * Controller on the angle between desired and achieved ground reaction force.
    */
   private double computeAngleFeedback()
   {
      double angleBetweenDesiredAndActual = desiredGroundReaction.angle(groundReaction);

      double yDifference = groundReaction.getY() - desiredGroundReaction.getY();
      angleBetweenDesiredAndActual = Math.signum(yDifference) * angleBetweenDesiredAndActual;

      this.angleBetweenDesiredAndActual.set(angleBetweenDesiredAndActual);
      if (Double.isNaN(angleBetweenDesiredAndActual))
         return 0.0;
      return angleController.compute(-angleBetweenDesiredAndActual, 0.0, 0.0, 0.0, dt);
   }

}
