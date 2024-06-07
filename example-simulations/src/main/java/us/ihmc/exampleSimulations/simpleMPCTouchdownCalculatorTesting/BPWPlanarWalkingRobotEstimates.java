package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting.MPC.BPWPlanarWalkerParameters;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.screwTheory.WholeBodyAngularVelocityCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BPWPlanarWalkingRobotEstimates
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BPWPLanarWalkingRobot robot;

   private final double mass;
   private final FramePoint3DReadOnly centerOfMassPosition;
   private final FrameVector3DReadOnly centerOfMassVelocity;
   private final YoFrameVector3D currentCentroidalAngularMomentum = new YoFrameVector3D("currentCentroidalAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D capturePoint = new YoFramePoint3D("capturePoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D acp = new YoFramePoint3D("ACP", ReferenceFrame.getWorldFrame(), registry);
   private final SideDependentList<YoFramePose3D> footPoses = new SideDependentList<>();


   private final MovingReferenceFrame centerOfMassControlFrame;
   private final MovingZUpFrame centerOfMassControlZUpFrame;


   private final WholeBodyAngularVelocityCalculator angularVelocityCalculator;

   private final BPWPlanarWalkerParameters parameters;

   public BPWPlanarWalkingRobotEstimates(BPWPLanarWalkingRobot robot, BPWPlanarWalkerParameters parameters, YoRegistry parentRegistry)
   {
      this.robot = robot;
      this.parameters = parameters;

      mass = robot.getMass();
      centerOfMassPosition = robot.getCenterOfMassPoint();
      centerOfMassVelocity = robot.getCenterOfMassVelocity();

      centerOfMassControlFrame = new MovingReferenceFrame("centerOfMassControlFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            //            pelvisFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), pelvisTwist);
            //            pelvisTwist.changeFrame(centerOfMassFrame); // FIXME we really want the rotation about the center of mass, relative to the world.

            twistRelativeToParentToPack.getLinearPart().setMatchingFrame(getCenterOfMassVelocity());
            //            twistRelativeToParentToPack.getAngularPart().setMatchingFrame(pelvisTwist.getAngularPart());
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(getCenterOfMass());
            //transformToParent.getRotation().set(desiredOrientation);
         }
      };

      centerOfMassControlZUpFrame = new MovingZUpFrame(centerOfMassControlFrame, "centerOfMassControlZUpFrame");

      for (RobotSide robotSide : RobotSide.values)
      {
         YoFramePose3D footPosition = new YoFramePose3D(robotSide.getCamelCaseName() + "FootPose", centerOfMassControlZUpFrame, registry);

         footPoses.put(robotSide, footPosition);
      }

      angularVelocityCalculator = new WholeBodyAngularVelocityCalculator(robot.getCenterOfMassFrame(), robot.getRootBody().subtreeArray());

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      //
      centerOfMassControlFrame.update();
      centerOfMassControlZUpFrame.update();

      // Update angular momentum
      angularVelocityCalculator.compute();
      currentCentroidalAngularMomentum.setMatchingFrame(angularVelocityCalculator.getAngularMomentum());

      // Update capture point and ACP
      capturePoint.setFromReferenceFrame(robot.getCenterOfMassFrame());
      capturePoint.scaleAdd(1.0 / parameters.getOmegaX().getDoubleValue(),
                            centerOfMassVelocity,
                            centerOfMassPosition);

      double mh = getTotalMass() * parameters.getDesiredWalkingHeight();
      double wmh = parameters.getOmegaX().getDoubleValue()* mh;

      acp.set(capturePoint);
      acp.addX(1.0 / wmh * getCentroidalAngularMomentum().getY());
      acp.addY(-1.0 / wmh * getCentroidalAngularMomentum().getX());

      capturePoint.setZ(0.0);

      // Update foot poses
      for (RobotSide robotSide : RobotSide.values)
         footPoses.get(robotSide).setFromReferenceFrame(robot.getFootFrame(robotSide));
   }

   public double getTotalMass()
   {
      return mass;
   }

   public double getGravity()
   {
      return 9.81;
   }

   public FramePoint3DReadOnly getCenterOfMass()
   {
      return centerOfMassPosition;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return centerOfMassVelocity;
   }

   public MovingZUpFrame getCenterOfMassControlZUPFrame()
   {
      return centerOfMassControlZUpFrame;
   }

   public YoFrameVector3D getCentroidalAngularMomentum()
   {
      return currentCentroidalAngularMomentum;
   }

   public YoFramePoint3D getCapturePoint()
   {
      return capturePoint;
   }

   public YoFramePoint3D getACP()
   {
      return acp;
   }

   public FramePoint3DReadOnly getFootPosition(RobotSide robotSide)
   {
      return footPoses.get(robotSide).getPosition();
   }
}
