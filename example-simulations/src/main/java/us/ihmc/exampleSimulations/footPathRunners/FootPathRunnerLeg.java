package us.ihmc.exampleSimulations.footPathRunners;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootPathRunnerLeg
{
   private final double footMass = 0.03;
   private final double footRadiusOfGyration = 0.05;
   private final double hipWidth = 0.2;

   private final SliderJoint footXJoint, footYJoint, footZJoint;
   private final PinJoint footPitchJoint, footRollJoint;
   private final SliderJoint footSliderJoint;

   private final GroundContactPoint foot;
   private final YoFramePoseUsingYawPitchRoll desiredFootPoseInBodyFrame;
   private final PDController footXController, footYController, footZController, footPitchController, footRollController;

   private final YoDouble footSliderSpring, footSliderDamper;
   private final YoDouble maxForce, maxTorque;
   private final YoDouble maxSliderForceWhenPositiveWork;

   public FootPathRunnerLeg(RobotSide robotSide, Robot robot, Joint rootJoint, ReferenceFrame bodyReferenceFrame)
   {
      YoRegistry registry = robot.getRobotsYoRegistry();
      String sideName = robotSide.getCamelCaseNameForStartOfExpression();

      footXJoint = new SliderJoint(sideName + "FootX", new Vector3D(0.0, robotSide.negateIfRightSide(hipWidth/2.0), 0.0), robot, Axis3D.X);
      footXJoint.setLink(createNullLink());
      rootJoint.addJoint(footXJoint);

      footYJoint = new SliderJoint(sideName + "FootY", new Vector3D(), robot, Axis3D.Y);
      footYJoint.setLink(createNullLink());
      footXJoint.addJoint(footYJoint);

      footZJoint = new SliderJoint(sideName + "FootZ", new Vector3D(), robot, Axis3D.Z);
      footZJoint.setLink(createNullLink());
      footYJoint.addJoint(footZJoint);

      footPitchJoint = new PinJoint(sideName + "FootPitch", new Vector3D(), robot, Axis3D.Y);
      footPitchJoint.setLink(createNullLink());
      footZJoint.addJoint(footPitchJoint);

      footRollJoint = new PinJoint(sideName + "FootRoll", new Vector3D(), robot, Axis3D.X);
      Link footRollLink = new Link(sideName + "FootRollLink");
      footRollLink.setMassAndRadiiOfGyration(0.001, 0.01, 0.01, 0.01);

      Graphics3DObject footRollLinkGraphics = new Graphics3DObject();
      footRollLinkGraphics.translate(0.0, 0.0, 0.1);
      footRollLinkGraphics.addCylinder(0.01, 0.05, YoAppearance.Black());
      footRollLink.setLinkGraphics(footRollLinkGraphics);
      footRollJoint.setLink(footRollLink);

      footPitchJoint.addJoint(footRollJoint);

      footSliderJoint = new SliderJoint(sideName + "FootSlider", new Vector3D(), robot, Axis3D.Z);
      Link footLink = new Link("foot");

      footLink.setMassAndRadiiOfGyration(footMass, footRadiusOfGyration, footRadiusOfGyration, footRadiusOfGyration);
      Graphics3DObject footGraphics = new Graphics3DObject();
      footGraphics.addCoordinateSystem(0.2);
      footGraphics.addSphere(0.02, YoAppearance.Red());
      footGraphics.addCylinder(1.4, 0.005, YoAppearance.Yellow());

      footLink.setLinkGraphics(footGraphics);
      footSliderJoint.setLink(footLink);
      footRollJoint.addJoint(footSliderJoint);

      foot = new GroundContactPoint("gc_" + sideName + "Foot", robot);
      footSliderJoint.addGroundContactPoint(foot);

      //      setFootPosition(0.0, 0.0, -0.3, 0.0, 0.0);

      double footKpTranslation = 20000.0;
      double footKdTranslation = 100.0;

      desiredFootPoseInBodyFrame = new YoFramePoseUsingYawPitchRoll(sideName + "DesiredFoot", bodyReferenceFrame, registry);
      footXController = new PDController(sideName + "FootX", registry);
      footXController.setProportionalGain(footKpTranslation);
      footXController.setDerivativeGain(footKdTranslation);

      footYController = new PDController(sideName + "FootY", registry);
      footYController.setProportionalGain(footKpTranslation);
      footYController.setDerivativeGain(footKdTranslation);

      footZController = new PDController(sideName + "FootZ", registry);
      footZController.setProportionalGain(footKpTranslation);
      footZController.setDerivativeGain(footKdTranslation);

      double footKpRotation = 1000.0;
      double footKdRotation = 10.0;

      footPitchController = new PDController(sideName + "FootPitch", registry);
      footPitchController.setProportionalGain(footKpRotation);
      footPitchController.setDerivativeGain(footKdRotation);

      footRollController = new PDController(sideName + "FootRoll", registry);
      footRollController.setProportionalGain(footKpRotation);
      footRollController.setDerivativeGain(footKdRotation);

      footSliderSpring = new YoDouble(sideName + "FootSliderSpring", registry);
      footSliderDamper = new YoDouble(sideName + "FootSliderDamper", registry);

      footSliderSpring.set(625.0);
      footSliderDamper.set(10.0);

      maxForce = new YoDouble(sideName + "MaxForce", registry);
      maxTorque = new YoDouble(sideName + "MaxTorque", registry);

      maxForce.set(200.0);
      maxTorque.set(10.0);
      
      maxSliderForceWhenPositiveWork = new YoDouble(sideName + "MaxSliderForceWhenPositiveWork", registry);
      maxSliderForceWhenPositiveWork.set(100.0);
   }

   public void setFootPosition(double footX, double footY, double footZ, double footPitch, double footRoll)
   {
      footXJoint.setQ(footX);
      footYJoint.setQ(footY);
      footZJoint.setQ(footZ);
      footPitchJoint.setQ(footPitch);
      footRollJoint.setQ(footRoll);
   }

   public void setActualFootLocation(Pose3DReadOnly footPoseInBodyFrame)
   {
      footXJoint.setQ(footPoseInBodyFrame.getX());
      footYJoint.setQ(footPoseInBodyFrame.getY());
      footZJoint.setQ(footPoseInBodyFrame.getZ());

      footPitchJoint.setQ(footPoseInBodyFrame.getPitch());
      footRollJoint.setQ(footPoseInBodyFrame.getRoll());
   }

   private Link createNullLink()
   {
      Link link = new Link("null");

      link.setMassAndRadiiOfGyration(0.001, 0.01, 0.01, 0.01);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      link.setLinkGraphics(linkGraphics);
      return link;
   }

   public void setDesiredFootLocation(Pose3DReadOnly desiredFootInBodyFrame)
   {
      this.desiredFootPoseInBodyFrame.set(desiredFootInBodyFrame);
   }

   public void controlFootLocation()
   {
      double tauFootX = footXController.compute(footXJoint.getQ(), desiredFootPoseInBodyFrame.getX(), footXJoint.getQD(), 0.0);
      tauFootX = MathTools.clamp(tauFootX, maxForce.getValue());
      footXJoint.setTau(tauFootX);

      double taufootY = footYController.compute(footYJoint.getQ(), desiredFootPoseInBodyFrame.getY(), footYJoint.getQD(), 0.0);
      taufootY = MathTools.clamp(taufootY, maxForce.getValue());
      footYJoint.setTau(taufootY);

      double taufootZ = footZController.compute(footZJoint.getQ(), desiredFootPoseInBodyFrame.getZ(), footZJoint.getQD(), 0.0);
      taufootZ = MathTools.clamp(taufootZ, maxForce.getValue());
      footZJoint.setTau(taufootZ);

      double tauFootPitch = footPitchController.computeForAngles(footPitchJoint.getQ(), desiredFootPoseInBodyFrame.getPitch(), footPitchJoint.getQD(), 0.0);
      tauFootPitch = MathTools.clamp(tauFootPitch, maxTorque.getValue());
      footPitchJoint.setTau(tauFootPitch);

      double tauFootRoll = footRollController.computeForAngles(footRollJoint.getQ(), desiredFootPoseInBodyFrame.getRoll(), footRollJoint.getQD(), 0.0);
      tauFootRoll = MathTools.clamp(tauFootRoll, maxTorque.getValue());
      footRollJoint.setTau(tauFootRoll);

      double tauFootSlider = -footSliderSpring.getValue() * footSliderJoint.getQ() - footSliderDamper.getValue() * footSliderJoint.getQD();
      tauFootSlider = MathTools.clamp(tauFootSlider, maxForce.getValue());
      
      if (footSliderJoint.getQD() * tauFootSlider > 0.0)
      {
         if (tauFootSlider < -maxSliderForceWhenPositiveWork.getValue())
         {
            tauFootSlider = -maxSliderForceWhenPositiveWork.getValue();
         }
      }
      footSliderJoint.setTau(tauFootSlider);
   }
}
