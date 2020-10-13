package us.ihmc.exampleSimulations.footPathRunners;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class FootPathRunnerRobot extends Robot
{
   //TODO: Extend to 3D!
//   private final FloatingJoint rootJoint;
   private final FloatingPlanarJoint rootJoint;

   private final double bodyMass = 4.0;
   private final double bodyRadiusOfGyrationX = 0.35;
   private final double bodyRadiusOfGyrationY = 0.1;
   private final double bodyRadiusOfGyrationZ = 0.1;

   private final JointReferenceFrame bodyReferenceFrame;
   private YoGraphicReferenceFrame bodyGraphicReferenceFrame;

   private final SideDependentList<FootPathRunnerLeg> legs;

   public FootPathRunnerRobot()
   {
      super("FootPathRunner");

      rootJoint = new FloatingPlanarJoint("root", new Vector3D(), this);

      Link bodyLink = new Link("bodyLink");
      bodyLink.setMass(bodyMass);
      bodyLink.setMassAndRadiiOfGyration(bodyMass, bodyRadiusOfGyrationX, bodyRadiusOfGyrationY, bodyRadiusOfGyrationZ);

      bodyLink.addEllipsoidFromMassProperties(YoAppearance.Gold());
      rootJoint.setLink(bodyLink);

      this.addRootJoint(rootJoint);
      rootJoint.setCartesianPosition(0.0, 0.9);
      bodyReferenceFrame = new JointReferenceFrame(rootJoint);

      FootPathRunnerLeg leftLeg = new FootPathRunnerLeg(RobotSide.LEFT, this, rootJoint, bodyReferenceFrame);
      FootPathRunnerLeg rightLeg = new FootPathRunnerLeg(RobotSide.RIGHT, this, rootJoint, bodyReferenceFrame);

      legs = new SideDependentList<>(leftLeg, rightLeg);
      
      //      this.setGravity(-0.01);
      this.update();
      this.updateReferenceFrames();

      rootJoint.setVelocity(new Vector3D(20.0, 0.0, 0.0));
   }

   public void setupReferenceFrameGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      boolean useYawPitchRoll = true;
      double scale = 1.0;

      bodyGraphicReferenceFrame = new YoGraphicReferenceFrame(bodyReferenceFrame, this.getRobotsYoRegistry(), useYawPitchRoll, scale);
      yoGraphicsListRegistry.registerYoGraphic("ReferenceFrames", bodyGraphicReferenceFrame);

   }

   public void updateReferenceFrames()
   {
      bodyReferenceFrame.update();
      if (bodyGraphicReferenceFrame != null)
         bodyGraphicReferenceFrame.update();
   }

   public void setDesiredFootLocation(RobotSide robotSide, Pose3D desiredFootInBodyFrame)
   {
      FootPathRunnerLeg leg = legs.get(robotSide);
      leg.setDesiredFootLocation(desiredFootInBodyFrame);
   }

   public void controlFootLocation(RobotSide robotSide)
   {
      FootPathRunnerLeg leg = legs.get(robotSide);
      leg.controlFootLocation();
   }

   public void setActualFootLocation(RobotSide robotSide, Pose3D footPoseInBodyFrame)
   {
      FootPathRunnerLeg leg = legs.get(robotSide);
      leg.setActualFootLocation(footPoseInBodyFrame);
   }

}
