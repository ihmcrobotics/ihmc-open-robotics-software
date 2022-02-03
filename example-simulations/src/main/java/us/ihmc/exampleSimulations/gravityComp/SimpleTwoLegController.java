package us.ihmc.exampleSimulations.gravityComp;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleTwoLegController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCompensator;
   private final SimpleTwoLeggedRobot robot;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   private final double robotMass;

   public SimpleTwoLegController(double controlDT, SimpleTwoLeggedRobot robot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robot = robot;

      gravityCompensator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemBasics.toMultiBodySystemBasics(robot.getRootBody()), false, true);
      gravityCompensator.setGravitionalAcceleration(-9.81);
      robotMass = TotalMassCalculator.computeSubTreeMass(robot.getRootBody());
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), robot.getRootBody());
   }

   @Override
   public void doControl()
   {
      robot.readSimulationSensorData();
      centerOfMassFrame.update();
//      FramePoint3D comFrame = new FramePoint3D(centerOfMassFrame);
//      comFrame.changeFrame(ReferenceFrameTools.getWorldFrame());
//      comFrame.setZ(0.0);

      gravityCompensator.setExternalWrenchesToZero();

      FrameVector3D footForce = new FrameVector3D(worldFrame, 0.0, 0.0, robotMass * 9.81 / 2);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint3D comFrame = new FramePoint3D(centerOfMassFrame);
         comFrame.changeFrame(ReferenceFrameTools.getWorldFrame());
         comFrame.setZ(0.0);
         comFrame.changeFrame(robot.getFoot(robotSide).getBodyFixedFrame());
         comFrame.setY(0);

         footForce.changeFrame(robot.getFoot(robotSide).getBodyFixedFrame());

         gravityCompensator.getExternalWrench(robot.getFoot(robotSide)).set(null, footForce, comFrame);
      }
      gravityCompensator.compute();
      gravityCompensator.writeComputedJointWrenches(robot.getOneDoFJoints());

      robot.writeControllerOutput();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void initialize()
   {
      // Do nothing, it's not being called from SCS anyway
   }
}
