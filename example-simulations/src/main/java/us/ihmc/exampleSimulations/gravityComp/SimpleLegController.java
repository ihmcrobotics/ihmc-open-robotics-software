package us.ihmc.exampleSimulations.gravityComp;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleLegController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCompensator;
   private final SimpleLegRobot robot;

   private final double robotMass;

   public SimpleLegController(double controlDT, SimpleLegRobot robot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robot = robot;

      gravityCompensator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemBasics.toMultiBodySystemBasics(robot.getRootBody()), false, true);
      gravityCompensator.setGravitionalAcceleration(-9.81);
      robotMass = TotalMassCalculator.computeSubTreeMass(robot.getRootBody());
   }

   @Override
   public void doControl()
   {
      robot.readSimulationSensorData();

      FrameVector3D footForce = new FrameVector3D(worldFrame, 0.0, 0.0, robotMass * 9.81);

      gravityCompensator.setExternalWrenchesToZero();
      gravityCompensator.getExternalWrench(robot.getFoot()).getLinearPart().setMatchingFrame(footForce);
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
