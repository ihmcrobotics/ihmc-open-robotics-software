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
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTwoLegController implements RobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble coriolisScalingFactor;
   private final YoDouble normalForceScalingFactor; 
   private final GravityCoriolisExternalWrenchMatrixCalculator gravityCompensator;
   private final GravityCoriolisExternalWrenchMatrixCalculator coriolisCompensator;
   private final SimpleTwoLeggedRobot robot;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   private final double robotMass;

   public SimpleTwoLegController(double controlDT, SimpleTwoLeggedRobot robot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robot = robot;

      coriolisScalingFactor = new YoDouble("coriolisScale", registry);
      normalForceScalingFactor = new YoDouble("normalForceScale", registry);
      coriolisScalingFactor.set(0.0);
      normalForceScalingFactor.set(1.0);
      
      gravityCompensator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemBasics.toMultiBodySystemBasics(robot.getRootBody()), false, true);
      gravityCompensator.setGravitionalAcceleration(-9.81);
      coriolisCompensator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemBasics.toMultiBodySystemBasics(robot.getRootBody()), true, true);
      coriolisCompensator.setGravitionalAcceleration(0.0);
      robotMass = TotalMassCalculator.computeSubTreeMass(robot.getRootBody());
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), robot.getRootBody());
   }

   double normalForceTorque = 0;
   double coriolisTorque = 0;
   double computedTorque = 0;
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
      coriolisCompensator.compute();
      for(int i = 0; i < robot.getOneDoFJoints().size(); i++) {
         normalForceTorque = gravityCompensator.getComputedJointTau(robot.getOneDoFJoints().get(i)).get(0);
         coriolisTorque = coriolisCompensator.getComputedJointTau(robot.getOneDoFJoints().get(i)).get(0);
         computedTorque = normalForceScalingFactor.getDoubleValue()*normalForceTorque + coriolisScalingFactor.getDoubleValue()*coriolisTorque;
         robot.getOneDoFJoints().get(i).setTau(computedTorque);
      }
      //gravityCompensator.writeComputedJointWrenches(robot.getOneDoFJoints());

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
