package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class FourBarLinkageSimpleClosedLoopConstraintController implements RobotController
{
   private final YoFramePoint yoPositionJoint0, yoPositionJoint4;
   private final YoFrameVector yoVelocityJoint0, yoVelocityJoint4;
   private final YoFrameVector yoForceJoint0, yoForceJoint4;
   private final YoFrameVector yoMomentJoint0, yoMomentJoint4;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable constraintGain = new DoubleYoVariable("constraintGain", registry);
   private final DoubleYoVariable constraintDamping = new DoubleYoVariable("constraintDamping", registry);
   private final DoubleYoVariable positionErrorMagnitude = new DoubleYoVariable("positionErrorMagnitude", registry);

   private final DoubleYoVariable qdJoint1, qdJoint2, qdJoint3;
   private final DoubleYoVariable closureJointDamping;

   // Temp variables:
   private Point3d posJoint0 = new Point3d();
   private Point3d posJoint4 = new Point3d();
   private Vector3d velJoint0 = new Vector3d();
   private Vector3d velJoint4 = new Vector3d();
   private Vector3d springForce0 = new Vector3d();
   private Vector3d dampingForce4 = new Vector3d();
   private Vector3d newForce0 = new Vector3d();
   private Vector3d newForce4 = new Vector3d();
   private Vector3d newMoment0 = new Vector3d();
   private Vector3d newMoment4 = new Vector3d();
   private double angleChangeAtLoopClosure;

   public FourBarLinkageSimpleClosedLoopConstraintController(FourBarLinkageRobot robot)
   {
      ExternalForcePoint efpJoint0 = robot.getEfpJoint0();
      ExternalForcePoint efpJoint4 = robot.getEfpJoint4();

      yoPositionJoint0 = efpJoint0.getYoPosition();
      yoPositionJoint4 = efpJoint4.getYoPosition();

      yoVelocityJoint0 = efpJoint0.getYoVelocity();
      yoVelocityJoint4 = efpJoint4.getYoVelocity();

      yoForceJoint0 = efpJoint0.getYoForce();
      yoForceJoint4 = efpJoint4.getYoForce();

      qdJoint1 = robot.getJoint(1).getQD();
      qdJoint2 = robot.getJoint(2).getQD();
      qdJoint3 = robot.getJoint(3).getQD();

      yoMomentJoint0 = efpJoint0.getYoMoment();
      yoMomentJoint4 = efpJoint4.getYoMoment();

      closureJointDamping = new DoubleYoVariable(robot.getName() + "ClosureJointDamping", registry);

      initialize();
   }

   @Override
   public void initialize()
   {
      constraintGain.set(1000.0);
      constraintDamping.set(100.0);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      yoPositionJoint0.get(posJoint0);
      yoPositionJoint4.get(posJoint4);

      yoVelocityJoint0.get(velJoint0);
      yoVelocityJoint4.get(velJoint4);

      springForce0.sub(posJoint4, posJoint0);
      positionErrorMagnitude.set(springForce0.length());
      springForce0.scale(constraintGain.getDoubleValue());

      dampingForce4.sub(velJoint4, velJoint0);
      dampingForce4.scale(constraintDamping.getDoubleValue());

      newForce0.add(springForce0, dampingForce4);
      newForce4.scale(-1.0, newForce0);

      yoForceJoint0.set(newForce0);
      yoForceJoint4.set(newForce4);

      angleChangeAtLoopClosure = -qdJoint1.getDoubleValue() - qdJoint2.getDoubleValue() - qdJoint3.getDoubleValue();
      newMoment0.setY(-angleChangeAtLoopClosure * closureJointDamping.getDoubleValue());
      newMoment4.setY(angleChangeAtLoopClosure * closureJointDamping.getDoubleValue());

      yoMomentJoint0.set(newMoment0);
      yoMomentJoint4.set(newMoment4);
   }

   public void setClosureJointDamping(double damping)
   {
      closureJointDamping.set(damping);
   }
}
