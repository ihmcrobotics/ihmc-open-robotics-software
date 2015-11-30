package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class PinFourBarJointInWorldController implements RobotController
{
   private String name;
   private YoVariableRegistry registry = new YoVariableRegistry("PinFourBarJointInWorldControllerRegistry");

   private final YoFramePoint yoJointEfpPosition;
   private final YoFrameVector yoJointEfpVelocity;
   private final YoFrameVector yoJointEfpForce;
   private final DoubleYoVariable fix_kp;
   private final DoubleYoVariable fix_kd;
   private final DoubleYoVariable positionErrorMagnitude;

   private final YoFramePoint yoDesiredJointCartesianPosition;

   // temp variables
   private Point3d jointEfpPosition = new Point3d();
   private Point3d jointEfpVelocity = new Point3d();
   private Vector3d positionForce = new Vector3d();
   private Vector3d velocityForce = new Vector3d();
   private Vector3d totalForce = new Vector3d();
   private Point3d desiredJointCartesianPosition = new Point3d();

   public PinFourBarJointInWorldController(FourBarLinkageRobot robot, String name, int jointToPin, YoFramePoint yoDesiredJointCartesianPosition)
   {
      this.name = name;

      ExternalForcePoint efpToPinToJoint = new ExternalForcePoint(name + "EfpJoint" + jointToPin, robot);
      robot.getJoint(jointToPin).addExternalForcePoint(efpToPinToJoint);
      yoJointEfpPosition = efpToPinToJoint.getYoPosition();
      yoJointEfpVelocity = efpToPinToJoint.getYoVelocity();
      yoJointEfpForce = efpToPinToJoint.getYoForce();

      fix_kp = new DoubleYoVariable(name + "FixGain", registry);
      fix_kd = new DoubleYoVariable(name + "FixDamping", registry);
      positionErrorMagnitude = new DoubleYoVariable(name + "PositionErrorMagnitude", registry);

      this.yoDesiredJointCartesianPosition = yoDesiredJointCartesianPosition;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   public void setDamping(double damping)
   {
      fix_kd.set(damping);
   }

   public void setGain(double gain)
   {
      fix_kp.set(gain);
   }

   @Override
   public void doControl()
   {
      yoJointEfpPosition.get(jointEfpPosition);
      yoDesiredJointCartesianPosition.get(desiredJointCartesianPosition);
      positionForce.sub(jointEfpPosition, desiredJointCartesianPosition);
      positionErrorMagnitude.set(positionForce.length());
      positionForce.scale(-fix_kp.getDoubleValue());

      yoJointEfpVelocity.get(jointEfpVelocity);
      velocityForce.scale(-fix_kd.getDoubleValue());

      totalForce.add(positionForce, velocityForce);
      yoJointEfpForce.set(totalForce);
   }
}
