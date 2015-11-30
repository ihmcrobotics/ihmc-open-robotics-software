package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class FourBarLinkageController implements RobotController
{
   private String name;
   private YoVariableRegistry registry = new YoVariableRegistry("controllerRegistry");

   private final YoFramePoint yoPositionFix;
   private final YoFrameVector yoVelocityFix;
   private final YoFrameVector yoTauFix;
   private final DoubleYoVariable fix_kp = new DoubleYoVariable("fixGain", registry);
   private final DoubleYoVariable fix_kd = new DoubleYoVariable("fixDamping", registry);
   private final DoubleYoVariable positionErrorMagnitude = new DoubleYoVariable("positionErrorMagnitude", registry);

   // temp variables
   private Point3d posEfpFix = new Point3d();
   private Point3d posEfpFixDesired;
   private Point3d velEfpFix = new Point3d();
   private Point3d velEfpFixDesired = new Point3d();
   private Vector3d pTau = new Vector3d();
   private Vector3d dTau = new Vector3d();
   private Vector3d totalTauFix = new Vector3d();

   public FourBarLinkageController(FourBarLinkageRobot robot, String name, FourBarLinkageParameters fourBarLinkageParameters)
   {
      this.name = name;

      ExternalForcePoint efpToFixJoint1InWorld = new ExternalForcePoint(name + "Joint1_efp", new Vector3d(), robot);
      robot.getJoint(1).addExternalForcePoint(efpToFixJoint1InWorld);
      yoPositionFix = efpToFixJoint1InWorld.getYoPosition();
      posEfpFixDesired = new Point3d(fourBarLinkageParameters.linkageLength_1, 0.0, 0.0);
      yoVelocityFix = efpToFixJoint1InWorld.getYoVelocity();
      yoTauFix = efpToFixJoint1InWorld.getYoForce();
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
      // Fix Joint 2 with respect to world

      yoPositionFix.get(posEfpFix);
      pTau.sub(posEfpFix, posEfpFixDesired);
      positionErrorMagnitude.set(pTau.length());
      pTau.scale(-fix_kp.getDoubleValue());

      yoVelocityFix.get(velEfpFix);
      dTau.sub(velEfpFix, velEfpFixDesired);
      dTau.scale(-fix_kd.getDoubleValue());

      totalTauFix.add(pTau, dTau);
      yoTauFix.set(totalTauFix);
   }
}
