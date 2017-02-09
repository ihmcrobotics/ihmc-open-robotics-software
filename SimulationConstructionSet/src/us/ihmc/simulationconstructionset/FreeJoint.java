package us.ihmc.simulationconstructionset;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */


public class FreeJoint extends Joint
{
   /**
    *
    */
   private static final long serialVersionUID = 5257393793242028735L;
   @SuppressWarnings("unused")
   private String xName, yName, zName, yawName, rollName, pitchName;
   private DoubleYoVariable xVar, yVar, zVar, yawVar, rollVar, pitchVar;

   private Vector3d vTranslate = new Vector3d();
   private RigidBodyTransform tYaw = new RigidBodyTransform();
   private RigidBodyTransform tRoll = new RigidBodyTransform();
   private RigidBodyTransform tPitch = new RigidBodyTransform();

   YoVariableList freeJointVars;

   public FreeJoint(String jname, Vector3d offset, Robot rob, String xName, String yName, String zName, String yawName, String rollName, String pitchName)
   {
      super(jname, offset, rob, 6);

//    VarList freeJointVars = new VarList(jname + " jointVars"); // rob.getVars();

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();

      this.xName = xName;
      this.yName = yName;
      this.zName = zName;
      this.yawName = yawName;
      this.rollName = rollName;
      this.pitchName = pitchName;

      xVar = new DoubleYoVariable(xName, "FreeJoint x position", registry);
      yVar = new DoubleYoVariable(yName, "FreeJoint y position", registry);
      zVar = new DoubleYoVariable(zName, "FreeJoint z position", registry);
      yawVar = new DoubleYoVariable(yawName, "FreeJoint yaw rotation", registry);
      rollVar = new DoubleYoVariable(rollName, "FreeJoint roll rotation", registry);
      pitchVar = new DoubleYoVariable(pitchName, "FreeJoint pitch rotation", registry);

//    rob.getVars().addVariables(freeJointVars);

      // this.setTransformAndGroup(freeTransform3D());
      setFreeTransform3D(this.jointTransform3D);
   }

   protected YoVariableList getJointVars()
   {
      return this.freeJointVars;
   }

   @Override
   public void update()
   {
       setFreeTransform3D(this.jointTransform3D, xVar.getDoubleValue(), yVar.getDoubleValue(), zVar.getDoubleValue(), yawVar.getDoubleValue(), rollVar.getDoubleValue(), pitchVar.getDoubleValue());
   }


   public void setFreeTransform3D(RigidBodyTransform tTranslate)
   {
      setFreeTransform3D(tTranslate, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   public void setFreeTransform3D(RigidBodyTransform tTranslate, double x, double y, double z, double yaw, double roll, double pitch)
   {
      // Transform3D tTranslate = new Transform3D();
      tTranslate.setIdentity();

      vTranslate.setX(x);
      vTranslate.setY(y);
      vTranslate.setZ(z);
      tTranslate.setTranslation(vTranslate);

      tYaw.setRotationYawAndZeroTranslation(yaw);
      tRoll.setRotationRollAndZeroTranslation(roll);
      tPitch.setRotationPitchAndZeroTranslation(pitch);

      tTranslate.multiply(tYaw);
      tTranslate.multiply(tRoll);
      tTranslate.multiply(tPitch);

      // return tTranslate;

   }

   public void jointDependentSetAndGetRotation(Matrix3d Rh_i)
   {
      Rh_i.setIdentity();
   }

   protected void jointDependentChangeVelocity(double delta_qd)
   {
      System.err.println("Error!!!! FreeJoint.jointDependentChangeVelocity should never be called!!!");
   }


   public void jointDependentFeatherstonePassOne()
   {
   }

   protected void jointDependentSet_d_i()
   {
      System.err.println("Error!!!! FreeJoint.jointDependentSet_d_i should never be called!!!");
   }

   public void jointDependentFeatherstonePassTwo(Vector3d w_h)
   {
   }

   public void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
   }

   public void jointDependentRecordK(int passNumber)
   {
   }

   public void recursiveEulerIntegrate(double stepSize)
   {
   }

   public void recursiveSaveTempState()
   {
   }

   public void recursiveRestoreTempState()
   {
   }

   public void recursiveRungeKuttaSum(double stepSize)
   {
   }

   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      return true;
   }

}
