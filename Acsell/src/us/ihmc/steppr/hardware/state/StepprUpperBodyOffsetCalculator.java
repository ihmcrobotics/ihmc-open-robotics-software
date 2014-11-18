package us.ihmc.steppr.hardware.state;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;

public class StepprUpperBodyOffsetCalculator
{
   private static final RigidBodyTransform imuToZUpMatrix = new RigidBodyTransform(new Matrix3d(1, 0, 0, 0, 0, -1, 0, 1, 0), new Vector3d());

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StepprActuatorState imuState;
   private final StepprActuatorState torsoX;
   private final StepprActuatorState torsoY;
   private final StepprActuatorState torsoZ;

   private final StepprXSensState xsens;

   private BooleanYoVariable update;

   private final Quat4d xsensQuat = new Quat4d();
   private final Matrix3d xsensMatrix = new Matrix3d();

   private final Vector3d accel = new Vector3d();
   private final Vector3d mag = new Vector3d();

   private final AlphaFilteredYoVariable q_calc_torso_x;
   private final AlphaFilteredYoVariable q_calc_torso_y;
   private final AlphaFilteredYoVariable q_calc_torso_z;

   private final Vector3d yAxis = new Vector3d();
   private final Vector3d zAxis = new Vector3d();

   private final Matrix3d rotationDifference = new Matrix3d();

   public StepprUpperBodyOffsetCalculator(StepprActuatorState imuState, StepprActuatorState backX, StepprActuatorState backY, StepprActuatorState backZ,
         StepprXSensState xsens, double dt, YoVariableRegistry parentRegistry)
   {
      this.imuState = imuState;
      this.torsoX = backX;
      this.torsoY = backY;
      this.torsoZ = backZ;
      this.xsens = xsens;

      update = new BooleanYoVariable("UpdateUpperBodyOffset", registry);
      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(0.1, dt);

      q_calc_torso_x = new AlphaFilteredYoVariable("q_calc_torso_x", registry, alpha);
      q_calc_torso_y = new AlphaFilteredYoVariable("q_calc_torso_y", registry, alpha);
      q_calc_torso_z = new AlphaFilteredYoVariable("q_calc_torso_z", registry, alpha);

      parentRegistry.addChild(registry);
   }

   private final Matrix3d imuOrientation = new Matrix3d();

   public void update()
   {
      imuState.getIMUAccelerationVector(accel);
      imuState.getIMUMagnetoVector(mag);

      imuToZUpMatrix.transform(accel);
      imuToZUpMatrix.transform(mag);

      mag.normalize();
      accel.normalize();
      imuOrientation.setColumn(0, mag);
      yAxis.cross(mag, accel);
      yAxis.normalize();
      imuOrientation.setColumn(1, yAxis);

      zAxis.cross(mag, yAxis);
      zAxis.normalize();
      imuOrientation.setColumn(2, zAxis);

      xsens.getQuaternion(xsensQuat);
      xsensMatrix.set(xsensQuat);

      rotationDifference.mulTransposeLeft(imuOrientation, xsensMatrix);

      q_calc_torso_x.update(RotationFunctions.getRoll(rotationDifference));
      q_calc_torso_y.update(RotationFunctions.getPitch(rotationDifference));
      q_calc_torso_z.update(RotationFunctions.getYaw(rotationDifference));

      if (update.getBooleanValue())
      {
         update.set(false);
      }
   }

}
