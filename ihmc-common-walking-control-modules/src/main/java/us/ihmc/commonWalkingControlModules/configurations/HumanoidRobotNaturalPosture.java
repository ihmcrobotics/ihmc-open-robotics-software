package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface HumanoidRobotNaturalPosture
{
   public void initialize();
   
   public Quaternion getNominalStandingPoseQoffset();
   
   public void setNaturalPostureOffset(QuaternionReadOnly Qoffset);

   public double[] getJointPositionArray();

   default void compute(Orientation3DReadOnly Qbase)
   {
      compute(getJointPositionArray(), Qbase);
   }

   public void compute(double[] q, Orientation3DReadOnly Qbase);

   public Quaternion getNaturalPostureQuaternion();

   public Quaternion getNaturalPostureQuaternionrtBase();
   
   public DMatrixRMaj getNaturalPostureJacobian();
   // For testing
   public DMatrixRMaj getNaturalPostureJacobianRtBaseEwrtBase();

   default YoRegistry getRegistry()
   {
      return null;
   }

   public void createVisuals(YoGraphicsListRegistry yoGraphicsListRegistry);
}