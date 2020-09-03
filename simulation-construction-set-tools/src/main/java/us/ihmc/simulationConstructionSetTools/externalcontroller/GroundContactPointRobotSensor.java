package us.ihmc.simulationConstructionSetTools.externalcontroller;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;

class GroundContactPointRobotSensor implements SensorInterface
{
   private final GroundContactPoint groundContactPoint;
   private final YoFramePoint3D position;
   private final YoFrameVector3D force;
   
   GroundContactPointRobotSensor(GroundContactPoint groundContactPoint)
   {
      this.groundContactPoint = groundContactPoint;

      position = groundContactPoint.getYoPosition();
      force = groundContactPoint.getYoForce();
   }

   @Override
   public double[] getMessageValues()
   {
      return new double[]
      {
         force.getX(), force.getY(), force.getZ(), position.getX(), position.getY(), position.getZ(),
         groundContactPoint.getYoFootSwitch().getDoubleValue()
      };

   }


   @Override
   public String getYoVariableOrder()
   {
      String variableOrder = force.getYoX().getName() + "," + force.getYoY().getName() + "," + force.getYoZ().getName() + "," + position.getYoX().getName() + "," + position.getYoY().getName() + ","
                             + position.getYoZ().getName() + "," + groundContactPoint.getYoFootSwitch().getName();

      return variableOrder;
   }


   @Override
   public int getNumberOfVariables()
   {
      return 7;
   }


   @Override
   public void setTau(double tau)
   {
   }

}
