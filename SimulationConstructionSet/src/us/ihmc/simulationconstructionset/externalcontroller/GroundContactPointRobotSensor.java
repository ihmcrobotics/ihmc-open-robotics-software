package us.ihmc.simulationconstructionset.externalcontroller;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

class GroundContactPointRobotSensor implements SensorInterface
{
   private final GroundContactPoint groundContactPoint;
   private final YoFramePoint position;
   private final YoFrameVector force;
   
   GroundContactPointRobotSensor(GroundContactPoint groundContactPoint)
   {
      this.groundContactPoint = groundContactPoint;

      position = groundContactPoint.getYoPosition();
      force = groundContactPoint.getYoForce();
   }

   public double[] getMessageValues()
   {
      return new double[]
      {
         force.getX(), force.getY(), force.getZ(), position.getX(), position.getY(), position.getZ(),
         groundContactPoint.getYoFootSwitch().getDoubleValue()
      };

   }


   public String getYoVariableOrder()
   {
      String variableOrder = force.getYoX().getName() + "," + force.getYoY().getName() + "," + force.getYoZ().getName() + "," + position.getYoX().getName() + "," + position.getYoY().getName() + ","
                             + position.getYoZ().getName() + "," + groundContactPoint.getYoFootSwitch().getName();

      return variableOrder;
   }


   public int getNumberOfVariables()
   {
      // TODO Auto-generated method stub
      return 7;
   }


   public void setTau(double tau)
   {
   }

}
