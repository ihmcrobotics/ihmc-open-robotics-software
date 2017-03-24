package us.ihmc.simulationconstructionsettools.externalcontroller;

import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.GroundContactPoint;

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
      // TODO Auto-generated method stub
      return 7;
   }


   @Override
   public void setTau(double tau)
   {
   }

}
