package us.ihmc.valkyrie.kinematics;

public class LinearActuator
{
   private final String name;

   private double position;
   private double velocity;
   private double effort;

   public LinearActuator(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public double getPosition()
   {
      return position;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public double getEffort()
   {
      return effort;
   }

   public void setPositionCommand(double positionCommand)
   {
      this.position = positionCommand;
   }

   public void setVelocityCommand(double velocityCommand)
   {
      this.velocity = velocityCommand;
   }

   public void setEffortCommand(double effortCommand)
   {
      this.effort = effortCommand;
   }
}
