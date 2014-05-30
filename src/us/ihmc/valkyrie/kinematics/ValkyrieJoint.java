package us.ihmc.valkyrie.kinematics;

public class ValkyrieJoint implements ValkyrieJointInterface
{
   private final String name;
   private double q;
   private double qd;
   private double f;
   
   private double q_d;
   private double qd_d;
   private double f_d;
   
   public ValkyrieJoint(String name)
   {
      this.name = name;
   }

   @Override
   public void setPosition(double q)
   {
      this.q = q;
   }

   @Override
   public void setVelocity(double qd)
   {
      this.qd = qd;
   }

   @Override
   public void setEffort(double effort)
   {
      this.f = effort;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public double getVelocity()
   {
      return qd;
   }

   @Override
   public double getEffort()
   {
      return f;
   }

   @Override
   public double getPosition()
   {
      return q;
   }

   @Override
   public double getDesiredEffort()
   {
      return f_d;
   }

   @Override
   public void setDesiredEffort(double effort)
   {
      this.f_d = effort;
   }

   @Override
   public double getDesiredPosition()
   {
      return q_d;
   }

   @Override
   public void setDesiredPosition(double position)
   {
      this.q_d = position;
   }

   @Override
   public double getDesiredVelocity()
   {
      return qd_d;
   }

   @Override
   public void setDesiredVelocity(double velocity)
   {
      this.qd_d = velocity;
   }

}
