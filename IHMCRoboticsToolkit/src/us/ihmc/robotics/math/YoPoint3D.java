package us.ihmc.robotics.math;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;

public class YoPoint3D implements Point3DBasics
{
   private final DoubleYoVariable x;
   private final DoubleYoVariable y;
   private final DoubleYoVariable z;
   
    public YoPoint3D(DoubleYoVariable xVariable, DoubleYoVariable yVariable, DoubleYoVariable zVariable)
   {
      this.x = xVariable;
      this.y = yVariable;
      this.z = zVariable;
   }
   
    public YoPoint3D(String namePrefix, YoVariableRegistry registry)
    {
       x = new DoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, ""), registry);
       y = new DoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, ""), registry);
       z = new DoubleYoVariable(YoFrameVariableNameTools.createZName(namePrefix, ""), registry);
    }
    
   public YoPoint3D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new DoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new DoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new DoubleYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
   }
   
   @Override
   public void setX(double x)
   {
      this.x.set(x);
   }

   @Override
   public void setY(double y)
   {
      this.y.set(y);
   }

   @Override
   public void setZ(double z)
   {
      this.z.set(z);
   }

   @Override
   public double getX()
   {
      return x.getDoubleValue();
   }

   @Override
   public double getY()
   {
      return y.getDoubleValue();
   }

   @Override
   public double getZ()
   {
      return z.getDoubleValue();
   }
}
