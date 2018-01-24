package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public abstract class YoFrameTuple implements FixedFrameTuple3DBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble x, y, z;
   private final ReferenceFrame referenceFrame;

   public YoFrameTuple(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(xVariable.getName(), yVariable.getName(), zVariable.getName());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(xVariable.getName(), yVariable.getName(), zVariable.getName());

      this.x = xVariable;
      this.y = yVariable;
      this.z = zVariable;
      this.referenceFrame = referenceFrame;
   }

   public YoFrameTuple(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameTuple(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;
   }

   public abstract void setAndMatchFrame(FrameTuple3DReadOnly frameTuple3DReadOnly);

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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
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

   public final YoDouble getYoX()
   {
      return x;
   }

   public final YoDouble getYoY()
   {
      return y;
   }

   public final YoDouble getYoZ()
   {
      return z;
   }

   public void notifyVariableChangedListeners()
   {
      x.notifyVariableChangedListeners(); // No need to do it for all
   }

   public final void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      x.addVariableChangedListener(variableChangedListener);
      y.addVariableChangedListener(variableChangedListener);
      z.addVariableChangedListener(variableChangedListener);
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple3DString(this) + "-" + referenceFrame;
   }
}