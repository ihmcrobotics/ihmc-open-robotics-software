package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public abstract class YoFrameTuple2d implements FixedFrameTuple2DBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble x, y;
   private final ReferenceFrame referenceFrame;

   public YoFrameTuple2d(YoDouble xVariable, YoDouble yVariable, ReferenceFrame referenceFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(xVariable.getName(), yVariable.getName());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(xVariable.getName(), yVariable.getName());

      this.x = xVariable;
      this.y = yVariable;
      this.referenceFrame = referenceFrame;
   }

   public YoFrameTuple2d(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameTuple2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;
   }

   public abstract void setAndMatchFrame(FrameTuple2DReadOnly frameTuple2DReadOnly);

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

   public final YoDouble getYoX()
   {
      return x;
   }

   public final YoDouble getYoY()
   {
      return y;
   }

   public void setByProjectionOntoXYPlane(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      set(frameTuple3DReadOnly);
   }

   public void notifyVariableChangedListeners()
   {
      x.notifyVariableChangedListeners(); // No need to do it for all
   }

   public final void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      x.addVariableChangedListener(variableChangedListener);
      y.addVariableChangedListener(variableChangedListener);
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
      return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
   }

}