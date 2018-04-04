package us.ihmc.robotics.math.frames;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQsName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQxName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQyName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createQzName;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

// Note: You should only make these once at the initialization of a controller. You shouldn't make
// any on the fly since they contain YoVariables.
public class YoFrameQuaternion implements FixedFrameQuaternionBasics
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble qx, qy, qz, qs;
   private final ReferenceFrame referenceFrame;

   public YoFrameQuaternion(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, registry);
   }

   public YoFrameQuaternion(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      qx = new YoDouble(createQxName(namePrefix, nameSuffix), registry);
      qy = new YoDouble(createQyName(namePrefix, nameSuffix), registry);
      qz = new YoDouble(createQzName(namePrefix, nameSuffix), registry);
      qs = new YoDouble(createQsName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;

      qs.set(1.0);
   }

   public YoFrameQuaternion(YoDouble qx, YoDouble qy, YoDouble qz, YoDouble qs, ReferenceFrame referenceFrame)
   {
      namePrefix = StringUtils.getCommonPrefix(qx.getName(), qy.getName(), qz.getName(), qs.getName());
      nameSuffix = YoFrameVariableNameTools.getCommonSuffix(qx.getName(), qy.getName(), qz.getName(), qs.getName());

      this.qx = qx;
      this.qy = qy;
      this.qz = qz;
      this.qs = qs;
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      this.qx.set(qx);
      this.qy.set(qy);
      this.qz.set(qz);
      this.qs.set(qs);
   }

   public YoDouble getYoQx()
   {
      return qx;
   }

   public YoDouble getYoQy()
   {
      return qy;
   }

   public YoDouble getYoQz()
   {
      return qz;
   }

   public YoDouble getYoQs()
   {
      return qs;
   }

   @Override
   public double getX()
   {
      return qx.getDoubleValue();
   }

   @Override
   public double getY()
   {
      return qy.getDoubleValue();
   }

   @Override
   public double getZ()
   {
      return qz.getDoubleValue();
   }

   @Override
   public double getS()
   {
      return qs.getDoubleValue();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      qx.addVariableChangedListener(variableChangedListener);
      qy.addVariableChangedListener(variableChangedListener);
      qz.addVariableChangedListener(variableChangedListener);
      qs.addVariableChangedListener(variableChangedListener);
   }

   /**
    * toString
    *
    * String representation of a FrameVector (qx, qy, qz, qs)-reference frame name
    *
    * @return String
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this) + "-" + getReferenceFrame();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }
}
