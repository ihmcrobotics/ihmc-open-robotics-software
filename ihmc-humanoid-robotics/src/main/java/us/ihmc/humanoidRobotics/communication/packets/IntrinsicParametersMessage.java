package us.ihmc.humanoidRobotics.communication.packets;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;

public class IntrinsicParametersMessage extends Packet<IntrinsicParametersMessage>
{
   public int width;
   public int height;
   public double fx;
   public double fy;
   public double skew;
   public double cx;
   public double cy;
   public TDoubleArrayList radial = new TDoubleArrayList();
   public double t1;
   public double t2;

   public IntrinsicParametersMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public IntrinsicParametersMessage(IntrinsicParametersMessage other)
   {
      set(other);
   }

   @Override
   public void set(IntrinsicParametersMessage other)
   {
      width = other.width;
      height = other.height;
      fx = other.fx;
      fy = other.fy;
      skew = other.skew;
      cx = other.cx;
      cy = other.cy;
      MessageTools.copyData(other.radial, radial);
      t1 = other.t1;
      t2 = other.t2;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public double getFx()
   {
      return fx;
   }

   public double getFy()
   {
      return fy;
   }

   public double getSkew()
   {
      return skew;
   }

   public double getCx()
   {
      return cx;
   }

   public double getCy()
   {
      return cy;
   }

   public TDoubleArrayList getRadial()
   {
      return radial;
   }

   public double getT1()
   {
      return t1;
   }

   public double getT2()
   {
      return t2;
   }

   public void setWidth(int width)
   {
      this.width = width;
   }

   public void setHeight(int height)
   {
      this.height = height;
   }

   public void setFx(double fx)
   {
      this.fx = fx;
   }

   public void setFy(double fy)
   {
      this.fy = fy;
   }

   public void setSkew(double skew)
   {
      this.skew = skew;
   }

   public void setCx(double cx)
   {
      this.cx = cx;
   }

   public void setCy(double cy)
   {
      this.cy = cy;
   }

   public void setRadial(double[] radial)
   {
      this.radial.add(radial);
   }

   public void setT1(double t1)
   {
      this.t1 = t1;
   }

   public void setT2(double t2)
   {
      this.t2 = t2;
   }

   @Override
   public boolean epsilonEquals(IntrinsicParametersMessage other, double epsilon)
   {
      if (width != other.width)
         return false;
      if (height != other.height)
         return false;
      if (!MathTools.epsilonEquals(fx, other.fx, epsilon))
         return false;
      if (!MathTools.epsilonEquals(fy, other.fy, epsilon))
         return false;
      if (!MathTools.epsilonEquals(skew, other.skew, epsilon))
         return false;
      if (!MathTools.epsilonEquals(cx, other.cx, epsilon))
         return false;
      if (!MathTools.epsilonEquals(cy, other.cy, epsilon))
         return false;
      if (!MathTools.epsilonEquals(t1, other.t1, epsilon))
         return false;
      if (!MathTools.epsilonEquals(t2, other.t2, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(radial, other.radial, epsilon))
         return false;
      return true;
   }
}
