package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is used to provides additional properties for cameras.
 */
public class IntrinsicParametersMessage implements Settable<IntrinsicParametersMessage>, EpsilonComparable<IntrinsicParametersMessage>
{
   private int width_;
   private int height_;
   private double fx_;
   private double fy_;
   private double skew_;
   private double cx_;
   private double cy_;
   private us.ihmc.idl.IDLSequence.Double radial_;
   private double t1_;
   private double t2_;

   public IntrinsicParametersMessage()
   {

      radial_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");
   }

   public IntrinsicParametersMessage(IntrinsicParametersMessage other)
   {
      set(other);
   }

   public void set(IntrinsicParametersMessage other)
   {
      width_ = other.width_;

      height_ = other.height_;

      fx_ = other.fx_;

      fy_ = other.fy_;

      skew_ = other.skew_;

      cx_ = other.cx_;

      cy_ = other.cy_;

      radial_.set(other.radial_);
      t1_ = other.t1_;

      t2_ = other.t2_;
   }

   public int getWidth()
   {
      return width_;
   }

   public void setWidth(int width)
   {
      width_ = width;
   }

   public int getHeight()
   {
      return height_;
   }

   public void setHeight(int height)
   {
      height_ = height;
   }

   public double getFx()
   {
      return fx_;
   }

   public void setFx(double fx)
   {
      fx_ = fx;
   }

   public double getFy()
   {
      return fy_;
   }

   public void setFy(double fy)
   {
      fy_ = fy;
   }

   public double getSkew()
   {
      return skew_;
   }

   public void setSkew(double skew)
   {
      skew_ = skew;
   }

   public double getCx()
   {
      return cx_;
   }

   public void setCx(double cx)
   {
      cx_ = cx;
   }

   public double getCy()
   {
      return cy_;
   }

   public void setCy(double cy)
   {
      cy_ = cy;
   }

   public us.ihmc.idl.IDLSequence.Double getRadial()
   {
      return radial_;
   }

   public double getT1()
   {
      return t1_;
   }

   public void setT1(double t1)
   {
      t1_ = t1;
   }

   public double getT2()
   {
      return t2_;
   }

   public void setT2(double t2)
   {
      t2_ = t2;
   }

   @Override
   public boolean epsilonEquals(IntrinsicParametersMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fx_, other.fx_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fy_, other.fy_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.skew_, other.skew_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cx_, other.cx_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cy_, other.cy_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.radial_, other.radial_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.t1_, other.t1_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.t2_, other.t2_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof IntrinsicParametersMessage))
         return false;

      IntrinsicParametersMessage otherMyClass = (IntrinsicParametersMessage) other;

      if (this.width_ != otherMyClass.width_)
         return false;

      if (this.height_ != otherMyClass.height_)
         return false;

      if (this.fx_ != otherMyClass.fx_)
         return false;

      if (this.fy_ != otherMyClass.fy_)
         return false;

      if (this.skew_ != otherMyClass.skew_)
         return false;

      if (this.cx_ != otherMyClass.cx_)
         return false;

      if (this.cy_ != otherMyClass.cy_)
         return false;

      if (!this.radial_.equals(otherMyClass.radial_))
         return false;

      if (this.t1_ != otherMyClass.t1_)
         return false;

      if (this.t2_ != otherMyClass.t2_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IntrinsicParametersMessage {");
      builder.append("width=");
      builder.append(this.width_);

      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);

      builder.append(", ");
      builder.append("fx=");
      builder.append(this.fx_);

      builder.append(", ");
      builder.append("fy=");
      builder.append(this.fy_);

      builder.append(", ");
      builder.append("skew=");
      builder.append(this.skew_);

      builder.append(", ");
      builder.append("cx=");
      builder.append(this.cx_);

      builder.append(", ");
      builder.append("cy=");
      builder.append(this.cy_);

      builder.append(", ");
      builder.append("radial=");
      builder.append(this.radial_);

      builder.append(", ");
      builder.append("t1=");
      builder.append(this.t1_);

      builder.append(", ");
      builder.append("t2=");
      builder.append(this.t2_);

      builder.append("}");
      return builder.toString();
   }
}