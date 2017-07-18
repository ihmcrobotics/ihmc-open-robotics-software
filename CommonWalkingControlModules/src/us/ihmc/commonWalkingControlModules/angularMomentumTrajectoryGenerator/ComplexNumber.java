package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.text.DecimalFormat;
import java.util.List;

import us.ihmc.commons.PrintTools;

public class ComplexNumber
{
   private static final double TAU = Math.PI*2.0;
   DecimalFormat format = new DecimalFormat("000.000");
   private static double tempVal1;
   private static double tempVal2;
   private double real;
   private double imag;
   
   public ComplexNumber()
   {
      this(0,0);
   }

   public ComplexNumber(double  real, double imag)
   {
      this.real = real;
      this.imag = imag;
   }

   public ComplexNumber(ComplexNumber other)
   {
      this.real = other.real;
      this.imag = other.imag;
   }
   
   public void set(double real, double imag)
   {
      this.real = real;
      this.imag = imag;
   }

   public void set(ComplexNumber other)
   {
      set(other.real, other.imag);
   }
   
   public void setReal(double real)
   {
      this.real = real;
   }
   
   public void setImaginary(double imag)
   {
      this.imag = imag;
   }
   
   public void setToPurelyReal(double real)
   {
      this.real = real;
      this.imag = 0;            
   }
   
   public void setToPurelyImag(double imag)
   {
      this.real = 0;
      this.imag = this.imag;
   }
   
   public void setFromEuler(double magnitude, double argument)
   {
      this.real = magnitude * Math.cos(argument);
      this.imag = magnitude * Math.sin(argument);
   }
   
   public double getRealPart()
   {
      return this.real;
   }
   
   public double getImaginaryPart()
   {
      return this.imag;
   }
   
   private void scale(double scalar)
   {
      this.real *= scalar;
      this.imag *= scalar;
   }
   
   private void setByScaling(ComplexNumber other, double scalar)
   {
      set(other);
      scale(scalar);
   }

   public void add(double real, double imag)
   {
      this.real += real;
      this.imag += imag;
   }
   
   public void add(ComplexNumber other)
   {
      add(other.real, other.imag);
   }
   
   public void add(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1);
      add(c2);
   }
   
   public void subtract(double real, double imag)
   {
      this.real -= real;
      this.imag -= imag;
   }
   
   public void subtract(ComplexNumber other)
   {
      subtract(other.real, other.imag);
   }
   
   public void subtract(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1);
      subtract(c2);
   }
   
   public void multiply(double real, double imag)
   {
      tempVal1 = this.real * real - this.imag * imag;
      this.imag  = this.real * imag + real * this.imag;
      this.real = tempVal1;
   }
   
   public void multiply(ComplexNumber other)
   {
      multiply(other.real, other.imag);
   }
   
   public void multiply(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1);
      multiply(c2);
   }
   
   public void divideBy(double real, double imag)
   {
      multiply(real, -imag);
      scale(1.0/(real*real + imag*imag));
   }
   
   public void divideBy(ComplexNumber other)
   {
      divideBy(other.real, other.imag);
   }
   
   public void divideBy(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1);
      divideBy(c2);
   }

   public void getConjugate(ComplexNumber conjugateToPack)
   {
      conjugateToPack.real = this.real;
      conjugateToPack.imag = -this.imag;
   }
   
   public void setToConjugate()
   {
      getConjugate(this);
   }
   
   public double getMagnitude()
   {
      return (this.real* this.real + this.imag*this.imag);
   }
   
   public double getArgument()
   {
      return Math.atan2(imag, real);
   }
   
   public void getRoots(ComplexNumber root1, ComplexNumber root2)
   {
      tempVal1 = Math.sqrt(getMagnitude());
      tempVal2 = getArgument();
      root1.setFromEuler(tempVal1, tempVal2/2.0);
      root2.setFromEuler(tempVal1, (tempVal2 + 2.0 * Math.PI)/2.0);
   }
   
   public void getRoots(List<ComplexNumber> rootsToPack, int n)
   {
      tempVal1 = Math.pow(getMagnitude(), 1.0/n);
      tempVal2 = getArgument()/n;
      for(int i = 0; i < n; i++)
         rootsToPack.get(i).setFromEuler(tempVal1, tempVal2 + ((double)TAU*i)/(double)n);
   }
   
   public String toString()
   {
      if(imag >= 0)
         return format.format(real) + " + " + format.format(imag) + "i";
      else
         return format.format(real) + " - " + format.format(-imag) + "i";
   }
}

