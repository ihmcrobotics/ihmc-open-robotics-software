package us.ihmc.robotics.dataStructures;

import us.ihmc.robotics.MathTools;

/**
 *   Complex implements a complex number and defines complex
 *   arithmetic and mathematical functions
 *   Last Updated February 27, 2001
 *   Copyright 1997-2001
 *   @version 1.0
 *   @author Andrew G. Bennett
 */
public class ComplexNumber
{
   private final double real, imag;

   /**
    *   Constructs the complex number z = u + i*v
    *   @param u Real part
    *   @param v imag part
    */
   public ComplexNumber(double real, double imag)
   {
      this.real = real;
      this.imag = imag;
   }

   public ComplexNumber(ComplexNumber complexNumber)
   {
      this.real = complexNumber.real;
      this.imag = complexNumber.imag;
   }

   /**
    *      Real part of this Complex number
    *      (the x-coordinate in rectangular coordinates).
    *      @return Re[z] where z is this Complex number.
    */
   public double real()
   {
      return real;
   }

   /**
    *   imag part of this Complex number
    *   (the y-coordinate in rectangular coordinates).
    *   @return Im[z] where z is this Complex number.
    */
   public double imag()
   {
      return imag;
   }

   /**
    *   Modulus of this Complex number
    *   (the distance from the origin in polar coordinates).
    *   @return |z| where z is this Complex number.
    */
   public double magnitude()
   {
      if ((real != 0.0) || (imag != 0.0))
      {
         return Math.sqrt(real * real + imag * imag);
      }
      else
      {
         return 0.0;
      }
   }


   public double magnitudeSquared()
   {
      if ((real != 0.0) || (imag != 0.0))
      {
         return (real * real + imag * imag);
      }
      else
      {
         return 0.0;
      }
   }

   /**
    *   Argument of this Complex number
    *   (the angle in radians with the x-axis in polar coordinates).
    *   @return arg(z) where z is this Complex number.
    */
   public double angle()
   {
      return Math.atan2(imag, real);
   }

   /**
    *   Complex conjugate of this Complex number
    *   (the conjugate of x+i*y is x-i*y).
    *   @return z-bar where z is this Complex number.
    */
   public ComplexNumber conj()
   {
      return new ComplexNumber(real, -imag);
   }

   /**
    *   Addition of ComplexNumbers (doesn't change this ComplexNumber).
    *   <br>(x+i*y) + (s+i*t) = (x+s)+i*(y+t).
    *   @param w is the number to add.
    *   @return z+w where z is this ComplexNumber.
    */
   public ComplexNumber plus(ComplexNumber w)
   {
      return new ComplexNumber(real + w.real(), imag + w.imag());
   }

   public ComplexNumber plus(double d)
   {
      return new ComplexNumber(real + d, imag);
   }

   /**
    *   Subtraction of ComplexNumbers (doesn't change this ComplexNumber).
    *   <br>(x+i*y) - (s+i*t) = (x-s)+i*(y-t).
    *   @param w is the number to subtract.
    *   @return z-w where z is this ComplexNumber.
    */
   public ComplexNumber minus(ComplexNumber w)
   {
      return new ComplexNumber(real - w.real(), imag - w.imag());
   }

   public ComplexNumber minus(double d)
   {
      return new ComplexNumber(real - d, imag);
   }

   /**
    *   ComplexNumber multiplication (doesn't change this ComplexNumber).
    *   @param w is the number to multiply by.
    *   @return z*w where z is this ComplexNumber.
    */
   public ComplexNumber times(ComplexNumber w)
   {
      return new ComplexNumber(real * w.real() - imag * w.imag(), real * w.imag() + imag * w.real());
   }


   /**
    * ComplexNumber multiplication by a real number (doesn't change this ComplexNumber).
    * @param w is the number to multiply by.
    * @return z*w where z is this ComplexNumber.
    */
   public ComplexNumber times(double w)
   {
      return new ComplexNumber(real * w, imag * w);
   }

   /**
    *   Division of ComplexNumbers (doesn't change this ComplexNumber).
    *   <br>(x+i*y)/(s+i*t) = ((x*s+y*t) + i*(y*s-y*t)) / (s^2+t^2)
    *   @param w is the number to divide by
    *   @return new ComplexNumber z/w where z is this ComplexNumber
    */
   public ComplexNumber dividedBy(ComplexNumber w)
   {
      double den = MathTools.square(w.magnitude());
      
      return new ComplexNumber((real * w.real() + imag * w.imag()) / den, (imag * w.real() - real * w.imag()) / den);
   }

   /**
    *   ComplexNumber exponential (doesn't change this ComplexNumber).
    *   @return exp(z) where z is this ComplexNumber.
    */
   public ComplexNumber exp()
   {
      return new ComplexNumber(Math.exp(real) * Math.cos(imag), Math.exp(real) * Math.sin(imag));
   }

   /**
    *   Principal branch of the ComplexNumber logarithm of this ComplexNumber.
    *   (doesn't change this ComplexNumber).
    *   The principal branch is the branch with -pi < arg <= pi.
    *   @return log(z) where z is this ComplexNumber.
    */
   public ComplexNumber log()
   {
      return new ComplexNumber(Math.log(this.magnitude()), this.angle());
   }

   /**
    *   ComplexNumber square root (doesn't change this ComplexNumber).
    *   Computes the principal branch of the square root, which
    *   is the value with 0 <= arg < pi.
    *   @return sqrt(z) where z is this ComplexNumber.
    */
   public ComplexNumber sqrt()
   {
      double r = Math.sqrt(this.magnitude());
      double theta = this.angle() / 2;

      return new ComplexNumber(r * Math.cos(theta), r * Math.sin(theta));
   }

   // Real cosh function (used to compute ComplexNumber trig functions)
   private double cosh(double theta)
   {
      return (Math.exp(theta) + Math.exp(-theta)) / 2;
   }

   // Real sinh function (used to compute ComplexNumber trig functions)
   private double sinh(double theta)
   {
      return (Math.exp(theta) - Math.exp(-theta)) / 2;
   }

   /**
    *   Sine of this ComplexNumber (doesn't change this ComplexNumber).
    *   <br>sin(z) = (exp(i*z)-exp(-i*z))/(2*i).
    *   @return sin(z) where z is this ComplexNumber.
    */
   public ComplexNumber sin()
   {
      return new ComplexNumber(cosh(imag) * Math.sin(real), sinh(imag) * Math.cos(real));
   }

   /**
    *   Cosine of this ComplexNumber (doesn't change this ComplexNumber).
    *   <br>cos(z) = (exp(i*z)+exp(-i*z))/ 2.
    *   @return cos(z) where z is this ComplexNumber.
    */
   public ComplexNumber cos()
   {
      return new ComplexNumber(cosh(imag) * Math.cos(real), -sinh(imag) * Math.sin(real));
   }

   /**
    *   Hyperbolic sine of this ComplexNumber
    *   (doesn't change this ComplexNumber).
    *   <br>sinh(z) = (exp(z)-exp(-z))/2.
    *   @return sinh(z) where z is this ComplexNumber.
    */
   public ComplexNumber sinh()
   {
      return new ComplexNumber(sinh(real) * Math.cos(imag), cosh(real) * Math.sin(imag));
   }

   /**
    *   Hyperbolic cosine of this ComplexNumber
    *   (doesn't change this ComplexNumber).
    *   <br>cosh(z) = (exp(z) + exp(-z)) / 2.
    *   @return cosh(z) where z is this ComplexNumber.
    */
   public ComplexNumber cosh()
   {
      return new ComplexNumber(cosh(real) * Math.cos(imag), sinh(real) * Math.sin(imag));
   }

   /**
    *   Tangent of this ComplexNumber (doesn't change this ComplexNumber).
    *   <br>tan(z) = sin(z)/cos(z).
    *   @return tan(z) where z is this ComplexNumber.
    */
   public ComplexNumber tan()
   {
      return (this.sin()).dividedBy(this.cos());
   }

   /**
    *   Negative of this ComplexNumber (chs stands for change sign).
    *   This produces a new ComplexNumber and doesn't change
    *   this ComplexNumber.
    *   <br>-(x+i*y) = -x-i*y.
    *   @return -z where z is this ComplexNumber.
    */
   public ComplexNumber changeSign()
   {
      return new ComplexNumber(-real, -imag);
   }

   public boolean epsilonEquals(double realNumber, double epsilon)
   {
      if (Math.abs(this.real - realNumber) > epsilon)
         return false;
      if (Math.abs(this.imag) > epsilon)
         return false;

      return true;
   }

   public boolean epsilonEquals(ComplexNumber complexNumber, double epsilon)
   {
      if (Math.abs(this.real - complexNumber.real()) > epsilon)
         return false;
      if (Math.abs(this.imag - complexNumber.imag()) > epsilon)
         return false;

      return true;
   }

   /**
    *   String representation of this ComplexNumber.
    *   @return x+i*y, x-i*y, x, or i*y as appropriate.
    */
   public String toString()
   {
      if ((real != 0) && (imag > 0))
      {
         return real + " + " + imag + "i";
      }

      if ((real != 0) && (imag < 0))
      {
         return real + " - " + (-imag) + "i";
      }

      if (imag == 0)
      {
         return String.valueOf(real);
      }

      if (real == 0)
      {
         return imag + "i";
      }

      // shouldn't get here (unless Inf or NaN)
      return real + " + i*" + imag;

   }




}
