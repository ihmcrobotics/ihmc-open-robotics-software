package us.ihmc.math;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.List;

/**
 * {@link ComplexNumber} implements a complex number and defines complex
 * arithmetic and mathematical functions. A complex number is a combination of a real and unreal number
 * <p>
 * z = u + i * v
 * </p>
 */
public class ComplexNumber implements Settable<ComplexNumber>
{
   private static final double TAU = Math.PI * 2.0;
   private double real, imaginary;

   /**
    * Constructs the complex number  with zero value.
    */
   public ComplexNumber()
   {
      setToZero();
   }

   /**
    * Constructs the complex number z = u + i*v
    *
    * @param real      Real part (u in above equation)
    * @param imaginary imag part (v in above equation)
    */
   public ComplexNumber(double real, double imaginary)
   {
      this.real = real;
      this.imaginary = imaginary;
   }

   /**
    * Constructs this complex number, and sets its value to be equivalent to {@code complexNumber}.
    *
    * @param complexNumber value to set this number to
    */
   public ComplexNumber(ComplexNumber complexNumber)
   {
      this(complexNumber.real, complexNumber.imaginary);
   }

   /**
    * Sets this complex number real and imaginary values.
    *
    * @param real      desired real value
    * @param imaginary desired imaginary value
    */
   public void set(double real, double imaginary)
   {
      this.real = real;
      this.imaginary = imaginary;
   }

   /**
    * Sets this complex number real and imaginary value to equal the other number.
    *
    * @param other desired complex number to copy
    */
   @Override
   public void set(ComplexNumber other)
   {
      set(other.real, other.imaginary);
   }

   /**
    * Sets the real value of this complex number.
    * (the x-coordinate in rectangular coordinates).
    *
    * @param real desired real value
    */
   public void setReal(double real)
   {
      this.real = real;
   }

   /**
    * Sets the imaginary value of this complex number
    * (the y-coordinate in rectangular coordinates).
    *
    * @param imaginary desired imaginary value
    */
   public void setImaginary(double imaginary)
   {
      this.imaginary = imaginary;
   }

   /**
    * Sets the complex number to only a real number, setting the imaginary value to zero.
    * (the x-coordinate in rectangular coordinates with no y-coordinate).
    *
    * @param real desired real value.
    */
   public void setToPurelyReal(double real)
   {
      this.real = real;
      this.imaginary = 0;
   }

   /**
    * Sets the complex number values from its polar coordinates
    *
    * @param magnitude magnitude in polar coordinates
    * @param angle     angle (radians) in polar coordinates
    */
   public void setFromPolar(double magnitude, double angle)
   {
      this.real = magnitude * Math.cos(angle);
      this.imaginary = magnitude * Math.sin(angle);
   }

   /**
    * Sets the real and imaginary values to zero, setting this complex number to the origin.
    */
   public void setToZero()
   {
      this.real = 0.0;
      this.imaginary = 0.0;
   }

   /**
    * Real part of this Complex number
    * (the x-coordinate in rectangular coordinates).
    *
    * @return Re[z] where z is this Complex number.
    */
   public double real()
   {
      return real;
   }

   /**
    * imag part of this Complex number
    * (the y-coordinate in rectangular coordinates).
    *
    * @return Im[z] where z is this Complex number.
    */
   public double imaginary()
   {
      return imaginary;
   }

   /**
    * Modulus of this Complex number
    * (the distance from the origin in polar coordinates).
    *
    * @return |z| where z is this Complex number.
    */
   public double magnitude()
   {
      if (real == 0.0)
      {
         return imaginary;
      }
      else if (imaginary == 0.0)
      {
         return real;
      }
      else
      {
         return EuclidCoreTools.norm(real, imaginary);
      }
   }

   /**
    * Squared value of the modulus of this Complex number
    * (the distance from the origin in polar coordinates).
    *
    * @return |z|^2 where z is this Complex number.
    */
   public double magnitudeSquared()
   {
      if (real == 0.0)
      {
         return imaginary;
      }
      else if (imaginary == 0.0)
      {
         return real;
      }
      else
      {
         return EuclidCoreTools.normSquared(real, imaginary);
      }
   }

   /**
    * Argument of this Complex number
    * (the angle in radians with the x-axis in polar coordinates).
    *
    * @return arg(z) where z is this Complex number.
    */
   public double angle()
   {
      return Math.atan2(imaginary, real);
   }

   /**
    * Complex conjugate of this Complex number
    * (the conjugate of x+i*y is x-i*y).
    *
    * @return z-bar where z is this Complex number.
    */
   public ComplexNumber conjugate()
   {
      return new ComplexNumber(real, -imaginary);
   }

   /**
    * Addition of ComplexNumbers (doesn't change this ComplexNumber).
    * <br>(x+i*y) + (s+i*t) = (x+s)+i*(y+t).
    * <p>
    * WARNING: this generates garbage
    * </p>
    *
    * @param w is the number to add.
    * @return z + w, where z is this ComplexNumber.
    */
   public ComplexNumber plus(ComplexNumber w)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.addEquals(w);
      return complexNumber;
   }

   /**
    * Addition of ComplexNumbers
    * <br> this = z<sub>1</sub> + z<sub>2</sub>.
    *
    * @param z1 is the first number to add. Not modified.
    * @param z2 is the second number to add. Not modified.
    */
   public void add(ComplexNumber z1, ComplexNumber z2)
   {
      set(z1.real() + z2.real(), z1.imaginary() + z2.imaginary());
   }

   /**
    * Addition of ComplexNumbers to this value, stored here.
    * <br> this += other.
    *
    * @param other complex number to add. Not modified.
    */
   public void addEquals(ComplexNumber other)
   {
      addEquals(other.real(), other.imaginary());
   }

   /**
    * Addition of ComplexNumbers to this value, stored here.
    * <br> this += other.
    *
    * @param real      real value of complex number to add.
    * @param imaginary real value of complex number to add.
    */
   public void addEquals(double real, double imaginary)
   {
      this.real += real;
      this.imaginary += imaginary;
   }

   /**
    * Addition of real only ComplexNumbers (doesn't change this ComplexNumber).
    * <br>(x+i*y) + d = (x+d)+i*y.
    * <p>
    * WARNING: this generates garbage
    * </p>
    *
    * @param d real value to add to this complex number.
    * @return z + w, where z is this ComplexNumber.
    */
   public ComplexNumber plus(double d)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.addEquals(d, 0.0);
      return complexNumber;
   }

   /**
    * Addition of ComplexNumbers (doesn't change this ComplexNumber).
    * <br>(x+i*y) - (s+i*t) = (x-s)+i*(y-t).
    * <p>
    * WARNING: this generates garbage
    * </p>
    *
    * @param w is the number to subtract.
    * @return z + w, where z is this ComplexNumber.
    */
   public ComplexNumber minus(ComplexNumber w)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.minusEquals(w);
      return complexNumber;
   }

   /**
    * Subtraction of real only ComplexNumbers (doesn't change this ComplexNumber).
    * <br>(x+i*y) - d = (x-d)+i*y.
    * <p>
    * WARNING: this generates garbage
    * </p>
    *
    * @param d real value to subtract to this complex number.
    * @return z + w, where z is this ComplexNumber.
    */
   public ComplexNumber minus(double d)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.minusEquals(d, 0.0);
      return complexNumber;
   }

   /**
    * Subtraction of ComplexNumbers, stored here.
    * <br> this = c1 - c2.
    *
    * @param c1 complex number to subtract from. Not modified.
    * @param c2 complex number to subtract. Not modified.
    */
   public void minus(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1.real() - c2.real(), c1.imaginary() - c2.imaginary());
   }

   /**
    * Subtraction of ComplexNumbers from this value, stored here.
    * <br> this -= other.
    *
    * @param other complex number to subtract. Not modified.
    */
   public void minusEquals(ComplexNumber other)
   {
      minusEquals(other.real, other.imaginary);
   }

   /**
    * Subtraction of ComplexNumbers from this value, stored here.
    * <br> this -= other.
    *
    * @param real      real value of complex number to subtract.
    * @param imaginary imaginary value of complex number to subtract.
    */
   public void minusEquals(double real, double imaginary)
   {
      this.real -= real;
      this.imaginary -= imaginary;
   }

   /**
    * ComplexNumber multiplication (doesn't change this ComplexNumber).
    *
    * @param w is the number to multiply by.
    * @return z*w where z is this ComplexNumber.
    */
   public ComplexNumber times(ComplexNumber w)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.timesEquals(w);
      return complexNumber;
   }

   /**
    * ComplexNumber multiplication by a real number (doesn't change this ComplexNumber).
    *
    * @param w is the number to multiply by.
    * @return z*w where z is this ComplexNumber.
    */
   public ComplexNumber times(double w)
   {
      ComplexNumber complexNumber = new ComplexNumber(this);
      complexNumber.timesEquals(w, 0.0);
      return complexNumber;
   }

   /**
    * ComplexNumber multiplication, and stores here.
    * this *= other
    *
    * @param other is the number to multiply by. Not modified.
    */
   public void timesEquals(ComplexNumber other)
   {
      this.timesEquals(other.real, other.imaginary);
   }

   /**
    * ComplexNumber multiplication, and stores here.
    * this *= other
    *
    * @param real      is the real value to multiply by.
    * @param imaginary is the real value to multiply by.
    */
   public void timesEquals(double real, double imaginary)
   {
      double tempVal1 = this.real * real - this.imaginary * imaginary;
      this.imaginary = this.real * imaginary + real * this.imaginary;
      this.real = tempVal1;
   }

   /**
    * ComplexNumber multiplication, storing here.
    * this = c1 * c2.
    *
    * @param c1 left side of the multiplication. Not modified.
    * @param c2 right side of the multiplication. Not modified.
    */
   public void times(ComplexNumber c1, ComplexNumber c2)
   {
      set(c1);
      timesEquals(c2);
   }

   /**
    * Division of ComplexNumbers (doesn't change this ComplexNumber).
    * <br>(x+i*y)/(s+i*t) = ((x*s+y*t) + i*(y*s-y*t)) / (s^2+t^2)
    *
    * @param w is the number to divide by
    * @return new ComplexNumber z/w where z is this ComplexNumber
    */
   public ComplexNumber dividedBy(ComplexNumber w)
   {
      double den = MathTools.square(w.magnitude());

      return new ComplexNumber((real * w.real() + imaginary * w.imaginary()) / den, (imaginary * w.real() - real * w.imaginary()) / den);
   }

   /**
    * Scales the magnitude of this complex number, while maintaining the direction
    * this *= scalar.
    *
    * @param scalar amount to scael the magnitude.
    */
   public void scale(double scalar)
   {
      this.real *= scalar;
      this.imaginary *= scalar;
   }

   /**
    * ComplexNumber exponential (doesn't change this ComplexNumber).
    *
    * @return exp(z) where z is this ComplexNumber.
    */
   public ComplexNumber exp()
   {
      return new ComplexNumber(Math.exp(real) * Math.cos(imaginary), Math.exp(real) * Math.sin(imaginary));
   }

   /**
    * Principal branch of the ComplexNumber logarithm of this ComplexNumber.
    * (doesn't change this ComplexNumber).
    * The principal branch is the branch with -pi < arg <= pi.
    *
    * @return log(z) where z is this ComplexNumber.
    */
   public ComplexNumber log()
   {
      return new ComplexNumber(Math.log(this.magnitude()), this.angle());
   }

   /**
    * ComplexNumber square root (doesn't change this ComplexNumber).
    * Computes the principal branch of the square root, which
    * is the value with 0 <= arg < pi.
    *
    * @return sqrt(z) where z is this ComplexNumber.
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
    * Sine of this ComplexNumber (doesn't change this ComplexNumber).
    * <br>sin(z) = (exp(i*z)-exp(-i*z))/(2*i).
    *
    * @return sin(z) where z is this ComplexNumber.
    */
   public ComplexNumber sin()
   {
      return new ComplexNumber(cosh(imaginary) * Math.sin(real), sinh(imaginary) * Math.cos(real));
   }

   /**
    * Cosine of this ComplexNumber (doesn't change this ComplexNumber).
    * <br>cos(z) = (exp(i*z)+exp(-i*z))/ 2.
    *
    * @return cos(z) where z is this ComplexNumber.
    */
   public ComplexNumber cos()
   {
      return new ComplexNumber(cosh(imaginary) * Math.cos(real), -sinh(imaginary) * Math.sin(real));
   }

   /**
    * Hyperbolic sine of this ComplexNumber
    * (doesn't change this ComplexNumber).
    * <br>sinh(z) = (exp(z)-exp(-z))/2.
    *
    * @return sinh(z) where z is this ComplexNumber.
    */
   public ComplexNumber sinh()
   {
      return new ComplexNumber(sinh(real) * Math.cos(imaginary), cosh(real) * Math.sin(imaginary));
   }

   /**
    * Hyperbolic cosine of this ComplexNumber
    * (doesn't change this ComplexNumber).
    * <br>cosh(z) = (exp(z) + exp(-z)) / 2.
    *
    * @return cosh(z) where z is this ComplexNumber.
    */
   public ComplexNumber cosh()
   {
      return new ComplexNumber(cosh(real) * Math.cos(imaginary), sinh(real) * Math.sin(imaginary));
   }

   /**
    * Tangent of this ComplexNumber (doesn't change this ComplexNumber).
    * <br>tan(z) = sin(z)/cos(z).
    *
    * @return tan(z) where z is this ComplexNumber.
    */
   public ComplexNumber tan()
   {
      return (this.sin()).dividedBy(this.cos());
   }

   /**
    * Negative of this ComplexNumber (chs stands for change sign).
    * This produces a new ComplexNumber and doesn't change
    * this ComplexNumber.
    * <br>-(x+i*y) = -x-i*y.
    *
    * @return -z where z is this ComplexNumber.
    */
   public ComplexNumber changeSign()
   {
      return new ComplexNumber(-real, -imaginary);
   }

   public boolean epsilonEquals(double realNumber, double epsilon)
   {
      if (Math.abs(this.real - realNumber) > epsilon)
         return false;
      if (Math.abs(this.imaginary) > epsilon)
         return false;

      return true;
   }

   public boolean epsilonEquals(ComplexNumber complexNumber, double epsilon)
   {
      if (Math.abs(this.real - complexNumber.real()) > epsilon)
         return false;
      if (Math.abs(this.imaginary - complexNumber.imaginary()) > epsilon)
         return false;

      return true;
   }

   public void getRoots(ComplexNumber root1, ComplexNumber root2)
   {
      double tempVal1 = Math.sqrt(magnitude());
      double tempVal2 = angle();
      root1.setFromPolar(tempVal1, tempVal2 / 2.0);
      root2.setFromPolar(tempVal1, (tempVal2 + 2.0 * Math.PI) / 2.0);
   }

   public void getRoots(List<ComplexNumber> rootsToPack, int n)
   {
      double tempVal1 = Math.pow(magnitude(), 1.0 / n);
      double tempVal2 = angle() / n;
      for (int i = 0; i < n; i++)
         rootsToPack.get(i).setFromPolar(tempVal1, tempVal2 + (TAU * i) / (double) n);
   }

   /**
    * String representation of this ComplexNumber.
    *
    * @return x+i*y, x-i*y, x, or i*y as appropriate.
    */
   public String toString()
   {
      if ((real != 0) && (imaginary > 0))
      {
         return real + " + " + imaginary + "i";
      }

      if ((real != 0) && (imaginary < 0))
      {
         return real + " - " + (-imaginary) + "i";
      }

      if (imaginary == 0)
      {
         return String.valueOf(real);
      }

      if (real == 0)
      {
         return imaginary + "i";
      }

      // shouldn't get here (unless Inf or NaN)
      return real + " + i*" + imaginary;
   }
}
