package us.ihmc.robotics.dataStructures;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import us.ihmc.robotics.hierarchicalKinematics.VectorXd;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

public class Vector64F extends DenseMatrix64F
{
   private static final long serialVersionUID = 7043632770776441173L;

   public Vector64F()
   {
      
   }

   public Vector64F(double[] data)
   {
      super(data.length, 1);
      System.arraycopy(data, 0, this.data, 0, data.length);
   }

   public Vector64F(VectorXd other)
   {
      super(other.size(), 1);

      for (int i = 0; i < this.getNumRows(); i++)
      {
         this.set(i, other.get(i));
      }
   }

   public void set(VectorXd other)
   {
      this.reshape( other.size(), 1);

      for (int i = 0; i < this.getNumRows(); i++)
      {
         this.set(i, other.get(i));
      }
   }

   public Vector64F(DenseMatrix64F orig)
   {
      super(orig.numRows, 1);
      if (orig.getNumCols() != 1)
      {
         throw new IllegalArgumentException("The argument of the constructor is not a Vector64f. getNumCols must be 1");
      }
      System.arraycopy(orig.data, 0, this.data, 0, orig.numRows);
   }

   public Vector64F(Vector64F orig)
   {
      super((DenseMatrix64F) orig);
   }

   public Vector64F(int numRows)
   {
      super(numRows, 1);
   }

   public Vector64F(int numRows, double... data)
   {
      super(numRows, 1, true, data);
   }

   public void toEigen(VectorXd other)
   {
      other.resize(this.getNumRows(), 1);
      for (int i = 0; i < this.getNumRows(); i++)
      {
         other.set(i, this.get(i));
      }
   }

   public void fromEigen(VectorXd other)
   {
      this.reshape(other.rows(), 1);
      for (int i = 0; i < this.getNumRows(); i++)
      {
         this.set(i, other.get(i));
      }
   }

   public VectorXd toEigen()
   {
      VectorXd out = new VectorXd(this.getNumRows());
      for (int i = 0; i < this.getNumRows(); i++)
      {
         out.set(i, this.get(i));
      }
      return out;
   }

   public double norm()
   {
      return NormOps.normF(this);
   }

   public void setOnes()
   {
      CommonOps.fill(this, 1);
   }

   public void fill(double val)
   {
      CommonOps.fill(this, val);
   }

   public void setZero()
   {
      this.zero();
   }

   @Override
   public String toString()
   {
      ByteArrayOutputStream stream = new ByteArrayOutputStream();
      PrintStream out = new PrintStream(stream);
      for (int i = 0; i < this.getNumRows(); i++)
      {
         out.format("%.3f\t", this.get(i));
      }
      return stream.toString();
   }

   static public void throwIfContainsNAN( Vector64F vect) throws Exception
   {
      for (int i=0; i<vect.getNumElements(); i++ )
      {
         if( Double.isNaN( vect.get(i) ) )
         {
            throw new Exception("This vector contains NaN");
         }
      }
   }
}
