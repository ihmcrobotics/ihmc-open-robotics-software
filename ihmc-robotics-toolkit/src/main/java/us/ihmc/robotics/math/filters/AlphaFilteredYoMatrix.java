package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredYoMatrix extends YoMatrix
{
   private final DMatrixRMaj previous;
   private final DMatrixRMaj current;
   private final DMatrixRMaj filtered;

   private final YoDouble alpha;

   public AlphaFilteredYoMatrix(String name, double alpha, int numberOfRows, int numberOfColumns, YoRegistry registry)
   {
      this(name, null, alpha, numberOfRows, numberOfColumns, null, null, registry);
   }

   public AlphaFilteredYoMatrix(String name, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, YoRegistry registry)
   {
      this(name, null, alpha, numberOfRows, numberOfColumns, rowNames, null, registry);
   }

   public AlphaFilteredYoMatrix(String name, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      this(name, null, alpha, numberOfRows, numberOfColumns, rowNames, columnNames, registry);
   }

   public AlphaFilteredYoMatrix(String name, String description, double alpha, int numberOfRows, int numberOfColumns, YoRegistry registry)
   {
      this(name, description, alpha, numberOfRows, numberOfColumns, null, null, registry);
   }

   public AlphaFilteredYoMatrix(String name, String description, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, YoRegistry registry)
   {
      this(name, description, alpha, numberOfRows, numberOfColumns, rowNames, null, registry);
   }

   public AlphaFilteredYoMatrix(String name, String description, double alpha, int numberOfRows, int numberOfColumns, String[] rowNames, String[] columnNames, YoRegistry registry)
   {
      super(name, description, numberOfRows, numberOfColumns, rowNames, columnNames, registry);
      this.alpha = new YoDouble(name + "_alpha", registry);
      this.alpha.set(alpha);

      previous = new DMatrixRMaj(numberOfRows, numberOfColumns);
      current = new DMatrixRMaj(numberOfRows, numberOfColumns);
      filtered = new DMatrixRMaj(numberOfRows, numberOfColumns);
   }

   public void setAlpha(double alpha)
   {
      this.alpha.set(alpha);
   }

   /**
    * Set the current value of the matrix to be filtered.
    * <p>
    * NOTE: This method does not solve for the filtered value. To solve for the filtered value, use {@link #solve()}.
    * </p>
    *
    * @param current the current value of the matrix to be filtered. Not modified.
    */
   @Override
   public void set(DMatrix current)
   {
      super.set(current);
      this.current.set(current);
   }

   /**
    * Assuming that the current value has been set, this method solves for the filtered value.
    * <p>
    * See {@link #set(DMatrix)} for how to set the matrix's current value.
    * </p>
    */
   public void solve()
   {
      filtered.set(previous);
      CommonOps_DDRM.scale(alpha.getDoubleValue(), filtered);

      super.get(current);
      CommonOps_DDRM.addEquals(filtered, 1 - alpha.getDoubleValue(), current);

      // Set the previous value to be the output of the filter, so it can be used next time
      previous.set(filtered);
      super.set(filtered);
   }

   /**
    * Set the current value of the matrix to be filtered and solve for the filtered value.
    *
    * @param current the current value of the matrix to be filtered. Not modified.
    */
   public void setAndSolve(DMatrix current)
   {
      filtered.set(previous);
      CommonOps_DDRM.scale(alpha.getDoubleValue(), filtered);

      super.set(current);
      this.current.set(current);
      CommonOps_DDRM.addEquals(filtered, 1 - alpha.getDoubleValue(), this.current);

      // Set the previous value to be the output of the filter, so it can be used next time
      previous.set(filtered);
      super.set(filtered);
   }
}
