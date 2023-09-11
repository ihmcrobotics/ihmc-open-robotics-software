package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaFilteredElementwiseYoMatrix
{
   private final DMatrixRMaj previous;

   private final DMatrixRMaj current;

   private final DMatrixRMaj filtered;

   private final YoDouble alpha;

   public AlphaFilteredElementwiseYoMatrix(String name, int numberOfRows, int numberOfColumns, double alpha, YoRegistry registry)
   {
      this.alpha = new YoDouble(name + "_alpha", registry);
      this.alpha.set(alpha);

      previous = new DMatrixRMaj(numberOfRows, numberOfColumns);
      current = new DMatrixRMaj(numberOfRows, numberOfColumns);
      filtered = new DMatrixRMaj(numberOfRows, numberOfColumns);
   }

   public AlphaFilteredElementwiseYoMatrix(String name, int numberOfRows, int numberOfColumns, YoRegistry registry)
   {
      this(name, numberOfRows, numberOfColumns, 0.0, registry);  // Default to no filtering if alpha is not supplied
   }

   public void setAndSolve(DMatrixRMaj current)
   {
      filtered.set(previous);
      CommonOps_DDRM.scale(alpha.getDoubleValue(), filtered);

      this.current.set(current);
      CommonOps_DDRM.addEquals(filtered, 1 - alpha.getDoubleValue(), current);

      // Set the previous value to be the output of the filter, so it can be used next time
      previous.set(filtered);
   }

   public DMatrixRMaj getFilteredMatrix()
   {
      return filtered;
   }
}
