package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

/**
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class UnreasonableAccelerationException extends Throwable
{
   private static final long serialVersionUID = 3265459168572596454L;
   private final ArrayList<Joint> unreasonableAccelerationJoints;

   public UnreasonableAccelerationException()
   {
      this.unreasonableAccelerationJoints = null;
   }

   public UnreasonableAccelerationException(ArrayList<Joint> unreasonableAccelerationJoints)
   {
      this.unreasonableAccelerationJoints = unreasonableAccelerationJoints;
   }

   public ArrayList<Joint> getUnreasonableAccelerationJoints()
   {
      return unreasonableAccelerationJoints;
   }
}
