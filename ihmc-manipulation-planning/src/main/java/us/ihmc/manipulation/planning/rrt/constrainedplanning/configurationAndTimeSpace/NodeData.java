package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

public class NodeData
{
   public double[] q;
   private int dimension;

   public NodeData(int dimension)
   {
      this.q = new double[dimension];
      this.dimension = dimension;
   }

   public NodeData(NodeData nodeData)
   {
      this.q = new double[nodeData.getDimension()];
      for (int i = 0; i < nodeData.getDimension(); i++)
      {
         this.q[i] = nodeData.getQ(i);
      }
      this.dimension = nodeData.getDimension();
   }

   public final int getDimension()
   {
      return dimension;
   }

   public final double getQ(int index)
   {
      return q[index];
   }

   public final void setQ(int index, double value)
   {
      this.q[index] = value;
   }

   public final double distance(NodeData nodeData)
   {
      double ret = 0;
      for (int i = 0; i < dimension; i++)
      {
         ret = ret + (nodeData.getQ(i) - this.getQ(i)) * (nodeData.getQ(i) - this.getQ(i));
      }
      ret = Math.sqrt(ret);

      return ret;
   }
}