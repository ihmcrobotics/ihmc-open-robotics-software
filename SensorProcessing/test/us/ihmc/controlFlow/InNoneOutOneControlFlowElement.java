package us.ihmc.controlFlow;

public class InNoneOutOneControlFlowElement extends AbstractControlFlowElement
{
   private ControlFlowOutputPort<DataTypeOne> outputPort = createOutputPort();
   private final double x, y;
   
   public InNoneOutOneControlFlowElement(double x, double y)
   {
      this.x = x;
      this.y = y;
   }
   
   public ControlFlowOutputPort<DataTypeOne> getOutputPort()
   {
      return outputPort;
   }

   public void startComputation()
   {
      DataTypeOne output = new DataTypeOne();

      output.setX(x);
      output.setY(y);

      outputPort.setData(output);
   }

   public void waitUntilComputationIsDone()
   {
   }

   public String toString()
   {
      return "InNoneOutOneControlFlowElement\n";
   }

   public void initialize()
   {
//    empty
   }
}
