package us.ihmc.controlFlow;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class AbstractControlFlowElement implements ControlFlowElement
{
   private List<ControlFlowInputPort<?>> inputPorts = new ArrayList<ControlFlowInputPort<?>>();
   private List<ControlFlowOutputPort<?>> outputPorts = new ArrayList<ControlFlowOutputPort<?>>();

   public List<ControlFlowInputPort<?>> getInputPorts()
   {
      return Collections.unmodifiableList(inputPorts);
   }

   public List<ControlFlowOutputPort<?>> getOutputPorts()
   {
      return Collections.unmodifiableList(outputPorts);
   }

   protected <T> ControlFlowInputPort<T> createInputPort(String name)
   {
      ControlFlowInputPort<T> ret = new ControlFlowInputPort<T>(name, this);
      registerPort(ret);

      return ret;
   }
   
   protected <T> ControlFlowInputPort<T> createInputPort()
   {
      return createInputPort("unnamedInputPort");
   }

   protected <T> ControlFlowOutputPort<T> createOutputPort(String name)
   {
      ControlFlowOutputPort<T> ret = new ControlFlowOutputPort<T>(name, this);
      registerOutputPort(ret);

      return ret;
   }
   
   protected <T> ControlFlowOutputPort<T> createOutputPort()
   {
      return createOutputPort("unnamedOutputPort");
   }

   public <T> void registerPort(ControlFlowInputPort<T> port)
   {
      if (port.getControlFlowElement() != this)
         throw new RuntimeException("Port is not registered with this control flow element");
      if (inputPorts.contains(port))
         throw new RuntimeException("Port already added");
      inputPorts.add(port);
   }

   public <T> void registerOutputPort(ControlFlowOutputPort<T> port)
   {
      if (port.getControlFlowElement() != this)
         throw new RuntimeException("Port is not registered with this control flow element");
      if (outputPorts.contains(port))
      {
         throw new RuntimeException("Port already added. port = " + port);
      }
      outputPorts.add(port);
   }
}
