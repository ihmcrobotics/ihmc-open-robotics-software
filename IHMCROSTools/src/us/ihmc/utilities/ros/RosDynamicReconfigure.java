package us.ihmc.utilities.ros;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;

import dynamic_reconfigure.BoolParameter;
import dynamic_reconfigure.DoubleParameter;
import dynamic_reconfigure.IntParameter;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;
import dynamic_reconfigure.StrParameter;

public class RosDynamicReconfigure
{

   private final RosServiceClient<ReconfigureRequest, ReconfigureResponse> client = new RosServiceClient<ReconfigureRequest, ReconfigureResponse>(
         Reconfigure._TYPE);
   private MessageFactory messageFactory= NodeConfiguration.newPrivate().getTopicMessageFactory();

   public RosDynamicReconfigure(String nodeName, RosMainNode rosMainNode)
   {
      try
      {
         rosMainNode.attachServiceClient(nodeName + "/set_parameters", client);
      }
      catch (Exception e)
      {
         System.err.println("Could Not connect to Node " + nodeName);
      }
   }

   public int setInt(String name, int value)
   {
      client.waitTillConnected();
      ReconfigureRequest request = client.getMessage();
      IntParameter param = messageFactory.newFromType(IntParameter._TYPE);
      param.setName(name);
      param.setValue(value);
      request.getConfig().getInts().add(param);
      ReconfigureResponse response = client.call(request);
      List<IntParameter> newSettings = response.getConfig().getInts();
      for (int i = 0; i < newSettings.size(); i++)
      {
         if (newSettings.get(i).getName().equals(name))
            return newSettings.get(i).getValue();
      }
      throw new RuntimeException("parameter " + name + " not found");
   }

   public String setStr(String name, String value)
   {
      client.waitTillConnected();
      ReconfigureRequest request = client.getMessage();
      StrParameter param = messageFactory.newFromType(StrParameter._TYPE);
      param.setName(name);
      param.setValue(value);
      request.getConfig().getStrs().add(param);
      ReconfigureResponse response = client.call(request);
      List<StrParameter> newSettings = response.getConfig().getStrs();
      for (int i = 0; i < newSettings.size(); i++)
      {
         if (newSettings.get(i).getName().equals(name))
            return newSettings.get(i).getValue();
      }
      throw new RuntimeException("parameter " + name + " not found");
   }

   public boolean setBool(String name, boolean value)
   {
      client.waitTillConnected();
      ReconfigureRequest request = client.getMessage();
      BoolParameter param = messageFactory.newFromType(BoolParameter._TYPE);
      param.setName(name);
      param.setValue(value);
      request.getConfig().getBools().add(param);
      ReconfigureResponse response = client.call(request);
      List<BoolParameter> newSettings = response.getConfig().getBools();
      for (int i = 0; i < newSettings.size(); i++)
      {
         if (newSettings.get(i).getName().equals(name))
            return newSettings.get(i).getValue();
      }
      throw new RuntimeException("parameter " + name + " not found");
   }

   public double setDouble(String name, double value)
   {
      client.waitTillConnected();
      ReconfigureRequest request = client.getMessage();
      DoubleParameter param = messageFactory.newFromType(DoubleParameter._TYPE);
      param.setName(name);
      param.setValue(value);
      request.getConfig().getDoubles().add(param);
      ReconfigureResponse response = client.call(request);
      List<DoubleParameter> newSettings = response.getConfig().getDoubles();
      for (int i = 0; i < newSettings.size(); i++)
      {
         if (newSettings.get(i).getName().equals(name))
            return newSettings.get(i).getValue();
      }
      throw new RuntimeException("parameter " + name + " not found");
   }
   
   public Map<String, Object> getParameters()
   {
      client.waitTillConnected();
      ReconfigureRequest request = client.getMessage();
      ReconfigureResponse response = client.call(request);
      HashMap<String,Object> allSettings = new HashMap<>();
      for(DoubleParameter param :response.getConfig().getDoubles())
         allSettings.put(param.getName(), new Double(param.getValue()));
      for(IntParameter param :response.getConfig().getInts())
         allSettings.put(param.getName(), new Integer(param.getValue()));
      for(StrParameter param :response.getConfig().getStrs())
         allSettings.put(param.getName(), new String(param.getValue()));
      for(BoolParameter param :response.getConfig().getBools())
         allSettings.put(param.getName(), new Boolean(param.getValue()));
      return allSettings;
   }

   public static void main(String[] args) throws URISyntaxException
   {
      RosMainNode mainNode = new RosMainNode(new URI("http://localhost:11311"), "testDC");
      RosDynamicReconfigure testDynamicReconfigure = new RosDynamicReconfigure("/camera/driver", mainNode);
      System.out.println("data_skip:"+testDynamicReconfigure.getParameters().get("data_skip"));
      testDynamicReconfigure.setInt("data_skip", 6);
      System.out.println("data_skip:"+testDynamicReconfigure.getParameters().get("data_skip"));
      System.exit(0);
   }
}
