package us.ihmc.utilities.ros.service;

import org.ros.exception.ServiceException;

import test_ros.AddTwoInts;
import test_ros.AddTwoIntsRequest;
import test_ros.AddTwoIntsResponse;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.RosServiceServer;

public class AddTwoIntsServer extends RosServiceServer<AddTwoIntsRequest, AddTwoIntsResponse>
{

   public AddTwoIntsServer()
   {
      super(AddTwoInts._TYPE);
   }
   

   @Override
   public void build(AddTwoIntsRequest request, AddTwoIntsResponse response) throws ServiceException
   {
      response.setSum(request.getA()+request.getB());
      
   }

}
