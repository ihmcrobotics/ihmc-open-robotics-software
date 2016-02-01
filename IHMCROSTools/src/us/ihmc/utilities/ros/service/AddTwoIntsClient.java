package us.ihmc.utilities.ros.service;

import test_ros.AddTwoInts;
import test_ros.AddTwoIntsRequest;
import test_ros.AddTwoIntsResponse;
import us.ihmc.utilities.ros.RosServiceClient;

public class AddTwoIntsClient extends RosServiceClient<AddTwoIntsRequest, AddTwoIntsResponse>
{

   public AddTwoIntsClient()
   {
      super(AddTwoInts._TYPE);
   }
   
   
   public long simpleCall (long a, long b) throws InterruptedException
   {
      AddTwoIntsRequest request = getMessage();
      request.setA(a);
      request.setB(b);
      return  call(request).getSum();
   }

}
