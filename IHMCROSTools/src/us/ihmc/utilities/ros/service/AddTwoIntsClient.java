package us.ihmc.utilities.ros.service;

import test_rosmaster.AddTwoIntsRequest;
import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsResponse;
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
