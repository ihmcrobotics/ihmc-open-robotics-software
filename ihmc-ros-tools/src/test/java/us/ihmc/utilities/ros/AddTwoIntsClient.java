package us.ihmc.utilities.ros;

import test_rosmaster.AddTwoIntsRequest;
import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsResponse;

public class AddTwoIntsClient extends RosServiceClient<AddTwoIntsRequest, AddTwoIntsResponse>
{
   public AddTwoIntsClient()
   {
      super(AddTwoInts._TYPE);
   }

   public long simpleCall (long a, long b)
   {
      AddTwoIntsRequest request = getMessage();
      request.setA(a);
      request.setB(b);
      return  call(request).getSum();
   }
}
