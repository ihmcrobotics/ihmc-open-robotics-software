package us.ihmc.utilities.ros;

import org.ros.exception.ServiceException;

import test_rosmaster.AddTwoIntsResponse;
import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsRequest;

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
