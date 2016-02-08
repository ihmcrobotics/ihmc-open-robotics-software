package us.ihmc.utilities.ros.service;

import org.ros.exception.ServiceException;

import test_rosmaster.AddTwoIntsResponse;
import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsRequest;
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
