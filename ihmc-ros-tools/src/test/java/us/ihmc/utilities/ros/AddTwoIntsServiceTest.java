package us.ihmc.utilities.ros;

import static us.ihmc.robotics.Assert.*;

import java.net.URISyntaxException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.service.ServiceResponseListener;

import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsRequest;
import test_rosmaster.AddTwoIntsResponse;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.utilities.ros.service.AddTwoIntsClient;
import us.ihmc.utilities.ros.service.AddTwoIntsServer;

public class AddTwoIntsServiceTest extends IHMCRosTestWithRosCore
{
   final static boolean USE_JAVA_ROSCORE = true;

   @Override
   @BeforeEach
   public void setUp()
   {
      super.setUp(USE_JAVA_ROSCORE);
   }

   @Test
   public void lowlevelRosServiceClientTest() throws URISyntaxException, InterruptedException
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "serviceClientTestNode");
      final RosServiceClient<AddTwoIntsRequest, AddTwoIntsResponse> serviceClient = new RosServiceClient<>(AddTwoInts._TYPE);
      final AddTwoIntsServer serviceServer = new AddTwoIntsServer();
      rosMainNode.attachServiceClient("/add_two_ints", serviceClient);
      if (USE_JAVA_ROSCORE)
         rosMainNode.attachServiceServer("/add_two_ints", serviceServer);
      rosMainNode.execute();

      serviceClient.waitTillConnected();

      int nTry = 100;
      final CountDownLatch latch = new CountDownLatch(nTry);

      ServiceResponseListener<AddTwoIntsResponse> responseListener = new ServiceResponseListener<AddTwoIntsResponse>()
      {

         @Override
         public void onSuccess(AddTwoIntsResponse response)
         {
            latch.countDown();
         }

         @Override
         public void onFailure(RemoteException e)
         {

            throw new RosRuntimeException(e);
         }
      };

      for (int i = 0; i < nTry; i++)
      {
         AddTwoIntsRequest request = serviceClient.getMessage();
         request.setA(i);
         request.setB(0);
         serviceClient.call(request, responseListener);
      }

      assertTrue(latch.await(1, TimeUnit.SECONDS));
   }

   @Test
   public void highLevelRosServiceClientTest() throws InterruptedException
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "serviceClientTestNode");
      final AddTwoIntsClient serviceClient = new AddTwoIntsClient();
      final AddTwoIntsServer serviceServer = new AddTwoIntsServer();
      if (USE_JAVA_ROSCORE)
         rosMainNode.attachServiceServer("/add_two_ints", serviceServer);
      rosMainNode.attachServiceClient("/add_two_ints", serviceClient);
      rosMainNode.execute();

      serviceClient.waitTillConnected();

      int nTry = 100;
      for (int i = 0; i < nTry; i++)
      {
         long answer = serviceClient.simpleCall(i, 0);
         assertEquals(i + 0, answer);
      }
   }
}
