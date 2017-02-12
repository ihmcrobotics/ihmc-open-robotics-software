package us.ihmc.utilities.ros;

import static org.junit.Assert.assertTrue;

import java.net.URISyntaxException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.Before;
import org.junit.Test;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.service.ServiceResponseListener;

import test_rosmaster.AddTwoInts;
import test_rosmaster.AddTwoIntsRequest;
import test_rosmaster.AddTwoIntsResponse;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.utilities.ros.service.AddTwoIntsClient;
import us.ihmc.utilities.ros.service.AddTwoIntsServer;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AddTwoIntsServiceTest extends IHMCRosTestWithRosCore
{
   final static boolean USE_JAVA_ROSCORE = true;

   @Override
   @Before
   public void setUp()
   {
      super.setUp(USE_JAVA_ROSCORE);
   }

   @ContinuousIntegrationTest(estimatedDuration = 2.2)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 2.7)
   @Test(timeout = 30000)
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
         org.junit.Assert.assertEquals(i + 0, answer);
      }
   }
}
