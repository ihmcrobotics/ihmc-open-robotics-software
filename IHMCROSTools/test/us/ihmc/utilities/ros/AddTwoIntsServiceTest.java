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

import test_ros.AddTwoInts;
import test_ros.AddTwoIntsRequest;
import test_ros.AddTwoIntsResponse;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.utilities.ros.service.AddTwoIntsClient;
import us.ihmc.utilities.ros.service.AddTwoIntsServer;

@BambooPlan(planType = {BambooPlanType.Flaky})
public class AddTwoIntsServiceTest extends IHMCRosTestWithRosCore
{
   
   final static boolean USE_JAVA_ROSCORE=true;
   @Before
   public void setUp()
   {
      super.setUp(USE_JAVA_ROSCORE);
   }

	@EstimatedDuration
	@Test(timeout=5000)
   public void lowlevelRosServiceClientTest  () throws URISyntaxException, InterruptedException
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "serviceClientTestNode");
      final RosServiceClient<AddTwoIntsRequest, AddTwoIntsResponse> serviceClient = new RosServiceClient<>(AddTwoInts._TYPE);
      final AddTwoIntsServer serviceServer = new AddTwoIntsServer();
      rosMainNode.attachServiceClient("/add_two_ints", serviceClient);
      if(USE_JAVA_ROSCORE)
         rosMainNode.attachServiceServer("/add_two_ints", serviceServer);
      rosMainNode.execute();

      serviceClient.waitTillConnected();
      
      int nTry=100;
      final CountDownLatch latch =new CountDownLatch(nTry);
            
      ServiceResponseListener<AddTwoIntsResponse> responseListener
         = new ServiceResponseListener<AddTwoIntsResponse>()
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
      

      for(int i=0;i<nTry;i++)
      {
         AddTwoIntsRequest request = serviceClient.getMessage();
         request.setA(i);
         request.setB(0);
         serviceClient.call(request, responseListener);
      }
      assertTrue(latch.await(1, TimeUnit.SECONDS));

   }
	
	
	@EstimatedDuration
	@Test (timeout=5000)
   public void highLevelRosServiceClientTest() throws InterruptedException
   {
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "serviceClientTestNode");
      final AddTwoIntsClient serviceClient = new AddTwoIntsClient();
      final AddTwoIntsServer serviceServer = new AddTwoIntsServer();
      if(USE_JAVA_ROSCORE)
         rosMainNode.attachServiceServer("/add_two_ints", serviceServer);
      rosMainNode.attachServiceClient("/add_two_ints", serviceClient);
      rosMainNode.execute();

      serviceClient.waitTillConnected();
      
      int nTry=100;
      for(int i=0;i<nTry;i++)
      {
         long answer=serviceClient.simpleCall(i, 0);
         org.junit.Assert.assertEquals(i+0,answer);
      }
   }
	
	
}
