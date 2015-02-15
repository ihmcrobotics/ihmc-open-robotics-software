package us.ihmc.utilities.ros;

import java.net.URI;
import java.net.URISyntaxException;

import org.junit.After;
import org.junit.Before;
import org.ros.RosCore;
import org.ros.RosTest;

/**
 *
 * @author tingfan
 * @see RosTest
 */
public abstract class IHMCRosTestWithRosCore
{
   private RosCore rosCore=null;
   protected URI rosMasterURI;

   protected void setUp(boolean USE_JAVA_ROSCORE)
   {
      if (USE_JAVA_ROSCORE)
      {
         rosCore = RosCore.newPrivate();
         rosCore.start();
         rosMasterURI = rosCore.getUri();
      }
      else
      {
         try
         {
            rosMasterURI = new URI("http://localhost:11311/");
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
      }
   }

   @Before
   public void setUp()
   {
      setUp(true);
   }

   @After
   public void tearDown()
   {
      if(rosCore!=null)
         rosCore.shutdown();
   }

}
