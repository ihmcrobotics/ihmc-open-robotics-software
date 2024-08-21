package us.ihmc.utilities.ros;

import java.net.URI;
import java.net.URISyntaxException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
//import org.ros.RosCore;

/**
 *
 * @author tingfan
 * @see RosTest
 */
public abstract class IHMCRosTestWithRosCore
{
//   private RosCore rosCore=null;
   protected URI rosMasterURI;

   protected void setUp(boolean USE_JAVA_ROSCORE)
   {
//      if (USE_JAVA_ROSCORE)
//      {
//         rosCore = RosCore.newPrivate();
//         rosCore.start();
//         rosMasterURI = rosCore.getUri();
//      }
//      else
//      {
//         try
//         {
//            rosMasterURI = new URI("http://localhost:11311/");
//         }
//         catch (URISyntaxException e)
//         {
//            e.printStackTrace();
//         }
//      }
   }

   @BeforeEach
   public void setUp()
   {
      setUp(true);
   }

   @AfterEach
   public void tearDown()
   {
//      if(rosCore!=null)
//         rosCore.shutdown();
   }

}
