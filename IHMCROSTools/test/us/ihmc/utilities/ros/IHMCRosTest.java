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
public abstract class IHMCRosTest
{
   final boolean USE_JAVA_ROSCORE = true;    // when set to false, needs to run a local roscore externally.
   private RosCore rosCore;
   protected URI rosMasterURI;

   @Before
   public void setUp() throws URISyntaxException
   {
      if (USE_JAVA_ROSCORE)
      {
         rosCore = RosCore.newPrivate();
         rosCore.start();
         rosMasterURI = rosCore.getUri();
      }
      else
      {
         rosMasterURI = new URI("http://localhost:11311/");
      }
   }


   @After
   public void tearDown()
   {
      rosCore.shutdown();
   }

}
