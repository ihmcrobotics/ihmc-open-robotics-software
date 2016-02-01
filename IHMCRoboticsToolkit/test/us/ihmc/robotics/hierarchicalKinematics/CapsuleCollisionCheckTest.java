package us.ihmc.robotics.hierarchicalKinematics;

import org.junit.Test;
import org.w3c.dom.Document;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import us.ihmc.robotics.geometry.Capsule;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point3d;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.ByteArrayInputStream;
import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class CapsuleCollisionCheckTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test() throws ParserConfigurationException, SAXException
   {
      DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
      DocumentBuilder builder = factory.newDocumentBuilder();

      CapsuleCollisionCheck collider = new CapsuleCollisionCheck();

      String filestring = "<?xml version=\"1.0\"?>"  
            + "<robot><link name=\"linkA\"> "
            + "<visual name=\"capsule_p1\">"
            +   "<origin xyz=\"0 0 0\"  />"
            +   "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            + "</link></robot>";
      try{
         Document document = builder.parse( new InputSource(new ByteArrayInputStream(filestring.getBytes("utf-8")))) ;
         collider.loadFromURDF(document);
         fail("this filestring is supposed to fail");
      }
      catch(IOException e) {  System.out.println("EXPECTED ERROR: " + e.getMessage() );  }     

      filestring = "<?xml version=\"1.0\"?>"  
            + "<robot><link name=\"linkA\"> "
            + "<visual name=\"capsule_p2\">"
            +   "<origin xyz=\"0 0 0\"  />"
            +   "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            + "</link></robot>";
      try{
         Document document = builder.parse(  new InputSource(new ByteArrayInputStream(filestring.getBytes("utf-8")))) ;
         collider.loadFromURDF(document);
         fail("this filestring is supposed to fail");
      }
      catch(IOException e) {  System.out.println("EXPECTED ERROR: " + e.getMessage() );  }
      
      filestring = "<?xml version=\"1.0\"?>"  
            + "<robot><link name=\"linkA\"> "
            + "<visual name=\"capsule_p1\">"
            +   "<origin xyz=\"0 0 0\"  />"
            +   "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            + "<visual name=\"capsule_p1\">"
            +   "<origin xyz=\"0 0 0\"  />"
            +   "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            + "</link></robot>";
      try{
         Document document = builder.parse(  new InputSource(new ByteArrayInputStream(filestring.getBytes("utf-8")))) ;
         collider.loadFromURDF(document);
         fail("this filestring is supposed to fail");
      }
      catch(IOException e) {  System.out.println("EXPECTED ERROR: " + e.getMessage() );  }
      
      filestring = "<?xml version=\"1.0\"?>"  
            + "<robot><link name=\"linkA\"> "
            + "<visual name=\"capsule_p1\">"
            +     "<origin xyz=\"  0   0  0  \"  />"
            +     "<geometry> <sphere radius=\"0.12\" /></geometry>"
            + "</visual>"
            + "<visual name=\"capsule_p2\">"
            +     "<origin xyz=\"0 0 0.10\"  />"
            +     "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            + "</link></robot>";
      try{
         Document document = builder.parse(  new InputSource(new ByteArrayInputStream(filestring.getBytes("utf-8")))) ;
         collider.loadFromURDF(document);
         fail("this filestring is supposed to fail");
      }
      catch(IOException e) {  System.out.println("EXPECTED ERROR: " + e.getMessage() );  }
      
      
      filestring = "<?xml version=\"1.0\"?>"  
            + "<robot>"
            + "<link name=\"linkA\"> "
            
            + "<visual name=\"capsule_p1\">"
            +     "<origin xyz=\"  -0.01   -0.02  0  \"  />"
            +     "<geometry> <sphere radius=\"0.1\" /></geometry>"
            + "</visual>"
            + "<visual name=\"capsule_p2\">"
            +     "<origin xyz=\"0.01 0.02 0.10\"  />"
            +     "<geometry> <sphere radius=\"0.10\" /></geometry>"
            + "</visual>"
            
            + "<visual name=\"capsule_p1\">"
            +     "<origin xyz=\"  -0.03   -0.02  -0.20  \"  />"
            +     "<geometry> <sphere radius=\"0.15\" /></geometry>"
            + "</visual>"
            + "<visual name=\"capsule_p2\">"
            +     "<origin xyz=\"0.03 0.02 0.20\"  />"
            +     "<geometry> <sphere radius=\"0.15\" /></geometry>"
            + "</visual>"
            
            + "</link>"
            
            + "<link name=\"linkB\"> "

            + "<visual name=\"capsule_p1\">"
            +     "<origin xyz=\" 0 0  0\"  />"
            +     "<geometry> <sphere radius=\"4\" /></geometry>"
            + "</visual>"
            + "<visual name=\"capsule_p2\">"
            +     "<origin xyz=\"1 2 3\"  />"
            +     "<geometry> <sphere radius=\"4\" /></geometry>"
            + "</visual>"   
            
            + "</link>"
            + "</robot>";
      try{
         Document document = builder.parse(  new InputSource(new ByteArrayInputStream(filestring.getBytes("utf-8")))) ;
         collider.loadFromURDF(document);
         
         double EPS = 0.000001;
         Capsule cap = collider.capsulesByLink.get("linkA").get(0);
         assertTrue( cap.p1.epsilonEquals( new Point3d( -0.01,  -0.02,  0 ),    EPS) );
         assertTrue( cap.p2.epsilonEquals( new Point3d(  0.01,   0.02,  0.10 ), EPS) );
         assertEquals( cap.radius, 0.10, EPS);
         
         cap = collider.capsulesByLink.get("linkA").get(1);
         assertTrue( cap.p1.epsilonEquals( new Point3d( -0.03,  -0.02, -0.20 ),    EPS) );
         assertTrue( cap.p2.epsilonEquals( new Point3d(  0.03,   0.02,  0.20 ), EPS) );
         assertEquals( cap.radius, 0.15, EPS);
         
         cap = collider.capsulesByLink.get("linkB").get(0);
         assertTrue( cap.p1.epsilonEquals( new Point3d( 0, 0, 0 ),    EPS) );
         assertTrue( cap.p2.epsilonEquals( new Point3d( 1, 2, 3 ), EPS) );
         assertEquals( cap.radius, 4, EPS);
      }
      catch(IOException e) {  fail();  }

   }

}
