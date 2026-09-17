package us.ihmc.sensorProcessing.pointClouds.testbed;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import georegression.struct.se.Se3_F64;

/**
 * Reads and writes {@link Se3_F64} in the XML format previously produced by XStream for point-cloud testbed data.
 */
final class Se3F64XmlIO
{
   private Se3F64XmlIO()
   {
   }

   static Se3_F64 read(InputStream inputStream)
   {
      try
      {
         Document document = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(inputStream);
         NodeList rotationValues = document.getElementsByTagName("double");
         if (rotationValues.getLength() < 9)
            throw new IllegalArgumentException("Expected at least 9 rotation values in Se3_F64 XML.");

         Se3_F64 transform = new Se3_F64();
         for (int i = 0; i < 9; i++)
            transform.R.set(i / 3, i % 3, Double.parseDouble(rotationValues.item(i).getTextContent().trim()));

         transform.T.x = parseChildDouble(document, "x");
         transform.T.y = parseChildDouble(document, "y");
         transform.T.z = parseChildDouble(document, "z");
         return transform;
      }
      catch (SAXException | IOException | ParserConfigurationException e)
      {
         throw new RuntimeException(e);
      }
   }

   static void write(Se3_F64 transform, OutputStream outputStream)
   {
      try (Writer writer = new OutputStreamWriter(outputStream, StandardCharsets.UTF_8))
      {
         writer.write("<georegression.struct.se.Se3__F64>\n");
         writer.write("    <R>\n");
         writer.write("        <numRows>3</numRows>\n");
         writer.write("        <numCols>3</numCols>\n");
         writer.write("        <data>\n");
         for (int row = 0; row < 3; row++)
         {
            for (int col = 0; col < 3; col++)
               writer.write("            <double>" + transform.R.get(row, col) + "</double>\n");
         }
         writer.write("        </data>\n");
         writer.write("    </R>\n");
         writer.write("    <T>\n");
         writer.write("        <x>" + transform.T.x + "</x>\n");
         writer.write("        <y>" + transform.T.y + "</y>\n");
         writer.write("        <z>" + transform.T.z + "</z>\n");
         writer.write("    </T>\n");
         writer.write("</georegression.struct.se.Se3__F64>");
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static double parseChildDouble(Document document, String tagName)
   {
      NodeList nodes = document.getElementsByTagName(tagName);
      if (nodes.getLength() == 0)
         throw new IllegalArgumentException("Missing <" + tagName + "> in Se3_F64 XML.");
      return Double.parseDouble(nodes.item(0).getTextContent().trim());
   }
}
