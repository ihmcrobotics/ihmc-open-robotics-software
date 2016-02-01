package us.ihmc.tools.clover;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import javax.swing.JFileChooser;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class CloverXMLParser
{
   private File xmlFile;

   private void parseXMLFile() throws ParserConfigurationException, SAXException, IOException
   {
      DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();

      DocumentBuilder builder = documentBuilderFactory.newDocumentBuilder();
      Document document = builder.parse(xmlFile);

      Element rootElement = document.getDocumentElement();
      NodeList nodeList = rootElement.getElementsByTagName("class");

      String xmlFileName = xmlFile.getName();
      String csvFileName = xmlFileName.substring(0, xmlFileName.indexOf(".")) + ".csv";
      File csvFile = new File(xmlFile.getParent() + File.separator + csvFileName);
      csvFile.createNewFile();

      FileWriter fileWriter = new FileWriter(csvFile);
      PrintWriter printWriter = new PrintWriter(fileWriter);
      printWriter.println("class name, coverage, complexity");

      for (int i = 0; i < nodeList.getLength(); i++)
      {
         Element classElement = (Element) nodeList.item(i);
         String name = classElement.getAttribute("name");
         Element metricsElement = (Element) classElement.getElementsByTagName("metrics").item(0);
         int elements = Integer.parseInt(metricsElement.getAttribute("elements"));
         int coveredElements = Integer.parseInt(metricsElement.getAttribute("coveredelements"));


         double coverage;
         if (elements > 0)
            coverage = (double) coveredElements / (double) elements;
         else
            coverage = 1.0;

         int complexity = Integer.parseInt(metricsElement.getAttribute("complexity"));

         printWriter.print(name + "," + coverage + "," + complexity + "\n");
      }

      printWriter.close();
      fileWriter.close();
   }

   public void chooseXMLFile()
   {
      JFileChooser fileChooser = new JFileChooser();
      int returnVal = fileChooser.showOpenDialog(null);

      if (returnVal == JFileChooser.APPROVE_OPTION)
      {
         xmlFile = fileChooser.getSelectedFile();
      }
      else
         throw new RuntimeException();
   }

   public void setXMLFile(File xmlFile)
   {
      this.xmlFile = xmlFile;
   }

   public static void main(String[] args) throws ParserConfigurationException, SAXException, IOException
   {
      CloverXMLParser parser = new CloverXMLParser();
      parser.chooseXMLFile();
      parser.parseXMLFile();
   }
}
