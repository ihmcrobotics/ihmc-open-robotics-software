package us.ihmc.simulationconstructionset.util;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class SimpleFileReader
{
    BufferedReader fileIn = null;
    FileReader read;

    public SimpleFileReader(File file)
    {
        try
        {
            read = new FileReader(file);
            fileIn = new BufferedReader(read);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
    }
    public String nextLine()
    {
        try
        {
            return fileIn.readLine();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return null;
    }
    public void close()
    {
        try
        {
            if (fileIn != null)
                fileIn.close();
        }
        catch (IOException e)
        {
            System.out.println(e);
        }
    }
}