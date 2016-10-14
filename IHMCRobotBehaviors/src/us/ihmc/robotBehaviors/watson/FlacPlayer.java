package us.ihmc.robotBehaviors.watson;

/* libFLAC - Free Lossless Audio Codec library
 * Copyright (C) 2000,2001,2002,2003  Josh Coalson
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA  02111-1307, USA.
 */

import java.io.EOFException;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Vector;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineListener;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;

import org.kc7bfi.jflac.FLACDecoder;
import org.kc7bfi.jflac.PCMProcessor;
import org.kc7bfi.jflac.metadata.StreamInfo;
import org.kc7bfi.jflac.util.ByteData;

/**
 * Play a FLAC file application.
 * @author kc7bfi
 */
public class FlacPlayer implements PCMProcessor
{
   private AudioFormat fmt;
   private DataLine.Info info;
   private SourceDataLine line;
   private Vector listeners = new Vector();

   public void addListener(LineListener listener)
   {
      listeners.add(listener);
   }
   
   public void playStream(InputStream audioFileStream)
   {
      try
      {
         decode(audioFileStream);
      }
      catch (IOException | LineUnavailableException e)
      {
         e.printStackTrace();
      }
   }
   public void playStream(String inFileName)
   {
      try
      {
         FileInputStream is = new FileInputStream(inFileName);
         decode(is);
      }
      catch (IOException | LineUnavailableException e)
      {
         e.printStackTrace();
      }
      
   }

   /**
    * Decode and play an input FLAC file.
    * @param inFileName    The input FLAC file name
    * @throws IOException  Thrown if error reading file
    * @throws LineUnavailableException Thrown if error playing file
    */
   public void decode(InputStream inputStream) throws IOException, LineUnavailableException
   {
      FLACDecoder decoder = new FLACDecoder(inputStream);
      decoder.addPCMProcessor(this);
      try
      {
         decoder.decode();
      }
      catch (EOFException e)
      {
         // skip
      }

//      line.drain();
      line.close();

      //  We're going to clear out the list of listeners as well, so that everytime through
      //  things are basically at the same starting point.
      listeners.clear();
   }

   /**
    * Process the StreamInfo block.
    * @param streamInfo the StreamInfo block
    * @see org.kc7bfi.jflac.PCMProcessor#processStreamInfo(org.kc7bfi.jflac.metadata.StreamInfo)
    */
   public void processStreamInfo(StreamInfo streamInfo)
   {
      try
      {
         fmt = streamInfo.getAudioFormat();
         info = new DataLine.Info(SourceDataLine.class, fmt, AudioSystem.NOT_SPECIFIED);
         line = (SourceDataLine) AudioSystem.getLine(info);

         //  Add the listeners to the line at this point, it's the only
         //  way to get the events triggered.
         int size = listeners.size();
         for (int index = 0; index < size; index++)
            line.addLineListener((LineListener) listeners.get(index));

         line.open(fmt, AudioSystem.NOT_SPECIFIED);
         line.start();
      }
      catch (LineUnavailableException e)
      {
         e.printStackTrace();
      }
   }

   /**
    * Process the decoded PCM bytes.
    * @param pcm The decoded PCM data
    * @see org.kc7bfi.jflac.PCMProcessor#processPCM(org.kc7bfi.jflac.util.ByteSpace)
    */
   public void processPCM(ByteData pcm)
   {
      line.write(pcm.getData(), 0, pcm.getLen());
   }

   public void removeListener(LineListener listener)
   {
      listeners.removeElement(listener);
   }

   /**
    * The main routine.
    * <p>args[0] is the input file name
    * @param args  Command line arguments
    */
   public static void main(String[] args)
   {

      try
      {
         FlacPlayer decoder = new FlacPlayer();
         decoder.decode(null);
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (LineUnavailableException e)
      {
         e.printStackTrace();
      }

      System.exit(0);
   }
}